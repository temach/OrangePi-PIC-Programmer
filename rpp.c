/*
 * Raspberry Pi PIC Programmer using GPIO connector
 * Copyright 2012 Giorgio Vazzana
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Compile: gcc -Wall -O rpp.c -o rpp */

// this enables usleep call
#define _BSD_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdbool.h>
#include <getopt.h>


/* GPIO <-> PIC connections */
#define PIC_VDD    (7)   /* Power */
#define PIC_DATA   (8)    /* Output */
#define PIC_CLK    (9)    /* Output */
#define PIC_PGM    (10)   /* Output */
#define PIC_MCLR   (20)    /* Output */

#define DELAY      (40)   /* microseconds */ // max(T_HLD0, T_HLD1, T_SET1, T_DLY2, T_DLY1)

/* 8K program memory + configuration memory + eeprom data */
// For PIC 16f628a
// 0x0000 - 0x07FF = program memory (0x800 = 2048 words)
// 0x2000 - 0x2007 = configuration mem (7 words)
// 0x2100 - 0x2180 = EEPROM memory (128 bytes)
#define PICMEMSIZE (0x2100 + 0x80)

/* AllWinner H3 datasheet, page 86, page 316 */
// Map from CCU_BASE because it seems that mmap can only map on 0x10000 boundary
uint32_t CCU_BASE = 0x01C20000ul;    /* Port controller */
uint32_t PIO_OFS =  0x00000800ul;

/* AllWinner H3 datasheet, page 86, block size is 1024 = 1K = 0x400 bytes  */
// OR alternatively 4K = 0x2000 = 4096
// This is not reallt end - start anymore,
// because it seems that mmap can only map on 0x10000 boundary
uint32_t PIO_MAP_LEN = 0x2000; /* Port controller end (0x01C20BFF) - Port controller start (0x01C20800)*/

/* AllWinner H3 datasheet, page 316 */
uint32_t PA_CFG0_OFS = 0x00000000ul; /*  Port A Configure Register 0 */
uint32_t PA_CFG1_OFS = 0x00000004ul; /*  Port A Configure Register 1 */
uint32_t PA_CFG2_OFS = 0x00000008ul; /*  Port A Configure Register 2 */
uint32_t PA_CFG3_OFS = 0x0000000Cul; /*  Port A Configure Register 3 (not used) */
uint32_t PA_DAT_OFS  = 0x00000010ul; /*  Port A Data Register */


/*  Prototype for function. This function lives at the end
 *  of the file, because there is no need to change it */
struct picmemory *read_inhx16(char *infile, int debug);

struct picmemory {
    uint16_t  program_memory_used_cells;
    uint16_t  program_memory_max_used_address;

    uint8_t   has_configuration_data;
    uint8_t   has_eeprom_data;

    uint16_t *data;     /* 14-bit data */
    uint8_t  *filled;   /* 1 if this cell is used */
};

struct picmicro {
    uint16_t device_id;
    char     name[16];
    size_t   program_memory_size;
    size_t   data_memory_size;

    int program_cycle_time;     /* in microseconds */   // T_PROG
    int eeprom_program_cycle_time;  // T_DPROG
    int bulk_erase_cycle_time;      // T_ERA

    uint8_t load_configuration_cmd;
    uint8_t load_data_for_program_memory_cmd;
    uint8_t load_data_for_data_memory_cmd;
    uint8_t read_data_from_program_memory_cmd;
    uint8_t read_data_from_data_memory_cmd;
    uint8_t increment_address_cmd;
    uint8_t begin_erase_programming_cycle_cmd;
    uint8_t begin_programming_only_cycle_cmd;
    uint8_t bulk_erase_program_memory_cmd;
    uint8_t bulk_erase_data_memory_cmd;
};

#define PIC16F628A 0x1060

const struct picmicro pic16f628a = {
    /* General */
    .device_id =                          PIC16F628A,
    .name =                               "pic16f628a",
    .program_memory_size =                0x800,
    .data_memory_size =                   128,
    /* Time intervals in microseconds */
    .program_cycle_time =                 4000,
    .eeprom_program_cycle_time =          6000,
    .bulk_erase_cycle_time =              6000,
    /* Commands */
    .load_configuration_cmd =             0x00,
    .load_data_for_program_memory_cmd =   0x02,
    .load_data_for_data_memory_cmd =      0x03,
    .read_data_from_program_memory_cmd =  0x04,
    .read_data_from_data_memory_cmd =     0x05,
    .increment_address_cmd =              0x06,
    .begin_erase_programming_cycle_cmd =  0xFF,
    .begin_programming_only_cycle_cmd =   0x08,
    .bulk_erase_program_memory_cmd =      0x09,
    .bulk_erase_data_memory_cmd =         0x0B
};

const struct picmicro *piclist[] = {&pic16f628a, NULL};

int                mem_fd;
volatile char* gpio;  /* Always use volatile pointer! */

/* AllWinner Pi GPIO setup macros. */
#define GPIO_IN(g)    gpio_input(g)
#define GPIO_OUT(g)   gpio_output(g)

/* AllWinner Pi GPIO */
/* This might need to be INVERTED based on your hardware setup */
// With my setup, I need to invert
#define GPIO_SET(g)   gpio_wr(g, 0)
#define GPIO_CLR(g)   gpio_wr(g, 1)

/*  In the AllWinner H3 datasheet, for registers
 *  bit 0 is LSB (most-right bit). And bits are both
 *  ends inclusive. So bits 5:4 are two bits. */

bool test_bit(uint32_t n, int bit)
{
    return n & (1U << bit);
}

void print_hex(uint32_t n)
{
    for (int i=0; i<32; i++) {
        if (i % 4 == 0)
            printf(" ");
        printf("%c", test_bit(n, 31 - i) ? '1' : '0');
    }
    printf("\n");
}

int gpio_rd(long pin) {
    (void)pin;
    perror("Reading is not supported");
    exit(1);
}

volatile uint32_t* get_data_reg(int pin)
{
    if (pin != 7 && pin != 8 && pin != 9 && pin != 10 && pin != 20) {
        perror("Invalid bit number");
        exit(1);
    }
    volatile uint32_t* data_reg = (volatile uint32_t * )(gpio + PA_DAT_OFS);
    return data_reg;
}

uint32_t modify_data(uint32_t data, int pin, int value)
{
    uint32_t new_data;
    if (value) {
        new_data = data | (1U << pin);
    }
    else {
        new_data = data & ~(1U << pin);
    }
    return new_data;
}

void gpio_wr(int pin, int value) {
    volatile uint32_t* data_reg = get_data_reg(pin);
    uint32_t data = *data_reg;
    uint32_t new_data = modify_data(data, pin, value);
    *(data_reg) = new_data;
}

volatile uint32_t* get_cfg_reg(int pin)
{
    volatile uint32_t* cfg_reg;
    if (pin == 7) {
        cfg_reg = (volatile uint32_t * )(gpio + PA_CFG0_OFS);
    }
    else if (pin == 8 || pin == 9 || pin == 10) {
        cfg_reg = (volatile uint32_t * )(gpio + PA_CFG1_OFS);
    }
    else if (pin == 20) {
        cfg_reg = (volatile uint32_t * )(gpio + PA_CFG2_OFS);
    }
    else {
        perror("Invalid pin number for setting output");
        exit(1);
    }
    return cfg_reg;
}

uint32_t modify_cfg_output(uint32_t cfg, int pin)
{
    /* Write value 001 to position of pin, each pin is z001 ('z' means dont care) */
    uint32_t new_cfg = cfg & ~(6U << 4 * (pin % 8));
    uint32_t new_new_cfg = new_cfg | (1U << 4 * (pin % 8));
    return new_new_cfg;
}

// Pin is one of PA7, PA8, PA9, PA10, PA20
void gpio_output(int pin)
{
    volatile uint32_t* cfg_reg = get_cfg_reg(pin);
    uint32_t cfg = *cfg_reg;
    uint32_t new_cfg = modify_cfg_output(cfg, pin);
    *(cfg_reg) = new_cfg;
}

uint32_t modify_cfg_input(uint32_t cfg, int pin)
{
    /* Write value 000 to position of pin, each pin is z000 ('z' means dont care) */
    uint32_t new_cfg = cfg & ~(7U << 4 * (pin % 8));
    return new_cfg;
}

void gpio_input(int pin)
{
    volatile uint32_t* cfg_reg = get_cfg_reg(pin);
    uint32_t cfg = *cfg_reg;
    uint32_t new_cfg = modify_cfg_input(cfg, pin);
    *(cfg_reg) = new_cfg;
}


/* Set up a memory regions to access GPIO */
void setup_io()
{
    /* open /dev/mem */
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (mem_fd < 0) {
        perror("Cannot open /dev/mem");
        exit(1);
    }

    /* mmap GPIO */
    gpio = mmap(NULL, PIO_MAP_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, CCU_BASE);
    if (gpio == MAP_FAILED) {
        perror("mmap() failed");
        exit(1);
    }
    gpio += PIO_OFS;

    /* Configure GPIOs */
    GPIO_OUT(PIC_CLK);
    GPIO_CLR(PIC_CLK);

    GPIO_OUT(PIC_DATA);
    GPIO_CLR(PIC_DATA);

    GPIO_OUT(PIC_PGM);
    GPIO_CLR(PIC_PGM);

    GPIO_OUT(PIC_MCLR);
    GPIO_CLR(PIC_MCLR);

    GPIO_OUT(PIC_VDD);
    GPIO_CLR(PIC_VDD);

    usleep(DELAY);
}

/* Release GPIO memory region */
void close_io()
{
    int ret;

    GPIO_CLR(PIC_CLK);
    GPIO_CLR(PIC_DATA);
    GPIO_CLR(PIC_PGM);
    GPIO_CLR(PIC_MCLR);
    GPIO_CLR(PIC_VDD);

    /* munmap GPIO */
    ret = munmap((void *)(gpio - PIO_OFS), PIO_MAP_LEN);
    if (ret == -1) {
        perror("munmap() failed");
        exit(1);
    }

    /* close /dev/mem */
    ret = close(mem_fd);
    if (ret == -1) {
        perror("Cannot close /dev/mem");
        exit(1);
    }
}

void free_picmemory(struct picmemory **ppm)
{
    free((*ppm)->data);
    free((*ppm)->filled);
    free(*ppm);
}

/* Send a 6-bit command to the PIC */
void pic_send_cmd(uint8_t cmd)
{
    int i;

    for (i = 0; i < 6; i++) {
        GPIO_SET(PIC_CLK);
        if ((cmd >> i) & 0x01)
            GPIO_SET(PIC_DATA);
        else
            GPIO_CLR(PIC_DATA);
        usleep(DELAY);  /* Setup time */
        GPIO_CLR(PIC_CLK);
        usleep(DELAY);  /* Hold time */
    }
    GPIO_CLR(PIC_DATA);
    usleep(DELAY);
}

/* Load 14-bit data to the PIC (start bit, 14-bit data, stop bit. lsb first) */
void pic_load_data(uint16_t data)
{
    int i;

    /* Insert start and stop bit (0s) */
    data = (data << 1) & 0x7FFE;
    for (i = 0; i < 16; i++) {
        GPIO_SET(PIC_CLK);
        if ((data >> i) & 0x01)
            GPIO_SET(PIC_DATA);
        else
            GPIO_CLR(PIC_DATA);
        usleep(DELAY);  /* Setup time */
        GPIO_CLR(PIC_CLK);
        usleep(DELAY);  /* Hold time */
    }
    GPIO_CLR(PIC_DATA);
    usleep(DELAY);
}

/*  Put pic into low voltage programming mode */
void pic_enter_lvp()
{
    GPIO_SET(PIC_VDD);
    usleep(DELAY);
    GPIO_SET(PIC_PGM);
    usleep(DELAY);
    GPIO_SET(PIC_MCLR);
    usleep(DELAY);
}

/*  Exit from voltage programming mode and turn off the chip power */
void pic_exit_lvp()
{
    /*  The order does not matter. Datasheet does not specify. */
    GPIO_CLR(PIC_MCLR);
    usleep(DELAY);
    GPIO_CLR(PIC_PGM);
    usleep(DELAY);
    /* Turn off power */
    GPIO_CLR(PIC_VDD);
    usleep(DELAY);
}

/* Bulk erase the chip */
void pic_bulk_erase(const struct picmicro *pic, int debug)
{
    pic_enter_lvp();

    fprintf(stderr, "Bulk erasing chip...\n");

    if (debug)
        fprintf(stderr, "  Erasing program and configuration memory...\n");
    // PIC16f628A EEPROM memory programming specification, page 11
    pic_send_cmd(pic->load_data_for_program_memory_cmd);
    pic_load_data(0x3FFF);
    pic_send_cmd(pic->bulk_erase_program_memory_cmd);
    // dont need to send begin_programming_only
    usleep(pic->bulk_erase_cycle_time);
    // This is not strictly needed
    usleep(DELAY);

    if (debug)
        fprintf(stderr, "  Erasing data memory...\n");
    // PIC16f628A EEPROM memory programming spec, page 12
    pic_send_cmd(pic->bulk_erase_data_memory_cmd);
    usleep(pic->bulk_erase_cycle_time);
    usleep(DELAY);

    pic_exit_lvp();
}


/* Bulk erase the chip, and then write contents of the .hex file to the PIC */
void pic_write(const struct picmicro *pic, char *infile, int debug)
{
    uint16_t addr;
    struct picmemory *pm;

    pm = read_inhx16(infile, debug);
    if (!pm)
        return;

    /* Bulk erase the chip first */
    pic_bulk_erase(pic, debug);

    /* Turn pic on. Give supply power. */
    pic_enter_lvp();

    fprintf(stderr, "Writing chip...\n");

    /* Write Program Memory */
    if (debug)
        fprintf(stderr, "  Program memory:\n");
    for (addr = 0; addr <= pm->program_memory_max_used_address; addr++) {
        if (pm->filled[addr]) {
            pic_send_cmd(pic->load_data_for_program_memory_cmd);
            pic_load_data(pm->data[addr]);
            // trigger
            pic_send_cmd(pic->begin_programming_only_cycle_cmd);
            usleep(pic->program_cycle_time);
            if (debug)
                fprintf(stderr, "  addr = 0x%04X  written = 0x%04X \n", addr, pm->data[addr]);
        }
        pic_send_cmd(pic->increment_address_cmd);
    }

    /* Write Configuration Memory */
    if (pm->has_configuration_data) {
        if (debug)
            fprintf(stderr, "  Configuration memory:\n");
        pic_send_cmd(pic->load_configuration_cmd);
        pic_load_data(pm->data[0x2000]);
        for (addr = 0x2000; addr < 0x2008; addr++) {
            if ((addr <= 0x2003 || addr == 0x2007) && pm->filled[addr]) {
                pic_send_cmd(pic->load_data_for_program_memory_cmd);
                pic_load_data(pm->data[addr]);
                // trigger
                pic_send_cmd(pic->begin_programming_only_cycle_cmd);
                usleep(pic->program_cycle_time);
                // debug
                if (debug)
                    fprintf(stderr, "  addr = 0x%04X  written = 0x%04X \n", addr, pm->data[addr]);
            }
            pic_send_cmd(pic->increment_address_cmd);
        }
    }

    // PIC16F628A EEPROM memory programming spec, page 3
    // Since EEPROM uses the lower bits of the same PC,
    // we must reset it to 0 before programming EEPROM.
    // To reset it we re-enter program/verify mode
    pic_exit_lvp();
    usleep(DELAY);
    pic_enter_lvp();

    /* Write Data Memory */
    if (pm->has_eeprom_data) {
        if (debug)
            fprintf(stderr, "  Data memory:\n");
        for (addr = 0x2100; addr < 0x2100 + pic->data_memory_size; addr++) {
            if (pm->filled[addr]) {
                pic_send_cmd(pic->load_data_for_data_memory_cmd);
                pic_load_data(pm->data[addr]);
                pic_send_cmd(pic->begin_programming_only_cycle_cmd);
                usleep(pic->eeprom_program_cycle_time);
                if (debug)
                    fprintf(stderr, "  addr = 0x%04X  written = 0x%04X \n", addr, pm->data[addr]);
            }
            pic_send_cmd(pic->increment_address_cmd);
        }
    }

    pic_exit_lvp();
    free_picmemory(&pm);
}

void usage(void)
{
    const struct picmicro **ppic;
    uint8_t comma = 0;

    fprintf(stderr,
"Usage: rpp [options]\n"
"       -h          print help\n"
"       -D          turn debug on\n"
"       -i file     input file\n"
"       -o file     output file (ofile.hex)\n"
"       -r          read chip\n"
"       -w          bulk erase and write chip\n"
"       -e          bulk erase chip\n"
"\n"
"Supported PICs:"
    );
    for (ppic = piclist; *ppic; ppic++) {
        fprintf(stderr, "%s %s", comma ? "," : "", (*ppic)->name);
        comma = 1;
    }
    fprintf(stderr, "\n");
}

int main(int argc, char *argv[])
{
    int opt, debug = 0, function = 0;
    char *infile  = NULL;
    const struct picmicro *pic;

    fprintf(stderr, "Raspberry Pi PIC Programmer, v0.1\n\n");

    while ((opt = getopt(argc, argv, "hDi:o:rwes")) != -1) {
        switch (opt) {
        case 'h':
            usage();
            exit(0);
            break;
        case 'D':
            debug = 1;
            break;
        case 'i':
            infile = optarg;
            break;
        case 'w':
            function |= 0x02;
            break;
        case 'e':
            function |= 0x04;
            break;
        default:
            fprintf(stderr, "\n");
            usage();
            exit(1);
        }
    }

    if (function == 0x02 && !infile) {
        fprintf(stderr, "Please specify an input file with -i option.\n");
        exit(1);
    }

    /* Setup gpio pointer for direct register access */
    setup_io();

    /* Read PIC device id word */
    pic = &pic16f628a;
    fprintf(stderr, "%s is assumed to be connected\n", pic->name);

    switch (function) {
    case 0x00:
        /* no function selected, exit */
        break;
    case 0x02:
        pic_write(pic, infile, debug);
        break;
    case 0x04:
        pic_bulk_erase(pic, debug);
        break;
    default:
        fprintf(stderr, "\nPlease select only one option in -w, -e.\n");
    };

    close_io();

    return 0;
}

/* Read a file in Intel HEX 16-bit format and return a pointer to the picmemory
   struct on success, or NULL on error */
struct picmemory *read_inhx16(char *infile, int debug)
{
        FILE *fp;
        int linenum;
        char line[256], *ptr;
        size_t linelen;
        int nread;

        uint16_t i;
        uint8_t  start_code;
        uint8_t  byte_count;
        uint16_t address;
        uint8_t  record_type;
        uint16_t data;
        uint8_t  checksum_calculated;
        uint8_t  checksum_read;

        struct picmemory *pm;

        fp = fopen(infile, "r");
        if (fp == NULL) {
                fprintf(stderr, "Error: cannot open source file %s.\n", infile);
                return NULL;
        }

        pm = calloc(1, sizeof(*pm));
        if (pm) {
                pm->data   = calloc(PICMEMSIZE, sizeof(*pm->data));
                pm->filled = calloc(PICMEMSIZE, sizeof(*pm->filled));
        }
        if (!pm || !pm->data || !pm->filled) {
                fprintf(stderr, "Error: calloc() failed.\n");
                return NULL;
        }

        fprintf(stderr, "Reading hex file...\n");

        linenum = 0;
        while (1) {
                ptr = fgets(line, 256, fp);

                if (ptr != NULL) {
                        linenum++;
                        linelen = strlen(line);
                        if (debug) {
                                fprintf(stderr, "  line %d (%zd bytes): '", linenum, linelen);
                                for (i = 0; i < linelen; i++) {
                                        if (line[i] == '\n')
                                                fprintf(stderr, "\\n");
                                        else if (line[i] == '\r')
                                                fprintf(stderr, "\\r");
                                        else
                                                fprintf(stderr, "%c", line[i]);
                                }
                        fprintf(stderr, "'\n");
                        }

                        start_code = line[0];
                        if (start_code != ':') {
                                fprintf(stderr, "Error: invalid start code.\n");
                                free_picmemory(&pm);
                                return NULL;
                        }

                        nread = sscanf(&line[1], "%2hhx", &byte_count);
                        if (nread != 1) {
                                fprintf(stderr, "Error: cannot read byte count.\n");
                                free_picmemory(&pm);
                                return NULL;
                        }
                        if (debug)
                                fprintf(stderr, "  byte_count  = 0x%02X\n", byte_count);

                        if (byte_count % 2 != 0) {
                            fprintf(stderr, "Error: expecting 14-bit data which always comes in multiples of 2 bytes\n");
                            free_picmemory(&pm);
                            return NULL;
                        }


                        nread = sscanf(&line[3], "%4hx", &address);
                        if (nread != 1) {
                                fprintf(stderr, "Error: cannot read address.\n");
                                free_picmemory(&pm);
                                return NULL;
                        }
                        if (debug)
                                fprintf(stderr, "  address     = 0x%04X\n", address);

                        nread = sscanf(&line[7], "%2hhx", &record_type);
                        if (nread != 1) {
                                fprintf(stderr, "Error: cannot read record type.\n");
                                free_picmemory(&pm);
                                return NULL;
                        }
                        if (debug)
                                fprintf(stderr, "  record_type = 0x%02X (%s)\n", record_type, record_type == 0 ? "data" : (record_type == 1 ? "EOF" : "Unknown"));
                        if (record_type != 0 && record_type != 1) {
                                fprintf(stderr, "Error: unknown record type.\n");
                                free_picmemory(&pm);
                                return NULL;
                        }

                        checksum_calculated  = byte_count;
                        checksum_calculated += (address >> 8) & 0xFF;
                        checksum_calculated += address & 0xFF;
                        checksum_calculated += record_type;

                        // NOTE: here we loop (byte_count / 2), becasue of 14-bit wide data
                        for (i = 0; i < byte_count / 2; i++) {
                                nread = sscanf(&line[9+4*i], "%4hx", &data);
                                if (nread != 1) {
                                        fprintf(stderr, "Error: cannot read data.\n");
                                        free_picmemory(&pm);
                                        return NULL;
                                }
                                if (debug)
                                        fprintf(stderr, "  data        = 0x%04X\n", data);
                                checksum_calculated += (data >> 8) & 0xFF;
                                checksum_calculated += data & 0xFF;

                                if (address + i < 0x2000) {
                                        pm->program_memory_used_cells       += 1;
                                        pm->program_memory_max_used_address  = address + i;
                                } else if (0x2000 <= address + i && address + i < 0x2008)
                                        pm->has_configuration_data = 1;
                                else if (address + i >= 0x2100)
                                        pm->has_eeprom_data = 1;

                                pm->data[address + i]   = data;
                                pm->filled[address + i] = 1;
                        }

                        checksum_calculated = (checksum_calculated ^ 0xFF) + 1;

                        nread = sscanf(&line[9+4*i], "%2hhx", &checksum_read);
                        if (nread != 1) {
                                fprintf(stderr, "Error: cannot read checksum.\n");
                                free_picmemory(&pm);
                                return NULL;
                        }
                        if (debug)
                                fprintf(stderr, "  checksum    = 0x%02X\n", checksum_read);

                        if (checksum_calculated != checksum_read) {
                                fprintf(stderr, "Error: checksum does not match.\n");
                                free_picmemory(&pm);
                                return NULL;
                        }

                        if (debug)
                                fprintf(stderr, "\n");

                        if (record_type == 1)
                                break;
                } else {
                        fprintf(stderr, "Error: unexpected EOF.\n");
                        free_picmemory(&pm);
                        return NULL;
                }
        }

        fclose(fp);

        return pm;
}

