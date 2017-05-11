#!/usr/bin/env bash
pwd && doxygen Doxyfile_paper
(cd latex_paper/ && make )
# (cd latex_paper/ && evince refman.pdf & )

latexfolder="/home/artem/Desktop/Uni/coursework-2016-2017-docs/explain/"
latexfile="/home/artem/Desktop/Uni/coursework-2016-2017-docs/explain/class_field_method.tex"

# clear latex file
echo "" > "$latexfile"

# copy files from output into documentation folder and add file names to latex file
for fname in latex_paper/rpp_8c.tex latex_paper/structpicmicro.tex latex_paper/structpicmemory.tex ;
do
  cp "$fname" "${latexfolder}${fname}"
  justname="$(basename -s .tex $fname)"
  echo "\\input{latex_paper/${justname}}" >> "$latexfile"
done
