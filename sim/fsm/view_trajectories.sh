#!/usr/bin/env bash

trap '
  trap - INT # restore default INT handler
  kill -s INT "$$"
' INT

for file in ./*.csv; do
    ../../../piplot.py -f "$file"
    # test $? -gt 128 && break
done
