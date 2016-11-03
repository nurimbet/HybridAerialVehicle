#!/usr/bin/gnuplot
# Gnuplot script file for plotting data in file "output.txt"
# This file is called ".gnu"
set   autoscale                        # scale axes automatically
unset log                              # remove any log-scaling
unset label                            # remove any previous labels
set xtic auto                          # set xtics automatically
set ytic auto                          # set ytics automatically
set title "XYZ"
set xlabel "Sample"
set ylabel "Position (M)"
set grid
plot 'result.txt' using 0:1 title 'X' with lines, 'result.txt' using 0:2 title 'Y' with lines, 'result.txt' using 0:3 title 'Z' with lines
pause -1 "Hit return to continue"
