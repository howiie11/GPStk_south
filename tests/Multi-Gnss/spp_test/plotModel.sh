#!/bin/bash


# This script is used to plot model file

if [ $# -eq 1 ]; then

	echo "Code here! "
	gnuplot << EOF

	 set terminal png
    set xlabel "Seconds of Day (s)"
    set ylabel "postfitC (m)"
	 unset key
    set output "$1.png"
    set title "$1 "
    plot "$1" using 3:7 with lines

EOF

else

	echo -e "Usage:<$0> <result.pos>"
fi
