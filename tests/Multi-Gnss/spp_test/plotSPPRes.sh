#!/bin/bash

# This script is used to plot SPP result
# Written by Lei Zhao, 2016/07/21, WHU


if [ $# -eq 1 ]; then

	echo "Code here! "
	gnuplot << EOF

	 set terminal png
    set xlabel "Seconds of Day (s)"
    set ylabel "Error Regarding IGS Nominal (m)"

    set output "$1.png"
    set title "$1 "
    plot "$1" using 3:4 with p pt 7 title "dN", \
         "$1" using 3:5 with p pt 8 title "dE", \
         "$1" using 3:6 with p pt 9 title "dU"

EOF

else

	echo -e "Usage:<$0> <result.pos>"
fi
