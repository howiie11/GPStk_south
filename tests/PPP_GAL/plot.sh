#!/bin/bash
#
if [ $# -eq 1 ]; then

	# Delete the first five lines
	awk '{ if(NR > 4) {print $0}}' $1 > tempfile

	# Plot
   gnuplot << EOF
   set terminal png
   set output '$1.png'
   set xlabel ' Second of day'
   set ylabel 'Error regard to IGS nominal(m)'
   set size 1.0, 1.0
	set yr[-1:1]
   set grid

	plot "tempfile" using 3:4  with linespoints linewidth 1 linecolor 1  title "dn", \
       "tempfile" using 3:5  with linespoints linewidth 1 linecolor 7  title "de", \
       "tempfile" using 3:6  with linespoints linewidth 1 linecolor 2  title "du"
EOF

#rm tempfile

else 
	
	echo "Usage: $0 <result file of pppar>"

fi

#plot "tempfile" using 3:4  with linespoints linewidth 1 linecolor 1  title "dn", \
#       "tempfile" using 3:5  with linespoints linewidth 1 linecolor 7  title "de", \
#       "tempfile" using 3:6  with linespoints linewidth 1 linecolor 2  title "du"

#set yr[-1:1]
