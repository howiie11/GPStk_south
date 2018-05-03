#!/bin/bash
#




if [ $# -eq 1 ]; then

 echo "$1"

	GSats=(G13 G15 G16 G21 G27 G29)
	for prn in ${GSats[@]}
	do

		echo "sat=$prn"
		sat=$prn
		echo "sys=${prn:0:1}"
		sys=${prn:0:1}
		echo "num=${prn:1}"
		num=${prn:1}
		declare system
		declare keyword

		if [ "$sys" == "G" ];then
			system=GPS	
		fi

		keyword=`echo "$system $num"` 
		echo "keyword=$keyword"

		grep -w "$keyword" $1 > ${1}.${sat}

		filename=${1}.${sat}
		echo "$filename"

		# Plot
	   gnuplot << EOF
	   set terminal png
		set grid
		set xdata time
		set timefmt "%s"
		set format x "%H:%M"
		set xtics 10800
	   set output '${filename}.png'
	   set ylabel 'value(m)'
	#   set y2label 'sat number'
		set ytics nomirror
	#	set y2tics
		set xlabel 'GPS Time'
	   set size 1.0, 1.0
	#	set y2range [0:15]
	#	set key left
	   set grid
	#unset key
		plot "${filename}" u 3:7 w p pt 3  t "dLI", "${filename}" u 3:9 w p pt 4  t "d2LI" 
EOF

	open ${filename}.png

	done # End of 'for prn in ${GSats[@]}' 

else 
	
	echo "Usage: $0 <result file of pppar>"

fi

#set yr[-1:1]
#set xlabel ' Second of day'


#set xdata time
#set timefmt "%S"
#set format x "%H:%M"
#plot "$1" using 3:7  with linespoints linewidth 1 linecolor 1
