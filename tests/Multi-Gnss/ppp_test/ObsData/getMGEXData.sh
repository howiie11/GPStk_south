#!/bin/bash 

# This script is used to auotmatedly download MGEX Obs data with specified 
# Parameters

if [ $# -eq 8 ]; then

	echo "Clear"
	echo "1st para: $1" # -s
	echo "2nd para: $2" 
	echo "3rd para: $3" # -t
	echo "4th para: $4"
	echo "5th para: $5" # -d 
	echo "6th para: $6"
	echo "7th para: $7" # -c
	echo "8th para: $8"

#		# Put all the present para in an array
#	array=($1 $2 $3 $4 $5 $6 $7 $8)
#
#		# Get station name, year and doy
#	for para in {-s,-t,-d,-c}
#	do
#		echo "para=$para"
#		pos=0
#			# loop the present para
#		for ((i=0; i<8; i++))
#		do
#			tmp=`echo ${array[$i]}`
#			if [ $tmp == $para ]; then
#				pos=$i	
#			fi
#		done
#
#		
#	done

	sta=$2;
	station=`echo $sta | tr 'A-Z' 'a-z' `
	timeStr=$4
	year=`echo $timeStr | awk '{print $1}'`
   yr=`echo "$year" | awk '{print substr($0, 3, 2)}'`
	month=`echo $timeStr | awk '{print $2}'`
	day=`echo $timeStr | awk '{print $3}'`
	hour=`echo $timeStr | awk '{print $4}'`
	minute=`echo $timeStr | awk '{print $5}'`
	second=`echo $timeStr | awk '{print $6}'`
	d=`timeconvert -c "$month $day $year $hour $minute $second" -F "%j"`
#	d=`echo $4 | awk '{print $2}'`
	if [ $d -lt 10 ];then
		doy=`echo "00$d"`
	elif [ $d -lt 100  ];then
		doy=`echo "0$d"`
	else
		doy=$d
	fi
	path="$6"
	ty=$8

	echo -e "station=$station\nyear=$year\ndoy=$doy\nty=$ty"

		# Now we download obs data from one of the MGEX provider:
		# url: ftp://cddis.gsfc.nasa.gov/pub/gps/data/campaign/mgex/daily/rinex3/
	wget --tries=10 --wait=10 --timeout=20 -c -k -nd -np -P $path ftp://cddis.gsfc.nasa.gov/pub/gps/data/campaign/mgex/daily/rinex3/$year/$doy/${yr}${ty}/${station}*.${yr}${ty}.Z 

		# uncompress 
	uncompress ${path}${station}*.${yr}${ty}.Z  
else

	echo -e "Usage: <$0> <-s> <station name> <-t1> <"civiltime"> \n<-d> <path> <-c> <o,n,l,...>\ne.g. $0 -s "metg" -t1 "2016 1 4 0 0 0" -d "./" -c "o" "
fi
