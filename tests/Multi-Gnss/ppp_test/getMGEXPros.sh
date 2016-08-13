#!/bin/bash

# This script is used to download products for MGEX
# from the server: ftp://cddis.gsfc.nasa.gov/pub/gps/products/mgex/ 

if [ $# -eq 8 ]; then

	echo "1st para: $1" # agency 
	echo "2nd para: $2"
	echo "3rd para: $3" # time
	echo "4th para: $4"
	echo "5th para: $5" # product type
	echo "6th para: $6"
	echo "7th para: $7" # path
	echo "8th para: $8"

	agency=$2
	ty=$6
	timeStr=$4
	year=`echo $timeStr | awk '{print $1}'`
	month=`echo $timeStr | awk '{print $2}'`
	day=`echo $timeStr | awk '{print $3}'`
	hour=`echo $timeStr | awk '{print $4}'`
	minute=`echo $timeStr | awk '{print $5}'`
	second=`echo $timeStr | awk '{print $6}'`

	gpsweek=`timeconvert -c "$month $day $year $hour $minute $second" -F "%F" `
	doy=`timeconvert -c "$month $day $year $hour $minute $second" -F "%j"`
	dow=`timeconvert -c "$month $day $year $hour $minute $second" -F "%w"`

	path=$8
	echo -e "agency=$agency\nty=$ty\ntimeStr=$timeStr\nyear=$year\nmonth=$month\nday=$day\nhour=$hour\nminute=$minute\nsecond=$second\ngpsweek=$gpsweek\ndoy=$doy\ndow=$dow"

		# download
	wget --tries=10 --wait=10 --timeout=20 -c -k -nd -np -P $path ftp://cddis.gsfc.nasa.gov/pub/gps/products/mgex/$gpsweek/${agency}${gpsweek}${dow}.${ty}.Z

		# uncompress
	uncompress ${path}${agency}${gpsweek}${dow}.${ty}.Z
else

	echo -e "Usage: <$0> <-a> <"agency name\(...\)"> <-t1> <"civilTime"> \n       <-c> <"product type\(...\)"> <-d> <path>\ne.g. getMGEXPros.sh -a "com" -t1 "2016 1 5 0 0 0" -c erp -d "./""

fi 
