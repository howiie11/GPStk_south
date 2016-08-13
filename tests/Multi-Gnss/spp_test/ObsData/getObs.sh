#!/bin/bash

# This script is used to download rinex3 obs files from MGEX
# url: ftp://cddis.gsfc.nasa.gov/pub/gps/data/campaign/mgex/daily/rinex3/
# Written by Lei Zhao, 2016/07/25

# Time
year=2016
doy=005
yr=`echo $year | awk '{print substr($0,3,2)}'`

ty=o

while read sta
do

 wget --tries=10 --wait=10 --timeout=20 -c -k -nd -np -P ./obs/ ftp://cddis.gsfc.nasa.gov/pub/gps/data/campaign/mgex/daily/rinex3/$year/$doy/${yr}${ty}/${sta}*.${yr}${ty}.Z 	

done < sta.l

