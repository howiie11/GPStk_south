#!/bin/bash

# This script is used to download navigation files provided by MGEX 
# url: ftp://cddis.gsfc.nasa.gov/pub/gps/data/campaign/mgex/daily/rinex3/
# Written by Lei Zhao, WHU, 2015/07/25

# Time
year=2016
doy=005
yr=`echo $year | awk '{print substr($0,3,2)}'`

# Navigation system
sys=l

wget --tries=10 --wait=10 --timeout=20 -c -k -nd -np -P ./nav/${sys}/ ftp://cddis.gsfc.nasa.gov/pub/gps/data/campaign/mgex/daily/rinex3/$year/$doy/${yr}${sys}/*.${yr}${sys}.Z




