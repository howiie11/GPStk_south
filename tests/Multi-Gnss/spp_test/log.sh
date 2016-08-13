#!/bin/bash 

# Project 1: Find a good SPP results for station METG 
echo "time: 2016/08/06 UTC"
say -i 'Project 1: Find a good SPP results for station METG'

# Step1: download a month data

station=metg
year=2016
dir_o=./ObsData/obs/
dir_l=./ObsData/nav/
for ((doy=10; doy<=10; doy++))
do

		# download obs data
	getMGEXData.sh -s $station -t "$year $doy" -d "$dir_o" -c "o" 

		# download nav data
	getMGEXData.sh -s $station -t "$year $doy" -d "$dir_l" -c "l" 

done



