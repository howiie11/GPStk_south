#!/bin/bash

# This script is used to get ideal station from a list
# Written by Lei Zhao, WHU, 2016/07/25

dir=ObsData/obs
dir2=ObsData/sta.l

#for rnxName in `ls $dir`
#do
#
#   sta=`echo "$rnxName" | awk '{print substr($0,1,4)}'`
#
##satWatcher ${dir}/$rnxName
#
#done


while read line 
do

	totalPath=`echo "${dir}/${line}0050.16o"`
	echo "$totalPath"

	satWatcher $totalPath 


done < $dir2
