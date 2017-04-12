#!/bin/bash

ln -f -s ../../tables/PRN_GPS PRN_GPS

proj="pppTest"
#proj="pppGALTest"
#
###############################
## get the rinex file list
###############################
inputDataDir2="/Users/leizhao/Data/MGEX/allPresentStations/"
inputDataDir="/Users/leizhao/Data/MGEX/"
resultDataDir="/Users/leizhao/Data/resultData/"
#
# Please note that TEQC can not handle rinex version >=2.11
# Station	Description
# TLSE  num of Galileo sat is too few. 
# MAS1  inconsistency of key word " SYS / PHASE SHIFT"
# KIRU  this station observes more Galileo satellites 
> $proj.mgexRnxList
year=2015
month=3
startDay=17
endDay=18
#get_rnx.sh -b "2016 5 5 0 0 0" -e "2016 5 6 0 0 0" -i 24 -a "MGEX_IGN" -u "../../tables/url.list" -s "$proj.stalist.test" -l "$proj" -p "$inputDataDir"  
#get_rnx.sh -b "2016 4 22 0 0 0" -e "2016 5 12 0 0 0" -i 24 -a "MGEX" -u "../../tables/url.list" -s "$proj.stalist.test" -l "$proj" -p "$inputDataDir"  
get_rnx.sh -b "$year $month $startDay 0 0 0" -e "$year $month $endDay 0 0 0" -i 24 -a "MGEX" -u "../../tables/url.list" -s "$proj.stalist.test" -l "$proj" -p "$inputDataDir2"  
#get_rnx.sh -b "2016 5 5 0 0 0" -e "2016 5 6 0 0 0" -i 24 -a "MGEX" -u "../../tables/url.list" -s "station.temp" -l "$proj" -p "$inputDataDir2"  
#
###############################
## get the ephemeris file list
###############################
#> $proj.ephlist
#> $proj.clklist
#> $proj.erplist
#> $proj.ssclist
get_eph.sh -b "$year $month $startDay 0 0 0" -e "$year $month $endDay 0 0 0" -i 6 -a "CNES" -u "../../tables/url.list" -t "type.list.gfz" -l "$proj" -p "$inputDataDir" 
#get_eph.sh -b "2016 4 22 0 0 0" -e "2016 5 12 0 0 0" -i 6 -a "GFZ" -u "../../tables/url.list" -t "type.list" -l "$proj" -p "$inputDataDir" 

# Download coordinate product
#get_eph.sh -b "2016 5 5 0 0 0" -e "2016 5 6 0 0 0" -i 6 -a "CNES" -u "../../tables/url.list" -t "type.list2" -l "$proj" -p "$inputDataDir" 

# Download erp file from IGS
#get_eph.sh -b "2016 4 22 0 0 0" -e "2016 5 12 0 0 0" -i 6 -a "IGS" -u "../../tables/url.list" -t "type.list.erp" -l "$proj" -p "$inputDataDir" 
get_eph.sh -b "$year $month $startDay 0 0 0" -e "$year $month $endDay 0 0 0" -i 6 -a "IGS" -u "../../tables/url.list" -t "type.list.erp" -l "$proj" -p "$inputDataDir" 

# Download ssc file from IGS
#get_eph.sh -b "2016 4 22 0 0 0" -e "2016 5 12 0 0 0" -i 6 -a "IGS" -u "../../tables/url.list" -t "type.list.ssc" -l "$proj" -p "$inputDataDir" 
get_eph.sh -b "$year $month $startDay 0 0 0" -e "$year $month $endDay 0 0 0" -i 6 -a "IGS" -u "../../tables/url.list" -t "type.list.ssc" -l "$proj" -p "$inputDataDir" 

# 
# Download snx from EPN
# 
#get_eph.sh -b "2016 5 5 0 0 0" -e "2016 5 6 0 0 0" -i 24 -a "EPN" -u "../../tables/url.list" -t "ssc.list.epn" -l "$proj" -p "$inputDataDir"



##############################
# get the msc file list
##############################
# remove all the msc file in the dir
result=""
for file in `ls ./`
do
#	echo $file
	result=`echo $file | awk 'BEGIN{ i=0};{ if(match($1,"msc")) {print i+1}}'`
	if [ "$result" == "1" ]; then
		break
	fi
done

#echo "result=$result"
if [ "$result" == "1" ]; then
	rm *.msc
fi

# convert ssc2msc
#cat $proj.mgexSscList | while read line
cat $proj.ssclist | while read line
do
#	echo "line=$line"
   ssc2msc -s $line 
done

> msc.txt
# now, let's merge all the msc files together for pppar positioning
ls *.msc | while read line
do
#   echo $line;
   cat $line >> msc.txt
done

#############################
# get the output file list
# 
# The format of output file suffix 
# .out.sys.ambRes.state 
# example: .out.
##############################
> $proj.mgexOutList
cat $proj.mgexRnxlist | while read line
do
	fileName=`basename $line`
#	echo "$fileName"
	dirName=`echo "$resultDataDir$fileName"`	
#	echo "$dirName"
   echo "$dirName".out.float.kin.test.G >> $proj.mgexOutList
done

# now, Let's perform the ppp positioning
#ppp_gal -r $proj.mgexRnxList -s $proj.mgexEphList -k $proj.mgexClkList -e $proj.mgexErpList -m msc.txt -D $proj.dcb_p1c1list -o $proj.mgexOutList
#ppp_gal -r $proj.mgexRnxList -s $proj.mgexEphList -k $proj.mgexClkList -e $proj.mgexErpList -m msc.txt -o $proj.mgexOutList
pppMGEX -r $proj.mgexRnxList -s $proj.mgexEphList -k $proj.mgexClkList -e $proj.erplist -m msc.txt -o $proj.mgexOutList

