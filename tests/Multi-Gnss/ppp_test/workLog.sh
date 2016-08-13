#!/bin/bash

# download erp file for PPP of METG on 2016 1 4

year=2016
month=1
ty="erp"

for (( day=3; day<=5; day++ ))
do
	echo "day=$day"
	getMGEXPros.sh -a "gbm" -t1 "$year $month $day 0 0 0" -c "$ty" -d "./ObsData/"

done

