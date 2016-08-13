#!/bin/bash

# this script is used to calculate max min average std of a column data file

if [ $# -eq 1 ];then
	awk '
			NR==1{max=min=$1}
		   {v[NR]=$1;sum+=$1}
			{if($1>max)max=$1;if(min>$1)min=$1}
		   END{avg=sum/NR;for(n=1;n<=NR;n++){sd+=(v[n]-avg)**2;rms+=(v[n])**2;}
		   sd=sqrt(sd/(NR-1));rms=sqrt(rms/(NR-1));
		   printf("Max %f\nMin %f\nSum %f\nAvg %f\nStd %f\nRMS %f\nItem %d\n",
				  max,min,sum,avg,sd,rms,NR)}' $1
else
	echo "Usage: $0 <data column file>"
fi
