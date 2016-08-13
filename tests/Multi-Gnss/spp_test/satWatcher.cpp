//==============================================================================
// File: satsWatcher.cpp 
// This program is used to watch the average observed sats for a given rinex3
// observation file
// Written by Lei Zhao, WHU, 2016/07/25
//==============================================================================

#include <iostream>
#include <string>
#include <set>

#include "Rinex3ObsStream.hpp"
#include "Rinex3ObsHeader.hpp"
#include "DataStructures.hpp"
#include "SatID.hpp"
#include "Stats.hpp"
#include "CivilTime.hpp"

using namespace std;
using namespace gpstk;

int main( int argc, char** argv)
{
		// Need an input filename
	if( argc == 2)
	{
		//cout << argv[1] << endl;
		
		string fileName(argv[1]);
			
			// RINEX3 obs stream
		Rinex3ObsStream r3In(fileName);
		//Rinex3ObsStream r3In("ObsData/obs/abmf0050.16o");
		r3In.exceptions(ifstream::failbit);
		
			// gnssRinex obj
		gnssRinex gRin;

			// Set to hold num of sats
		set<double> SatNumSet;

			// Stat obj
		Stats<double> statistic;

		SatID dummySat;
		SatID::SatelliteSystem sys = SatID::systemGalileo;
		string strSys = dummySat.convertSatelliteSystemToString(sys);

//			// Valid header 
//		Rinex3ObsHeader r3Header;
//		r3In >> r3Header; 
//		if( ! r3Header.isValid() )
//		{
//			cerr << "invalid header of rinex file: " << fileName << endl;
//			return 1;
//		}

		try 
		{
				// Feed gRin
			while( r3In >> gRin )
			{
					// Keep only the specified system
				gRin.keepOnlySatSystem(sys);
	
			//	gRin.body.dump(cout);
				
				double satNum( gRin.body.numSats() );
				
				statistic.Add(satNum);

				CivilTime ct(gRin.header.epoch);
				cout << ct << " satNum: " << satNum << endl;

			} // End of 'while( r3In >> gRin )'
		}
		catch(FFStreamError& e)
		{
			cerr << e.what() << endl;
			return 1;
		}

//		cout << strSys << " average_sats_num: " << statistic.Average() << endl; 

			
	}
	else{
		cout << "Usage: satsWatcher <rinex3 Obs file>" << endl;
	}
	return 0;
}


