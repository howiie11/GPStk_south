#include <iostream>

#include "Rinex3ObsStream.hpp"
#include "Rinex3ObsHeader.hpp"
#include "Rinex3ObsData.hpp"
#include "CivilTime.hpp"
#include "DataStructures.hpp"

using namespace std;
using namespace gpstk;

int main(void)
{
	cout << "Hello, rinex3!" << endl;
		// Create the input file stream
	Rinex3ObsStream rin("ObsData/brst0050.16o");
		
		// TEST 1 ------------------------------------------------------
//		// Create the output file stream
//	Rinex3ObsStream rout("ResultData/rinexOut.txt", ios::out|ios::trunc);
//		
//		// Read the rinex header
//	Rinex3ObsHeader head;
//	rin >> head; 
//	
//	rout.header = rin.header;
//	rout << rout.header;
//	
//		// Loop over all data epochs
//	Rinex3ObsData data;
//	while(rin >> data)
//	{
//		rout << data; 
//	}

		// TEST2 --------------------------------------------------------
		// It turns out that the gpstk can read rinex3 file of GPS system

//		// Read rinex header
//	Rinex3ObsHeader roh;
//	Rinex3ObsData roe;
////	rin >> roh;
//	gnssRinex gRin;
////	roh.dump(cout);
//
//		// Observation type interested
////	int indexC1(roh.getObsIndex("P1") );
//	
//	while(rin >> gRin)
//	{
//		CivilTime civtime(gRin.header.epoch);
//		cout << civtime << endl;
//		SatID prn(26, SatID::systemGPS);
///*		RinexDatum dataObj;
//		dataObj = roe.getObs(prn, indexC1);
//		double C1 = dataObj.data;
//
//		cout << fixed << setprecision(3);
//		cout << "P1 of G21 is find as:  " << C1 << endl; 
//*/
//		cout << fixed << setprecision(3);
//		cout << "C1: " << gRin(prn)(TypeID::C1) << endl; 
//		cout << "P2: " << gRin(prn)(TypeID::P2) << endl; 
//		cout << "C5: " << gRin(prn)(TypeID::C5) << endl; 
//		break;
//	}

	// TEST3 


	return 0;
}
