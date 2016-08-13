//==============================================================================
// This file of courcse is part of GPSTk, the GPS Toolkit.
// The popurse of this file is to try to process RINEX3 data.
// Author Lei Zhao, WHU, 2016/06/30 ~ 2016/07/21 
//==============================================================================

#include<iostream>

#include "Rinex3NavStream.hpp"
#include "Rinex3NavHeader.hpp"
#include "Rinex3NavData.hpp"
#include "Rinex3EphemerisStore.hpp"
#include "Rinex3ObsHeader.hpp"
#include "Rinex3ObsStream.hpp"
#include "DataStructures.hpp"
#include "SolverLMS.hpp"
#include "SimpleFilter.hpp"
#include "BasicModel.hpp"
#include "ModelObs.hpp"
#include "XYZ2NEU.hpp"
#include "YDSTime.hpp"
#include "CivilTime.hpp"
#include "IonoModel.hpp"
#include "IonoModelStore.hpp"
#include "TropModel.hpp"
#include "SatID.hpp"

using namespace std;
using namespace gpstk;

int main(void)
{
		///////////// Initialization phase ///////////////////////

		///////////// COMMON OBJECTS  ///////////////////////
	  
		cout << fixed << setprecision(3);
			
			// Object to store Rinex3 navigation data
		Rinex3NavData r3NavData;

			// Object to read the header of Rinex3 navigation files
		Rinex3NavHeader r3NavHeader;

			// Create the input observation file stream
		Rinex3ObsStream rin3("ObsData/brst0050.16o");

			// Create the input navigation file stream
		Rinex3NavStream r3NavIn("ObsData/brst0050.16n");

			// Nominal Position
		Position nominalPos(4231162.4437, -332746.4829, 4745131.0258);

			// Rinex3 ephemeris store
		Rinex3EphemerisStore r3EphStore;
		r3EphStore.loadFile("ObsData/brst0050.16n");

			// Object to store ionospheric model
		IonoModelStore ionoStore;
			// Declare a Ionospheric Model object
		IonoModel ioModel;
			
			// Let's feed the ionospheric model (Klobuchar type) from data
			// in the Rinex3 navigation header
		ioModel.setModel(r3EphStore.Rhead.mapIonoCorr["GPSA"].param, r3EphStore.Rhead.mapIonoCorr["GPSB"].param);
			
			// Beware: In this case, the same model will be used for the full 
			// data span
		ionoStore.addIonoModel(CommonTime::BEGINNING_OF_TIME, ioModel);

			// Declare a MOPSTropModel object, setting the defaults
		MOPSTropModel mopsTM( nominalPos.getAltitude(), 
									 nominalPos.getGeodeticLatitude(),
									 5 );
			
			// Model observable
		ModelObs model(nominalPos, ionoStore, mopsTM, r3EphStore, TypeID::C1);

			// Basic model object to model observable
		//BasicModel basic(nominalPos, r3EphStore);
		

			// Declare a smple filter object. By default, it filters C1 with 
			// default limits
		SimpleFilter myFilter;
		

			// This is the GNSS data structure that will hold all the 
			// GNSS-related information
		gnssRinex gOriginal;

		///////////// COMMON OBJECTS END  ///////////////////////
		
		///////////// CASE #1 OBJECTS  ///////////////////////

			// Declare a base changing object from ECEF to NEU
		XYZ2NEU baseChange(nominalPos);

			
			
			// Declare a SolverLMS object
		SolverLMS  solver;

			// Configure this solver to use NEU
				// First, an obj of TypeIDSet
		TypeIDSet typeSet;
		typeSet.insert(TypeID::dLat);
		typeSet.insert(TypeID::dLon);
		typeSet.insert(TypeID::dH);
		typeSet.insert(TypeID::cdt);

				// Second, this is the proper equation structure to use with a 
				// NEU system
		gnssEquationDefinition newEq(TypeID::prefitC, typeSet);

				// Third, set the solver
		SolverLMS solverNEU;
		solverNEU.setDefaultEqDefinition(newEq);

		///////////// CASE #1 OBJECTS END  ///////////////////////
		
		////////////////////////////// CASE #1 ////////////////////////////////
			// This case is a common C1 + Least Mean Square Solver
			// LMS processing

		while( rin3 >> gOriginal )
		{
				// Let's output the time stamp
			CommonTime ct( gOriginal.header.epoch );
				
				// Print out the time in format of year, doy and sod
			cout << static_cast<YDSTime>(ct).year << " ";
			cout << static_cast<YDSTime>(ct).doy << " ";
			cout << static_cast<YDSTime>(ct).sod << " ";

				// Let's make a working copy
			gnssRinex gRin1(gOriginal);
			gRin1.keepOnlySatSystem(SatID::systemGPS);
	
			try
			{
					// This is the line that we process all the GNSS data
				//gRin1 >> myFilter >> basic >> baseChange >> solver;
				gRin1 >> myFilter >> model >> baseChange >> solverNEU;
				//gRin1 >> myFilter >> model >> solver;
				
				//gRin1 >> myFilter >> model >> solver;

	
					//	Get your result out of the solver object. In ECEF system by default
				cout << solverNEU.getSolution(TypeID::dLat) << " ";
				cout << solverNEU.getSolution(TypeID::dLon) << " ";
				cout << solverNEU.getSolution(TypeID::dH) << endl;
			}
			catch(...)
			{
				cerr << "Case 1. Exception at epoch: " << gRin1.header.epoch << endl;
			}  // End of try-catch block of case 1

			
		}  // End of ' while( rin3 >> gOriginal ) '

	return 0;
}
