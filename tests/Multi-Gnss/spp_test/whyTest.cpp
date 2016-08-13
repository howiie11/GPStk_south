//============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 3.0 of the License, or
//  any later version.
//
//  The GPSTk is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with GPSTk; if not, write to the Free Software Foundation,
//  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
//  
//  Copyright 2004, The University of Texas at Austin
//
//============================================================================

//============================================================================
//
//This software developed by Applied Research Laboratories at the University of
//Texas at Austin, under contract to an agency or agencies within the U.S. 
//Department of Defense. The U.S. Government retains all rights to use,
//duplicate, distribute, disclose, or release this software. 
//
//Pursuant to DoD Directive 523024 
//
// DISTRIBUTION STATEMENT A: This software has been approved for public 
//                           release, distribution is unlimited.
//
//=============================================================================

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
#include "RequireObservables.hpp"

using namespace std;
using namespace gpstk;

void printModel( ofstream& modelfile, const gnssRinex& gData, int precision=4);

int main(int argc, char** argv)
{
		///////////// Initialization phase ///////////////////////

		///////////// COMMON OBJECTS  ///////////////////////
	   if(argc != 3 )
		{
			cerr << "Usage: <" << argv[0] << ">" << "<o file> <n file>" << endl; 
			return -1;
		}
		cout << fixed << setprecision(3);
			
			// Object to store Rinex3 navigation data
		Rinex3NavData r3NavData;

			// Object to read the header of Rinex3 navigation files
		Rinex3NavHeader r3NavHeader;

			// Create the input observation file stream
//		Rinex3ObsStream rin3("ObsData/obs/metg0010.16o");
		Rinex3ObsStream rin3(argv[1]);
		rin3.exceptions(ifstream::failbit);



			// Rinex3 ephemeris store
		Rinex3EphemerisStore r3EphStore;
//		r3EphStore.loadFile("ObsData/nav/metg0010.16l");
		r3EphStore.loadFile(argv[2]);
//		r3EphStore.loadFile("ObsData/nav/g/yel20050.16g", true, cout);
//		r3EphStore.loadFile("ObsData/metg0050.16l");
//		r3EphStore.loadFile("ObsData/brst0050.16n");

//		r3EphStore.dump(cout, 1);

//			// Nominal Position
			// BRST
		//Position nominalPos(4231162.4437, -332746.4829, 4745131.0258);
//		Position nominalPos(-3464038.8185, 1334173.1372, -5169223.9594);
			// tlse
		//Position nominalPos(4627851.7059, 119640.2239, 4372993.6863); 
			// yel2
//		Position nominalPos(-1224442.0696, -2689174.6637, 5633660.3798);
			// METG(good)
		Position nominalPos(2890652.4283, 1310295.6036, 5513958.9116);
			// NKLG
//		Position nominalPos(6287385.7305, 1071574.7724, 39133.1189);
			// DJIG
//		Position nominalPos(4583086.0404, 4250982.5720, 1266243.1300);

			// BRUX
//		Position nominalPos(4027881.4382, 306998.6806, 4919498.9892);

			// DYNG
//		Position nominalPos(4595220.0388, 2039434.1411, 3912625.9516);
			
			// JFNG
//		Position nominalPos(-2279828.9540, 5004706.5001, 3219777.4317);

//		PZ90Ellipsoid pz90;
//		nominalPos.asGeodetic(&pz90);
//		nominalPos.asECEF();
//
//			// Object to store ionospheric model
//		IonoModelStore ionoStore;
//			// Declare a Ionospheric Model object
//		IonoModel ioModel;
//			
//			// Let's feed the ionospheric model (Klobuchar type) from data
//			// in the Rinex3 navigation header
//		ioModel.setModel(r3EphStore.Rhead.mapIonoCorr["GPSA"].param, r3EphStore.Rhead.mapIonoCorr["GPSB"].param);
//			
//			// Beware: In this case, the same model will be used for the full 
//			// data span
//		ionoStore.addIonoModel(CommonTime::BEGINNING_OF_TIME, ioModel);
//
//			// Declare a MOPSTropModel object, setting the defaults
		MOPSTropModel mopsTM( nominalPos.getAltitude(), 
									 nominalPos.getGeodeticLatitude(),
									 5 );
//			
//			// Model observable
		ModelObs model(nominalPos, mopsTM, r3EphStore, TypeID::C1);
//		ModelObs model( mopsTM, r3EphStore, TypeID::P2);
//
//			// Basic model object to model observable
//		BasicModel basic(nominalPos, r3EphStore);
//
//			// Declare a smple filter object. By default, it filters C1 with 
//			// default limits
//		SimpleFilter myFilter;
//		myFilter.setFilteredType(TypeID::P2);
//
//			// Declare an object to check that all required observables
//			// are present
		RequireObservables requireObs(TypeID::C1);
//		
//
//			// This is the GNSS data structure that will hold all the 
//			// GNSS-related information
		gnssRinex gOriginal;
//
//		///////////// COMMON OBJECTS END  ///////////////////////
//		
//		///////////// CASE #1 OBJECTS  ///////////////////////
//
//			// Declare a base changing object from ECEF to NEU
		XYZ2NEU baseChange(nominalPos);
//
//			
//			
//			// Declare a SolverLMS object
//		SolverLMS  solver;
//
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

//		///////////// CASE #1 OBJECTS END  ///////////////////////
//		
//			// This case is a common C1 + Least Mean Square Solver
//			// LMS processing

//		string modelName;
//		ofstream modelfile;

//		modelName = "ObsData/brst0050.16model";
//		modelfile.open(modelName.c_str(), ios::out );

		try
		{
			while( rin3 >> gOriginal )
			{
				
	//				// Let's output the time stamp
				CommonTime ct( gOriginal.header.epoch );
				CivilTime civilTime(gOriginal.header.epoch);
	//				
	//
	//				// Let's make a working copy
				gnssRinex gRin1(gOriginal);
				gRin1.keepOnlySatSystem(SatID::systemGalileo);
	//			gRin1.body.dump(cout);
				//gRin1.keepOnlySatSystem(SatID::systemGPS);
		
				try
				{
						// This is the line that we process all the GNSS data
					gRin1 >> requireObs;
	
					gRin1	>> model; 

					gRin1	>> baseChange;

					gRin1 >> solverNEU;
		
				}
				catch(SVNumException& e)
				{
//					cerr << civilTime << " Sats num < 4" << endl;
					continue; 
				}
				catch(Exception& e)
				{
					cerr  << civilTime << e <<  endl;
				}  // End of try-catch block of case 1
	
					// Print out the time in format of year, doy and sod
				cout << static_cast<YDSTime>(ct).year << " ";
				cout << static_cast<YDSTime>(ct).doy << " ";
				cout << static_cast<YDSTime>(ct).sod << " ";
	
					//	Get your result out of the solver object. In ECEF system by default
				cout << solverNEU.getSolution(TypeID::dLat) << " ";
				cout << solverNEU.getSolution(TypeID::dLon) << " ";
				cout << solverNEU.getSolution(TypeID::dH) << endl;
	
	//			printModel(modelfile, gRin1);
	
				
			}  // End of ' while( rin3 >> gOriginal ) '
		}
		catch (FFStreamError& e)
		{
			cerr << e.what() << endl;
			return 1;
		}

//		modelfile.close();

	return 0;
}

void printModel( ofstream& modelfile, const gnssRinex& gData, int precision)
{
		// Prepare for printing
	modelfile << fixed << setprecision( precision );

		// Get epoch out of GDS
	CommonTime time(gData.header.epoch);

		// Iterator through the GNSS DataStructure
	for( satTypeValueMap::const_iterator it = gData.body.begin();
		  it != gData.body.end();
		  ++it )
	{
			// print epoch
		modelfile << static_cast<YDSTime>(time).year << " ";
		modelfile << static_cast<YDSTime>(time).doy  << " ";
		modelfile << static_cast<YDSTime>(time).sod  << " ";

			// sat
		modelfile << (*it).first << " ";

			// // Print model values
		for( typeValueMap::const_iterator itObs = (*it).second.begin();
			  itObs != (*it).second.end();
			  ++itObs )
		{
			TypeID type = itObs->first;
			if( type == TypeID::postfitC )
			{
				modelfile << (*itObs).first << " ";
				modelfile << (*itObs).second << " ";
			}
		}  // End of ' for( typeValueMap::const_iterator  ... '

		modelfile << endl;

	}  // End of ' for( satTypeValueMap::const_iterator ... '

}  // End of ' void printModel( ofstream& modelfile, ... '

