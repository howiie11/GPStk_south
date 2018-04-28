#pragma ident "$Id$"

/**
 * @file: GNSSObsSTDTables.cpp 
 * More info about this class can be found in its header file 
 *
 */

//============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 2.1 of the License, or
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2009
//
//============================================================================


#include "GNSSObsSTDTables.hpp"

namespace gpstk
{

		/// Returns a string identifying this object.
	std::string GNSSObsSTDTables::getClassName(void) const
	{ return "GNSSObsSTDTables"; }

		// Initialize the 'systemObsPrecisionMap'
	satTypeValueMap GNSSObsSTDTables::systemObsPrecisionMap;

	GNSSObsSTDTables::Initializer tableSingleton;

		/// Default constructor
	GNSSObsSTDTables::Initializer::Initializer()
	{
		SatID gpsSat(-1, SatID::systemGPS );
		SatID galileoSat(-1, SatID::systemGalileo );

			// GPS 
		systemObsPrecisionMap[ gpsSat ][ TypeID::prefitC1 ] = 0.25; // unit meter 
		systemObsPrecisionMap[ gpsSat ][ TypeID::prefitP1 ] = 0.50; // unit meter 
		systemObsPrecisionMap[ gpsSat ][ TypeID::prefitP2 ] = 0.50;  
		systemObsPrecisionMap[ gpsSat ][ TypeID::prefitC5 ] = 0.15;  
		systemObsPrecisionMap[ gpsSat ][ TypeID::prefitL1 ] = 0.001;  
		systemObsPrecisionMap[ gpsSat ][ TypeID::prefitL2 ] = 0.0013;  
		systemObsPrecisionMap[ gpsSat ][ TypeID::prefitL5 ] = 0.0013;  
		systemObsPrecisionMap[ gpsSat ][ TypeID::prefitPC ] = 1;  

			// Galileo 
		systemObsPrecisionMap[ galileoSat ][ TypeID::prefitC1 ] = 0.2; // unit meter 
		systemObsPrecisionMap[ galileoSat ][ TypeID::prefitC5 ] = 0.15;  
		systemObsPrecisionMap[ galileoSat ][ TypeID::prefitC7 ] = 0.15;  
		systemObsPrecisionMap[ galileoSat ][ TypeID::prefitL1 ] = 0.001;  
		systemObsPrecisionMap[ galileoSat ][ TypeID::prefitL5 ] = 0.0013;  
		systemObsPrecisionMap[ galileoSat ][ TypeID::prefitL7 ] = 0.0013;  
		
	}


		// Return precision info for refered sys and type
	double GNSSObsSTDTables::getGNSSObsSTD( SatID::SatelliteSystem sys, 
														 TypeID type )
	{
	
			// Dummy sat for index 
		SatID dummySat( -1, sys );

		double value(0.0);
		try
		{
			value = systemObsPrecisionMap.getValue( dummySat, type );
		}
		catch( Exception& u )
		{
			Exception e( getClassName() + ":" + u.what() );
			GPSTK_THROW(e);
		}
		return value; 

	}   // End of ' double GNSSObsSTDTables::getGNSSObsSTD() '


}   // End of namespace gpstk








