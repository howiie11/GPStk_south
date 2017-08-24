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

		/// Default constructor
	GNSSObsSTDTables::GNSSObsSTDTables()
	{
		SatID gpsSat(-1, SatID::systemGPS );
		SatID galileoSat(-1, SatID::systemGalileo );

			// GPS 
		systemObsPrecisionMap[ gpsSat ][ TypeID::P1 ] = 0.25; // unit meter 
		systemObsPrecisionMap[ gpsSat ][ TypeID::P2 ] = 0.25;  
		systemObsPrecisionMap[ gpsSat ][ TypeID::C5 ] = 0.15;  
		systemObsPrecisionMap[ gpsSat ][ TypeID::L1 ] = 0.001;  
		systemObsPrecisionMap[ gpsSat ][ TypeID::L2 ] = 0.0013;  
		systemObsPrecisionMap[ gpsSat ][ TypeID::L5 ] = 0.0013;  

			// Galileo 
		systemObsPrecisionMap[ galileoSat ][ TypeID::C1 ] = 0.2; // unit meter 
		systemObsPrecisionMap[ galileoSat ][ TypeID::C5 ] = 0.15;  
		systemObsPrecisionMap[ galileoSat ][ TypeID::C7 ] = 0.15;  
		systemObsPrecisionMap[ galileoSat ][ TypeID::L1 ] = 0.001;  
		systemObsPrecisionMap[ galileoSat ][ TypeID::L5 ] = 0.0013;  
		systemObsPrecisionMap[ galileoSat ][ TypeID::L7 ] = 0.0013;  
		
	}   // End of constructor


		// Return precision info for refered sys and type
	double GNSSObsSTDTables::getGNSSObsSTD( SatID::SatelliteSystem sys, 
														 TypeID type )
	{
	
			// Dummy sat for index 
		SatID dummySat( -1, sys );

		return systemObsPrecisionMap.getValue( dummySat, type ); 

	}   // End of ' double GNSSObsSTDTables::getGNSSObsSTD() '


}   // End of namespace gpstk








