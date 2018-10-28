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

#include <string>
#include <vector>
#include "StringUtils.hpp"
#include "PositioningResultsStream.hpp"
#include "PositioningResultsHeader.hpp"
#include "PositioningResultsData.hpp"
#include "YDSTime.hpp"

using namespace gpstk::StringUtils;
using namespace std;

namespace gpstk
{
	void PositioningResultsData::reallyGetRecord(FFStream& ffs)
		throw(exception, FFStreamError, StringException)
	{
			// Cast the stream to be a PositioningResultsStream
		PositioningResultsStream& strm = dynamic_cast<PositioningResultsStream&>(ffs);
		string line;
		strm.formattedGetLine(line, true);
		cout << "Pos data line: " << line << endl;

		vector<string> dataStr( StringUtils::split(line,' ') );
			
			// Get time
		int year( StringUtils::asInt( dataStr[0] ) );
		int doy( StringUtils::asInt( dataStr[1] ) );
		double sod( StringUtils::asDouble( dataStr[2] ) );
	
		YDSTime yds( year, doy, sod );
		time = yds.convertToCommonTime();

			// Get position
		coordinate[0] = StringUtils::asDouble( dataStr[3] );
		coordinate[1] = StringUtils::asDouble( dataStr[4] );
		coordinate[2] = StringUtils::asDouble( dataStr[5] );

		cout << "yds: "<< yds << " coordinate: " << coordinate << endl; 

	} // End of 'void PositioningResultsData::reallyGetRecord(FFStream& ffs)'


}


