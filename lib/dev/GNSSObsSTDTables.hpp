#pragma ident "$Id$"

/**
 *	@file GNSSObsSTDTables.hpp 
 * This class defines a zenith-referenced std table for GNSS observables 
 * to use it, you have to take elevation into consideration 
 */

#ifndef GPSTK_GNSSOBSSTDTABLES_HPP
#define GPSTK_GNSSOBSSTDTABLES_HPP

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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2011
//
//============================================================================
// Date     :	2017/08/24 - 2017/08/24
// Version	:	0.0
//	Author(s):	Lei Zhao, a Ph.D. candiate
// School of Geodesy and Geomatics, Wuhan University
//============================================================================



#include "SatID.hpp"
#include "TypeID.hpp"
#include "DataStructures.hpp"

namespace gpstk
{

		/**
		 * 
		 * Explaination here ...
		 * 
		 */

	class GNSSObsSTDTables
	{
		public:

			/// Default constructor 
		GNSSObsSTDTables(){};

			/// Return precision info for refered sys and type 
		virtual double getGNSSObsSTD( SatID::SatelliteSystem sys, 
												TypeID type );


			/// Returns a string identifying this object.
		virtual std::string getClassName(void) const; 

			/// Destructor
		virtual ~GNSSObsSTDTables() {}; 

			/// BIG Map holding the precision info for muti-gnss system
		static satTypeValueMap systemObsPrecisionMap;

	public:
			/// Class to initialize the 'systemObsPrecisionMap'
		class Initializer
		{
		public:
			Initializer();
		};

		static Initializer tableSingleton;

			/// Dummy sat 
		//gpstk::SatID dummySat;

	

	};   // End of 'class GNSSObsSTDTables'



}   // End of ' namespace gpstk '



#endif   // End of GPSTK_GNSSOBSSTDTABLES_HPP 
