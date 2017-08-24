#pragma ident "$Id$"

/**
 *	@file GNSSObservalesPrecisionTables.hpp 
 * This class defines a table for GNSS 
 */

#ifndef GPSTK_GNSSOBSERVABLESPRECISIONTABLES_HPP
#define GPSTK_GNSSOBSERVABLESPRECISIONTABLES_HPP

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

namespace gpstk
{

		/**
		 * 
		 * Explaination here ...
		 * 
		 */

	class GNSSObservalesPrecisionTables
	{
		public:

			/// Default constructor 
		GNSSObservalesPrecisionTables();

			/// Return precision info for refered sys and type 
		virtual double getGNSSObservablesPrecision( const SatID& sat, 
																  TypeID& type );

		private:

			/// BIG Map holding the precision info for muti-gnss system
		satTypeValueMap systemObsPrecisionMap;

			/// Dummy sat 
		gpstk::SatID dummySat;


	

	};   // End of 'class GNSSObservalesPrecisionTables'



}   // End of ' namespace gpstk '



#endif   // End of GPSTK_GNSSOBSERVABLESPRECISIONTABLES_HPP 
