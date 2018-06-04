#pragma ident "$Id$"

/**
 * @file PositioningResultsEvaluator.cpp
 * This is a class to get converged time and RMS of real-time positioning results
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2011
//
//============================================================================
// Date     :	2018/06/04
// Version	:	0.0
//	Author(s):	Lei Zhao, a Ph.D. candiate
// School of Geodesy and Geomatics, Wuhan University
//============================================================================

# include "PositioningResultsEvaluator.hpp"

namespace gpstk
{

		// Return a string identifying this object
	std::string PositioningResultsEvaluator::getClassName() const
	{ return "PositioningResultsEvaluator"; }

		// Add Positioning Solution
	void PositioningResultsEvaluator::addPositioningSolution( 
			const CommonTime& time, const Triple& position )
	{

		size_t timeWindowSize( timeWindow.size() );
		if( timeWindowSize == timeLength ) 
		{	
			ConvergedTimePrepared = true;
			return;
		}

		double error( position.mag() );

		if( error <= convergedPosError )
		{
				// Store this epoch until the size of window reaches 'timeLength'  	
			timeWindow.push_back( time );
			posWindow.push_back( position );
		}
		else{
			Reset();
		} // End of 'if( error < convergedPosError )'
	
	} // End of 'void PositioningResultsEvaluator:: ... '

} // End of 'namespace gpstk'

