#pragma ident "$Id$"

/**
 * @file PositioningResultsEvaluator.hpp
 * This is a class to get converged time and RMS of real-time positioning results
 * 
 */ 

#ifndef GPSTK_RESULTSEVALUATION_HPP
#define GPSTK_RESULTSEVALUATION_HPP

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
// Date     :	2018/06/03
// Version	:	0.0
//	Author(s):	Lei Zhao, a Ph.D. candiate
// School of Geodesy and Geomatics, Wuhan University
//============================================================================

#include <map>
#include <deque>

#include "CommonTime.hpp"
#include "Stats.hpp"
#include "Triple.hpp"
#include "StringUtils.hpp"

#include "PositioningResultsStream.hpp"
#include "PositioningResultsHeader.hpp"
#include "PositioningResultsData.hpp"

namespace gpstk
{

		/*** Usage
		 *
		 *
		 */
	class PositioningResultsEvaluator 
	{
		public:

				/** Common constructor
				 * 
				 * @param time
				 * @param position
				 */
			PositioningResultsEvaluator() :
								ConvergedTimePrepared(false), firstTime(true),
								firstEpoch(CommonTime::BEGINNING_OF_TIME),
								lastEpoch(CommonTime::BEGINNING_OF_TIME),
								convergedPosError(0.10), acceptablePosError(0.50), 
								dTMax(61.0), timeLength(20)
								{}; 

				/// Reset method
			void Reset()
			{ 
				timeWindow.clear();
				posWindow.clear();

//				firstTime = true;

//				statistician.Reset();
			}

				/// Add Solution
			virtual void addPositioningSolution( const CommonTime& time, 
															 const Triple& positon ); 


			virtual void addPositioningSolution( const CommonTime& time, 
															 const double& dx, 
															 const double& dy, 
															 const double& dz )
			{ 
				Triple pos( dx, dy, dz);
				addPositioningSolution( time, pos );
			}

				/// Load Positioning Results file
			virtual void loadFile( const std::string& filename ) throw(Exception);


				/// Return converged time (second of day)
			virtual double getConvergedTime();

				/// Return RMS of N/X error component
			virtual double getNorthErrorRMS()
			{
				return statisticianN.RMS();
			}

				/// Return average of N/X error component
			virtual double getNorthErrorAverage()
			{
				return statisticianN.Average();
			}

				/// Return RMS of E/Y error component
			virtual double getEastErrorRMS()
			{
				return statisticianE.RMS();
			}

				/// Return average of E/Y error component
			virtual double getEastErrorAverage()
			{
				return statisticianE.Average();
			}

				/// Return RMS of U/Z error component
			virtual double getUpErrorRMS()
			{
				return statisticianU.RMS();
			}

				/// Return average of U/Z error component
			virtual double getUpErrorAverage()
			{
				return statisticianU.Average();
			}

				/// Return a string identifying this object
			std::string getClassName() const;

		private:

				/// First time indicator
			bool firstTime;

				/// First time recorder
			CommonTime firstEpoch;

				/// Last time recorder
			CommonTime lastEpoch;

				/// Converged positioning error
			double convergedPosError;

				/// Acceptable positioning error
			double acceptablePosError;

				/// dTMax
			double dTMax; 

				/// Evaluted length of time
			size_t timeLength;

				/// Time window 
			std::deque< CommonTime > timeWindow;

				/// Data window
			std::deque< Triple > posWindow;  

				/// ConvergedTimePrepared
			bool ConvergedTimePrepared;

				/// Statistician 
			Stats<double> statistician;
			Stats<double> statisticianN;
			Stats<double> statisticianE;
			Stats<double> statisticianU;

				
		
				
	};





} // End of 'namespace gpstk'







#endif
