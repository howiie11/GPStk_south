#pragma ident "$Id$"

/**
 * @file CycleSlipEstimator.hpp 
 * This is a class to estimate cycle slips using time differenced code and phase
 * observables, as well as pseudo ionosphere observable 
 *
 */ 

#ifndef GPSTK_CYCLESLIPESTIMATOR_HPP
#define GPSTK_CYCLESLIPESTIMATOR_HPP

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
// Date     :	2017/06/01 - 2017/06/03
// Version	:	0.0
//	Author(s):	Lei Zhao, a Ph.D. candiate
// School of Geodesy and Geomatics, Wuhan University
//============================================================================

#include "ProcessingClass.hpp"

namespace gpstk
{

	class CycleSlipEstimator : public ProcessingClass
	{
	public:

		/* Common constructor 
		 *
		 * @param staticRec
		 *
		 */
		CycleSlipEstimator( )
			: firstTime( true ), 
			  codeType1( TypeID::prefitP1 ), codeType2( TypeID::prefitP2 )
		{ }; 



         /** Returns a satTypeValueMap object, adding the new data generated
          *  when calling this object.
          *
          * @param epoch     Time of observations.
          * @param gData     Data object holding the data.
          * @param epochflag Epoch flag.
          */
      virtual satTypeValueMap& Process( const CommonTime& epoch,
                                        satTypeValueMap& gData,
                                        const short& epochflag = 0 )
         throw(ProcessingException);


         /** Returns a gnnsRinex object, adding the new data generated when
          *  calling this object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssRinex& Process(gnssRinex& gData)
         throw(ProcessingException);

         /** Returns a gnnsSatTypeValue object, adding the new data generated
          *  when calling this object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData)
         throw(ProcessingException)
      { (*this).Process(gData.header.epoch, gData.body); return gData; };


			/** Set systems and observation types  
			 *
			 * @param sys			sat systems 
			 * @param ots			obs types of sys
			 */
			/// 
		virtual CycleSlipEstimator& setSysObsTypes( 
										const SatID::SatelliteSystem& sys,
										TypeIDSet& ots    )
		{	sysObsTypes[ sys ] = ots; return (*this); }


			/// Get frequency ratio constant miu for specifed system 
			/// and frequency str

         /// Returns a string identifying this object.
      virtual std::string getClassName(void) const;


         /// Destructor
      virtual ~CycleSlipEstimator() {};

			

	private:
			
			/// Observation types used in this class 
		TypeIDList obsTypeList;
		TypeID codeType1;
		TypeID codeType2;

			/// Pair of sys and obsType 
		std::map<SatID::SatelliteSystem, TypeIDSet> sysObsTypes; 

			/// Receiver state
		bool staticReceiver; 

			/// First epoch Flag
		bool firstTime;

			/// Initialization method 
		void Init(void);

			/// A structure used to store filter data for a SV
		struct filterData 
		{
			int windowSize; 
			double formerP1;
			double formerP2; 
		};	// End of ' struct filterData '


	};   // End of class CycleSlipEstimator

}   // End of namespace gpstk   





#endif	// GPSTK_CYCLESLIPESTIMATOR_HPP 
