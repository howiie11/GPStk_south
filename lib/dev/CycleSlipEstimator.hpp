#pragma ident "$Id$"

/**
 * @file CycleSlipEstimator.hpp
 * This is a class to detect code blunders using quality control theory
 * 
 * For more info, please refer to: 
 * Banville and B. Langley (2013) Mitigating the impact of ionospheric cycle 
 * slips in GNSS observations 
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
// Date     :	2017/08/15 - 2017/08/16
// Version	:	0.0
//	Author(s):	Lei Zhao, a Ph.D. candiate
// School of Geodesy and Geomatics, Wuhan University
//============================================================================




#include "ProcessingClass.hpp" 

namespace gpstk
{



		/*** Usage
		 *
		 *
		 *
		 */
	
	class CycleSlipEstimator : public ProcessingClass
	{
		public:
			
			/* Default constructor 
			 * 
			 */
//			CycleSlipEstimator()
//				: deltaTMax(61.0), staticReceiver(true), 
//				  useTimeDifferencedLI(false) 
//			{ 
//				// Get band list first from the obsTypes defined by the user 
//				// But now only GPS 
//				getBandList();
//			};

			/* Common Constructor 
			 *
			 * @param usrSys 
			 * @param obsTypes 
			 *
			 */
			CycleSlipEstimator( const SatID::SatelliteSystem& usrSys,
										 const TypeIDSet& usrObsTypes )
										 : sys(usrSys), obsTypes(usrObsTypes),
											staticReceiver(true), deltaTMax(61.0),
											useTimeDifferencedLI(false) 
			{
				// Get band list first from the obsTypes defined by the user 
				// But now only GPS 
				getBandList();
				sysNumFreq[SatID::systemGPS][TypeID::numFreq] = 
													CountFreqNum( SatID::systemGPS, obsTypes );
			};


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
		{ (*this).Process(gData.header.epoch, gData.body); return gData;  };


			/** Set systems and observation types  
			 *
			 * @param sys			sat systems 
			 * @param ots			obs types of sys
			 */
			/// 
		virtual CycleSlipEstimator& addSysObsTypes( 
										const SatID::SatelliteSystem& usrSys,
										TypeIDSet& usrObsTypes )
		{ sys  = usrSys; obsTypes = usrObsTypes; return (*this); }


			///  Get LI time-diff data
		virtual satTypeValueMap& getSatLITimeDiffData()
		{ return satLITimeDiffData; }


			/// Set using info of time-differenced LI 
		virtual CycleSlipEstimator& setUsingTimeDiffLI( bool use )
		{ useTimeDifferencedLI = use; return (*this); };

			/** Set LI  types  
			 *
			 * @param sys			sat systems 
			 * @param ots			obs types of sys
			 */
			/// 
		virtual CycleSlipEstimator& setLITypes( TypeIDSet& usrLITypes )
		{ liTypes = usrLITypes; return (*this); }


			/// Set receiver state
		virtual CycleSlipEstimator& setReceiverStatic( bool staticRec )
		{ staticReceiver = staticRec; return (*this); }


			/// Returns a string identifying this object.
		virtual std::string getClassName(void) const;

			/// Return a frequency index given a band in obsTypes
		virtual void getBandList();

			/// Return a frequency index given a band in obsTypes
		virtual int getBandIndex( int band );

			/// Destructor
		virtual ~CycleSlipEstimator() {};


		private:

				/// Sat sys 
			SatID::SatelliteSystem sys;

				/// Obs types 
			TypeIDSet obsTypes; 

				/// Band list determined by obsTypes
			std::set<int> bandSet;

				/// Num of frequency used, which should be sat depentdent 
			SysTypeValueMap sysNumFreq;

				/// LI types 
			TypeIDSet liTypes;

				/// Receiver state
			bool staticReceiver;

				/// Max limit of time gap 
			double deltaTMax;

				/// Use external ionosperic delay info 
			bool useTimeDifferencedLI;

				/// postfit residuals 
			satTypeValueMap satPostfitRes;

				/// Struct to store sat data of former epoch
			satEpochTypeValueMap satFormerData;
			
				/// Sat time-differenced code data
//			satTypeValueMap satTimeDiffData;

				/// Sat time-differenced ionospheric delay data 
			satTypeValueMap satLITimeDiffData;
			

			/* Get filter data
			 *
			 * Return the flag indicating whether the phase is continious in time
			 * and there are LLI cycle-slip declaration(TO DO!!!)
			 *
			 * @param epoch
			 * @param sat
			 * @param tvMap		data related to this sat
			 * @param epochFlag 
			 */ 
		virtual bool  getSatFilterData( const CommonTime& epoch, 
												  const SatID& sat,
												  typeValueMap& tvMap, 
												  const short& epochFlag, 
												  satTypeValueMap& satTimeDiffData); 

			/** Model time-differenced code data
			 *
			 * @param stvm
			 * @param csVec
			 * @param csCov
			 *
			 */ 
		virtual SatIDSet  modelTimeDifferencedData( satTypeValueMap& satTimeDiffData,
																  satTypeValueMap& stvm,
																  Vector<double>& csVec, 
																  Matrix<double>& csCov );


	};   // End of class declaration


}   // End of namespace gpstk 

#endif	// GPSTK_CYCLESLIPESTIMATOR_HPP 
