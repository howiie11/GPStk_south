#pragma ident "$Id$"

/**
 * @file CodeBlunderDetection.hpp
 * This is a class to detect code blunders using quality control theory
 * 
 * For more info, please refer to: 
 * Banville and B. Langley (2013) Mitigating the impact of ionospheric cycle 
 * slips in GNSS observations 
 *
 */  

#ifndef GPSTK_CODEBLUNDERDETECTION_HPP 
#define GPSTK_CODEBLUNDERDETECTION_HPP

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
	
	class CodeBlunderDetection : public ProcessingClass
	{
		public:
			
			/* Default constructor 
			 * 
			 */
			CodeBlunderDetection()
				: deltaTMax(61.0), staticReceiver(true), 
				  useExternalIonoDelayInfo(true) 
			{};

			/* Common Constructor 
			 *
			 * @param usrSys 
			 * @param codeTypes 
			 *
			 */
			CodeBlunderDetection( const SatID::SatelliteSystem& usrSys,
										 const TypeIDSet& usrCodeTypes )
										 : sys(usrSys), codeTypes(usrCodeTypes),
											staticReceiver(true), deltaTMax(61.0),
											useExternalIonoDelayInfo(true) 
			{};


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
		virtual CodeBlunderDetection& setSysCodeTypes( 
										const SatID::SatelliteSystem& usrSys,
										TypeIDSet& usrCodeTypes )
		{ sys  = usrSys; codeTypes = usrCodeTypes; return (*this); }


			///  Get LI time-diff data
		virtual satTypeValueMap& getSatLITimeDiffData()
		{ return satLITimeDiffData; }


			/** Set LI  types  
			 *
			 * @param sys			sat systems 
			 * @param ots			obs types of sys
			 */
			/// 
		virtual CodeBlunderDetection& setLITypes( TypeIDSet& usrLITypes )
		{ liTypes = usrLITypes; return (*this); }


			/// Returns a string identifying this object.
		virtual std::string getClassName(void) const;

			/// Destructor
		virtual ~CodeBlunderDetection() {};


		private:

				/// Sat sys 
			SatID::SatelliteSystem sys;

				/// Code obs types
			TypeIDSet codeTypes; 

				/// LI types 
			TypeIDSet liTypes;

				/// Receiver state
			bool staticReceiver;

				/// Max limit of time gap 
			double deltaTMax;

				/// Use external ionosperic delay info 
			bool useExternalIonoDelayInfo;

				/// Struct to store sat data of former epoch
			satEpochTypeValueMap satFormerData;
			

				/// Sat time-differenced code data
			satTypeValueMap satCodeTimeDiffData;

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
		virtual bool getSatFilterData( const CommonTime& epoch, 
													  const SatID& sat,
													  typeValueMap& tvMap, 
													  const short& epochFlag ); 


	};   // End of class declaration



}   // End of namespace gpstk 

#endif	// GPSTK_CODEBLUNDERDETECTION_HPP 
