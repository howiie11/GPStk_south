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

#include <deque>

#include "ProcessingClass.hpp" 
#include "GNSSObsSTDTables.hpp"
#include "GNSSconstants.hpp"
#include "SolverWMS.hpp"
#include "ARMLambda.hpp"
#include "SuccessRate.hpp"
#include "Stats.hpp"



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
			
			typedef std::map< SatID::SatelliteSystem, std::set<int> > SysSet;
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
										 : staticReceiver(true), deltaTMax(61.0),
											SR(0.0), useTimeDifferencedLI(false),
											dLI(0), dLIStd(0.1), maxBufferSize(12),
											ionoWeighted(false)
			{
					// Insert this sys<---> obsTypes pair 	
				sysObsTypes[usrSys] = usrObsTypes;

					// Get the band list of this sys, 
					// through which, num of frequency can also be derived 
				getBandList( usrSys );
				//sysNumFreq[SatID::systemGPS][TypeID::numFreq] = 
				//									CountFreqNum( SatID::systemGPS, obsTypes );
			};


			/* Add system <---> obsTypes pair 
			 * 
			 *	@param	sys
			 * @param	obsTypes
			 *
			 */



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
		virtual CycleSlipEstimator& addSystemObsTypes( 
										const SatID::SatelliteSystem& usrSys,
										TypeIDSet& usrObsTypes )
		{  
			sysObsTypes[usrSys] = usrObsTypes; 
			getBandList( usrSys );
			return (*this); 
		}


			/// Get LI time-diff data for a given sat and given LI type
			/// Note: the type parameter should be the time-differenced form   
		virtual double& getSatLITimeDiffData( const SatID& sat,
														  const TypeID& type )
		{ return satLITimeDiffData(sat)(type); }

			/// Get LI time-diff map 
		virtual satTypeValueMap getSatLITTimeDiffMap()
		{ return satLITimeDiffData; }


			/// Set using info of time-differenced LI 
		virtual CycleSlipEstimator& useTimeDiffLI( bool use )
		{ useTimeDifferencedLI = use; return (*this); };

			/** Set LI  types  
			 *
			 * @param sys			sat systems 
			 * @param ots			obs types of sys
			 */
			/// 
		virtual CycleSlipEstimator& setLITypes( const SatID::SatelliteSystem& sys, 
															 const TypeIDSet& usrLITypes )
		{ sysLITypes.clear(); sysLITypes[sys] = usrLITypes; return (*this); }

			/// Add a pair of sys <---> TypeIDSet
		virtual CycleSlipEstimator& addLITypes( const SatID::SatelliteSystem& sys, 
															 const TypeIDSet& usrLITypes )
		{ sysLITypes[sys] = usrLITypes; return (*this); }


			/// Set receiver state
		virtual CycleSlipEstimator& setReceiverStatic( bool staticRec )
		{ staticReceiver = staticRec; return (*this); }

			/// Set deltaLI value and std
		virtual CycleSlipEstimator& setDeltaLIState( const double& deltaLI, 
																	const double& deltaLIStd )
		{ dLI = deltaLI; dLIStd = deltaLIStd; return (*this); }


			/// Returns a string identifying this object.
		virtual std::string getClassName(void) const;

			/// Return a frequency index given a band in obsTypes
		virtual void getBandList( SatID::SatelliteSystem sys );

			/// Return a frequency index given a band in obsTypes
		virtual int getBandIndex( SatID::SatelliteSystem sys,
										  ObsID::ObservationType ot,
				                    int band );

			/// Return valid sat set
		virtual SatIDSet getValidSatSet()
		{ return validSatSet; }

			/// Get obsTypes of a specified sat system
		virtual TypeIDSet& getObsTypes( SatID::SatelliteSystem sys )
		{ return sysObsTypes[sys]; }

			/// Get Success rate
		virtual double getSuccessRate()
		{ return SR; }

			/// Set max window size of satTimeLIDeque 
		virtual CycleSlipEstimator& setMaxBufferSize(const int& maxBufSize);

			/* Return average deltaLI and its variance from 'satTimeLIDeque' given
			 * a sat
			 *
			 * @param sat		I 
			 * @param adLI  O  average of deltaLI 
			 * @param var		O  variance of average of deltaLI 
			 *
			 */
		virtual void getSmoothDeltaLI( const SatID& sat, const TypeID& ty, 
												 double& adLI, double& var );
		

			/* Return post residual regarding to a specific sat and a specific 
			 * type. These types should be corresponding to the data member
			 * obsTypes.
			 *
			 * @param sat
			 * @param type
			 * 
			 */
		virtual double getPostfitResidual( SatID sat, TypeID type );
			
			/// Return 'satPostfitResiduals'
//		virtual satTypeValueMap satPostfitResiduals; 

			/// Destructor
		virtual ~CycleSlipEstimator() {};


		private:

			bool ionoWeighted;

				/// Sat sys 
//			SatID::SatelliteSystem sys;

				/// Obs types 
//			TypeIDSet obsTypes; 

				/// Sat System <---> obsTypes
			SysTypeIDSetMap sysObsTypes;

				/// sat system <---> ( obsType <---> bandSet )
			struct SysObsTypeSet : std::map< SatID::SatelliteSystem, std::map<ObsID::ObservationType, std::set<int> > >
			{ 
				
					/// Returns a reference to the bandSet of sysObsTypeBandSet with 
					/// given sys and ot
				std::set<int>& operator()( const SatID::SatelliteSystem sys, 
													const ObsID::ObservationType ot )
					throw( ValueNotFound ); 
					/// Destructor 
				virtual ~SysObsTypeSet() {};
			};

			SysObsTypeSet sysObsTypeBandSet;

				/// Band list determined by obsTypes
			
			SysSet sysBandSet;
//			std::set<int> bandSet;

				/// Valid sat set used to estimate cycle slip
			SatIDSet validSatSet;

				/// Num of frequency used, which should be sat depentdent 
			SysTypeValueMap sysNumFreq;

				/// LI types 
			//TypeIDSet liTypes;
			SysTypeIDSetMap sysLITypes;

				/// sat index recorder in the defined time-differenced model 
			std::map< int, SatID > rowSat;
			std::map< int, std::map< SatID, TypeID > > ambColSat;
			std::map< int, SatID > ionColSat;

				/// Handy clear function 
			void clearSatIndexRecorder()
			{ rowSat.clear(); ambColSat.clear(); ionColSat.clear(); };

				/// Receiver state
			bool staticReceiver;

				/// Max limit of time gap 
			double deltaTMax;

				/// SuccessRate 
			double SR;

				/// deltaLI and its std
			double dLI, dLIStd;

				/// Use external ionosperic delay info 
			bool useTimeDifferencedLI;

				// Define a window to compute LI average
//				// This is a bad structure for inconvenient 'delete' operation
//			struct SatTimeLIDeque  std::map< SatID, std::map< CommonTime, double > > 
//		{
//			SatTimeLIDeque(): windowSize(10) {};
//
//				/// Set window size 
//			SatTimeLIDeque& setWindowSize( size_t size )
//			{ windowSize = size; return *this; }
//
//				/// Compute weight of every LI regarding to a 
//
//				/// Return a defined map with given sat
//			std::map< CommonTime, double >& operator()( const SatID& sat )
//					throw( ValueNotFound ); 
//			
//
//				/// Return the deltaLI value given a sat and time
//			double& operator()( const SatID& sat, 
//									  const CommonTime& time )
//					throw( ValueNotFound ); 
//			
//
//				/// Destructor 
//			virtual ~SatTimeLIDeque() {};
//
//				/// window size
//			size_t windowSize;
//		};
		
//		SatTimeLIDeque satTimeLIDeque;

		int maxBufferSize;

		static const int minBufferSize;

			// A better version of satTimeLIDeque
		struct TypeValueDeque : std::map< TypeID, std::deque<double> > 
		{
			std::deque<double>& operator()( const TypeID& ty )
					throw( TypeIDNotFound ); 

				// Destructor
			virtual ~TypeValueDeque() {};

			std::deque<double> eleWeightBuffer;
			std::deque<CommonTime> epochBuffer;

		};

		struct SatTypeValueDeque : std::map< SatID, TypeValueDeque >
		{
			
			TypeValueDeque& operator()( const SatID& sat ) 
					throw( SatIDNotFound ); 

				// Destructor
			 virtual ~SatTypeValueDeque() {};	
		};

		SatTypeValueDeque satLIDataDeque;
			

//				/// postfit residuals 
//			satTypeValueMap satPostfitRes;
			Vector<double> postfitResiduals;
			satTypeValueMap satPostfitResiduals;

				/// Struct to store sat data of former epoch
			satEpochTypeValueMap satFormerData;
			
				/// Sat time-differenced code data
//			satTypeValueMap satTimeDiffData;

				/// Sat time-differenced ionospheric delay data 
			satTypeValueMap satLITimeDiffData;

				/// Std tables 
			static GNSSObsSTDTables obsStd;


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
		virtual SatIDSet modelTimeDifferencedData( satTypeValueMap& satTimeDiffData,
																  satTypeValueMap& stvm,
																  Vector<double>& csVec, 
																  Matrix<double>& csCov );

			/** Cycle slip resolution
			 *
			 * @param csVec	I float estimates of cycle slip
			 * @param csCov   I
			 * @param gData   I/O
			 *
			 */
		virtual void cycleSlipResolution( Vector<double>& csVec, 
							  					    Matrix<double>& csCov, 
											       satTypeValueMap& gData );



	};   // End of class declaration


}   // End of namespace gpstk 

#endif	// GPSTK_CYCLESLIPESTIMATOR_HPP 
