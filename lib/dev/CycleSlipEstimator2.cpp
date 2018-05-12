#pragma ident "$Id$"

/**
 * @file CycleSlipEstimator2.cpp
 * This is a class to detect code blunders using quality control theory
 * 
 * For more info, please refer to: 
 * Banville and B. Langley (2013) Mitigating the impact of ionospheric cycle 
 * slips in GNSS observations 
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
// Date     :	2017/08/15 - 2017/08/16
// Version	:	0.0
//	Author(s):	Lei Zhao, a Ph.D. candiate
// School of Geodesy and Geomatics, Wuhan University
//============================================================================


#include "CycleSlipEstimator2.hpp"

namespace gpstk
{
		// Return a string identifying this object
	std::string CycleSlipEstimator2::getClassName() const
	{ return "CycleSlipEstimator2"; }


		// Static member: 
	GNSSObsSTDTables CycleSlipEstimator2::obsStd;

//	double CycleSlipEstimator2::unitWeightStd = 
//											obsStd.getGNSSObsSTD( SatID::systemGPS, 
//																		 TypeID::prefitPC );


      // Minimum buffer size . It is always set to 5
   const int CycleSlipEstimator2::minBufferSize = 5;

		// student distribution table (alpha = 0.005 )
	const double CycleSlipEstimator2::t[30] = { 
			63.66, 9.92, 5.84, 4.60, 4.03, 3.71, 3.50, 3.36, 3.25, 3.17, 3.11, 3.05, 3.01, 2.98, 2.95, 2.92, 2.90, 2.88, 2.86, 2.85, 2.83, 2.82, 2.81, 2.80, 2.79, 2.78, 2.77, 2.76, 2.76, 2.75
	};

		// F distribution table ( alpha = 0.001 )
		// Col: degree of freedom of denominator
		// Rows: degree of freedom of numerator 
	const double CycleSlipEstimator2::F2[ 20*3 ] = {  
			405284.07, 499999.50, 540379.20,
			998.50, 999.00, 999.17,
			167.03, 148.50, 141.11,
			74.14, 61.25, 56.18,
			47.18, 37.12, 33.20,
			35.51, 27.00, 23.70,
			29.25, 21.69, 18.77,
			25.41, 18.49, 15.83,
			22.86, 16.39, 13.90,
			21.04, 14.91, 12.55,
			19.69, 13.81, 11.56,
			18.64, 12.97, 10.80,
			17.82, 12.31, 10.21,
			17.14, 11.78, 9.73,
			16.59, 11.34, 9.34,
			16.12, 10.97, 9.01,
			15.72, 10.66, 8.73,
			15.38, 10.39, 8.49,
			15.08, 10.16, 8.28,
			14.82, 9.95, 8.10,
	};

		// F distribution table ( alpha = 0.005 )
		// Col: degree of freedom of denominator
		// Rows: degree of freedom of numerator 
	const double CycleSlipEstimator2::F[ 20*3 ] = {  
			16210.72, 19999.50, 21614.74, 
			198.50, 199.00, 199.17, 
			55.55, 49.80, 47.47, 
			31.33, 26.28, 24.26, 
			22.78, 18.31, 16.53, 
			18.63, 14.54, 12.92, 
			16.24, 12.40, 10.88, 
			14.69, 11.04, 9.60, 
			13.61, 10.11, 8.72, 
			12.83, 9.43, 8.08, 
			12.23, 8.91, 7.60, 
			11.75, 8.51, 7.23, 
			11.37, 8.19, 6.93, 
			11.06, 7.92, 6.68, 
			10.80, 7.70, 6.48, 
			10.58, 7.51, 6.30, 
			10.38, 7.35, 6.16, 
			10.22, 7.21, 6.03, 
			10.07, 7.09, 5.92, 
			9.94, 6.99, 5.82
	};  

		// Chi-square table (alpha = 0.005, upper probablity) 
//const double CycleSlipEstimator2::chisqrRight[5] = { 7.879, 10.597, 12.838, 14.860, 16.750 };

		// Chi-square table (alpha = 0.001, upper probablity) 
	const double CycleSlipEstimator2::chisqrRight[5] = { 10.828, 13.816, 16.266, 18.467, 20.515 };

		// Chi-square table (alpha = 0.975, upper probablity) 
	const double CycleSlipEstimator2::chisqrLeft[5] = { 0.001, 0.051, 0.216, 0.484, 0.831 };

      /* Returns a gnnsRinex object, adding the new data generated when
       * calling this object.
       *
       * @param gData    Data object holding the data.
       */
   gnssRinex& CycleSlipEstimator2::Process(gnssRinex& gData)
      throw(ProcessingException)
   {

      try
      {

			Process(gData.header.epoch, gData.body, gData.header.epochFlag);			

         return gData;

      }
      catch(Exception& u)
      {
            // Throw an exception if something unexpected happens
         ProcessingException e( getClassName() + ":"
                                + u.what() );

         GPSTK_THROW(e);

      }

   }  // End of method 'CycleSlipEstimator2::Process()'



      /** Returns a satTypeValueMap object, adding the new data generated
       *  when calling this object.
       *
       * @param epoch     Time of observations.
       * @param gData     Data object holding the data.
       * @param epochflag Epoch flag.
       */
	satTypeValueMap& CycleSlipEstimator2::Process( const CommonTime& epoch,
																	satTypeValueMap& gData,
																	const short& epochflag )
     throw(ProcessingException)
	{
 		try
 		{
			// debug code vvv
			CivilTime ct(epoch);
			std::cout << ct.year << " " << ct.month << " " << ct.day 
							<< " " << ct.hour << " " << ct.minute << " " 
								<< ct.second << std::endl;
			// debug code ^^^ 

 			SatIDSet satRejectedSet;

				// This map stores the time-differenced observables 
			satTypeValueMap satTimeDiffData;

				// Clear sat time-differenced LI data
				// at the beginning of each processing epoch 
			satLITimeDiffData.clear();

				// Clear sat postfit residual data
				// at the beginning of each processing epoch 
			satPostfitResiduals.clear();
			satFixedCS.clear();

				// Clear sat Index recorder
			clearSatIndexRecorder();
				

 				// Loop through all the satellites
 			satTypeValueMap::iterator it;
 			for( it = gData.begin(); it != gData.end(); ++it )
 			{
 				
 					// SatID 
 				SatID sat( it -> first );
				typeValueMap tvm( it -> second );
				typeValueMap codeData;

					// Check required code data 
				double testValue(0.0);
				try
				{

						// Loop through the obsTypes to get their values
					//for( TypeIDSet::const_iterator it = obsTypes.begin();
					for( TypeIDSet::const_iterator it = sysObsTypes(sat.system).begin();
						  it != sysObsTypes(sat.system).end();
						  ++it )
					{
							// Present type	
						TypeID type( *it );

							// Try to get the value of this sat 
						testValue = tvm( type );

					}   // End of ' for( TypeIDSet::const_iterator it ... '
	

				}
				catch(...)
				{
						// If some value is missing, then schedule this satellite
						// for removal
					satRejectedSet.insert( sat );
					continue;

				}   // End of 'try-catch' block
				
					// We also need to check the LI types	
				if( useTimeDifferencedLI )
				{
					try
					{
						TypeIDSet typeSet = sysLITypes( sat.system );
						for( TypeIDSet::iterator it = typeSet.begin(); 
							  it != typeSet.end(); 
							  ++it )
						{
							TypeID type( *it );
	
							testValue = tvm( type );		
					
						} // End of ' for( TypeIDSet::iterator it = typeSet.begin(); '
					}
					catch( TypeIDNotFound& e )
					{
							// If some value is missing, then schedule this satellite
							// for removal
						satRejectedSet.insert( sat );
						continue;
					}

				} // End of ' if( useTimeDifferencedLI ) '


 					// Get filter data of this sat 
				bool timeInteruption(false);
 				timeInteruption = getSatFilterData( epoch, sat, tvm, 
																epochflag, satTimeDiffData ); 
				if( useTimeDifferencedLI )
				{
					try
					{
							// An initialization process is needed 
						TypeIDSet& tySet = sysLITypes(sat.system);
						TypeID liType = *(tySet.begin());


						if( sat.system == SatID::systemGPS )
						{
							if( liType == TypeID::LI )
								liType = TypeID::LI12;
						}

						TypeID type( liType.ConvertToTimeDiffType(1));
						size_t s( satLIDataDeque(sat)(type).size() );

						if( s < minBufferSize )
						{
							gData(sat)[TypeID::CSL1] = 1.0;
							satTimeDiffData.removeSatID(sat);
						} // End of ' if( s < minBufferSize ) '
					}
					catch( ... )
					{
							// Just continue 
						continue;
					}  

				} // End of ' if( useTimeDifferencedLI ) '

 			}   // End of ' for( it = gData.begin();  ... '

				// Remove satellites with missing data
			gData.removeSatID(satRejectedSet);


				// Now, let's model time-differenced code observations 
			size_t numOfSats( satTimeDiffData.numSats() );
			if( numOfSats >= 4 )
			{
					// Estimate Cycle Slips  
					// *** Only GPS observations on L1 and L2 freq *** 
					// *** Now, GPS L1, L2; Galileo E1, E5a
				Vector<double> cycleSlipVec;
				Matrix<double> cycleSlipCov;

				std::cout << "code blunder detection" << std::endl;
				SatIDSet badSats;
				for( int iteration=0; iteration<2; ++iteration )
				{
					SatIDSet tempBadSats;
					//tempBadSats = modelTimeDifferencedData( satTimeDiffData, gData, cycleSlipVec, cycleSlipCov );

					bool codeOnly(true);
					bool phaseOnly(false);
					tempBadSats = modelTimeDifferencedData( codeOnly, phaseOnly, 
																		 satTimeDiffData, gData );

						// Remove 'tempBadSats' from 'satTimeDiffData'
					satTimeDiffData.removeSatID( tempBadSats );

						// Insert 'tempBadSats' into 'badSats'
					badSats.insert(tempBadSats.begin(), tempBadSats.end());

				} // End of 'for( int iteration=0; iteration<0; ++iteration )'

					// Remove satellites with code blunder
				gData.removeSatID( badSats );

					// Detect cycle slip
//				std::cout << "detect cycle slips" << std::endl;
				cycleSlipDetection( satTimeDiffData, gData );

//				std::cout << "fix cycle slips" << std::endl;
//				if( correctCS )
//				{
//						// Correct cycle slip 
//					cycleSlipResolution( cycleSlipVec, cycleSlipCov, gData );
//				} // End of 'if( correctCS )'
			}   
			else
			{
				std::cout << "not enough sats" << std::endl;
			}

		
			return gData;
 		}
 
 		catch( Exception& u )
 		{
 				// Thrown an exception if something unexpected happens
 			ProcessingException e( getClassName() + ":" 
 											+ u.what() );
			std::cout << "u.what: " << u.what() << std::endl; 
 			GPSTK_THROW(e);
 
 		}   // End of 'try-catch' block
 
 
 
 	}   // End of 'satTypeValueMap& CycleSlipEstimator2::Process( ... '



		/* Get filter data
		 *
		 * Return the flag indicating whether the phase is continious in time
		 * and there are LLI cycle-slip declaration(TO DO!!!) 
		 *
		 * @param epoch
		 * @param sat
		 * @param tvMap      data related to this sat
		 * @param epochFlag 
		 *
		 */
	bool CycleSlipEstimator2::getSatFilterData( const CommonTime& epoch, 
															 const SatID& sat, 
															 typeValueMap& tvMap,
															 const short& epochFlag,
															 satTypeValueMap& satTimeDiffData )
	{

			// Time-interruption indicator 
		bool reportTimeInteruption(false);

			// Difference between current and former epochs, in sec
		double currentDeltaT(0.0);

			// Incorporate epoch and tvMap  
		epochTypeValueBody etvb(epoch, tvMap);
		
			//  Is this  a new visible sat?
		satEpochTypeValueMap::const_iterator itFormData = satFormerData.find( sat );
		if( itFormData == satFormerData.end() )
		{
				// This means a new sat
			satFormerData[sat] = etvb;
			//satFormerData[sat].epoch = epoch;

				// Just return
			reportTimeInteruption = true;; 
			return reportTimeInteruption;
		
		}   // End of ' if( itFormData == satFormerData.end() ) '


			// Get the difference between current epoch and former epoch,
			// in seconds
		currentDeltaT = ( epoch - satFormerData[sat].epoch );
		
		if( currentDeltaT > deltaTMax )
		{
				// This means time gap of this sat, report time interuption
			reportTimeInteruption = true;

				// Updata satFormerData 
			satFormerData[sat] = etvb;

			if( useTimeDifferencedLI )
			{
					// Clear data of this sat
				satLIDataDeque.erase(sat);
			}

				// Just return
			return reportTimeInteruption;

		}
		
			// TO DO!!!
			// epochFlag 

			// Now, everything is OK!
			// Loop through the obsTypes to compute time-differenced values
		//for( TypeIDSet::const_iterator it = obsTypes.begin();
		for( TypeIDSet::const_iterator it = sysObsTypes[ sat.system ].begin();
			  it != sysObsTypes[ sat.system ].end();
			  ++it )
		{
			TypeID type( *it );

			
				// Time differenced value of this type 
			satTimeDiffData[sat][type] = tvMap(type) - 
														satFormerData(sat)(type);

		
		}   // End of ' for( TypeIDSet::const_iterator it = obsTypes.begin(); ... '

			// Insert weight info into satTimeDiffData with
			// the consideration of elevation 
		try
		{
			double wNow( tvMap(TypeID::weight) );
			double wPrevious( satFormerData(sat)(TypeID::weight) );
				
				// Compute weight factor 
			satTimeDiffData[sat][TypeID::weight] = wNow*wNow*wPrevious*wPrevious/( wNow*wNow + wPrevious*wPrevious); 

				// Store elevation weight factor in 'satTimeLIDeque'  
			if( useTimeDifferencedLI )
			{
					// Insert present weight into satLIDataDeque
				satLIDataDeque[sat].eleWeightBuffer.push_back( wNow );
				satLIDataDeque[sat].epochBuffer.push_back( epoch );

					// Keep its size no more than 'maxBufferSize'
				if( satLIDataDeque(sat).eleWeightBuffer.size() > maxBufferSize )
				{
					satLIDataDeque(sat).eleWeightBuffer.pop_front();
					satLIDataDeque(sat).epochBuffer.pop_front();
				} // End of ' if( satLIDataDeque(sat).eleWeightBuffer.size() ... '

					// Loop through the liTypes to compute time-differenced values
				TypeIDSet liTypes( sysLITypes(sat.system) );
				for( TypeIDSet::const_iterator it = liTypes.begin(); 
					  it != liTypes.end(); 
					  ++it )
				{
					TypeID type( *it );
	
						// Result type
					TypeID type1d(type.ConvertToTimeDiffType(1));
					TypeID type2d(type.ConvertToTimeDiffType(2));
	
						// Time differenced value of this type 
					double deltaLI(0.0);
	
					deltaLI = tvMap(type) - satFormerData(sat)(type);
	
//					std::cout << sat << std::endl;
//					std::cout << "deltaLI: " << deltaLI << std::endl;
					//satLITimeDiffData[sat][TypeID::deltaLI] = deltaLI;
	
						// Store present deltaLI in satLITimeDiffData for output later 
					satLITimeDiffData[sat][type1d] = deltaLI;
	
						// Store present deltaLI in the deque 
					satLIDataDeque[sat][type1d].push_back( deltaLI );
	
						// Keep its size less than maxBufferSize
					try
					{
						size_t s( satLIDataDeque(sat)(type1d).size() );	
						if( s > maxBufferSize )
						{
								// Too many data, delete the 1st one
							satLIDataDeque(sat)(type1d).pop_front();
						}
	
					}
					catch( ValueNotFound& e )
					{
						GPSTK_THROW( e );
					}
	
						// Record deltaLI in etvb, for later update of satFormerData 
					etvb[type1d] = deltaLI;
	
						// Double time-differenced LI 
					double deltaDeltaLI(0.0);
					epochTypeValueBody::const_iterator iter = 
													satFormerData(sat).find( type1d );	
					if( iter != satFormerData(sat).end() )
					{
						deltaDeltaLI = deltaLI - satFormerData(sat)(type1d);
						satLITimeDiffData[sat][ type2d ] = deltaDeltaLI;
//						std::cout << "deltaDeltaLI: " << deltaDeltaLI << std::endl;
					}
					
				}   // End of ' for( TypeIDSet::const_iterator it =  ... '


		
			} // End of ' if( useTimeDifferencedLI ) '
		}
		catch( ... )
		{
			Exception e( "SatID or TypeID::weight cannot be found!!!" );
			GPSTK_THROW( e );
		}
			// Loop through the liTypes to compute time-differenced values
//		if( useTimeDifferencedLI )
//		{
//			TypeIDSet liTypes( sysLITypes(sat.system) );
//			for( TypeIDSet::const_iterator it = liTypes.begin(); 
//				  it != liTypes.end(); 
//				  ++it )
//			{
//				TypeID type( *it );
//
//					// Result type
//				TypeID type1d(type.ConvertToTimeDiffType(1));
//				TypeID type2d(type.ConvertToTimeDiffType(2));
//
//					// Time differenced value of this type 
//				double deltaLI(0.0);
//
//				deltaLI = tvMap(type) - satFormerData(sat)(type);
//
//				std::cout << "deltaLI: " << deltaLI << std::endl;
//				//satLITimeDiffData[sat][TypeID::deltaLI] = deltaLI;
//
//					// Store present deltaLI in satLITimeDiffData for output later 
//				satLITimeDiffData[sat][type1d] = deltaLI;
//
//					// Store present deltaLI in the deque 
//				satLIDataDeque[sat][type1d].dataBuffer.push_back( deltaLI );
//
//					// Keep its size less than maxBufferSize
//				try
//				{
//					size_t s( satLIDataDeque(sat)(type1d).dataBuffer.size() );	
//					if( s > maxBufferSize )
//					{
//							// Too many data, delete the 1st one
//						satTimeLIDeque(sat)(type1d).dataBuffer.pop_front();
//					}
//
//				}
//				catch( ValueNotFound& e )
//				{
//					GPSTK_THROW( e );
//				}
//
//					// Record deltaLI in etvb, for later update of satFormerData 
//				etvb[type1d] = deltaLI;
//
//					// Double time-differenced LI 
//				double deltaDeltaLI(0.0);
//				epochTypeValueBody::const_iterator iter = 
//												satFormerData(sat).find( type1d );	
//				if( iter != satFormerData(sat).end() )
//				{
//					deltaDeltaLI = deltaLI - satFormerData(sat)(type1d);
//					satLITimeDiffData[sat][ type2d ] = deltaDeltaLI;
//					std::cout << "deltaDeltaLI: " << deltaDeltaLI << std::endl;
//				}
//				
//			}   // End of ' for( TypeIDSet::const_iterator it =  ... '
//
//		}   // End of ' if( useExternalIonoDelayInfo ) '


			// Update satFormerData data
		satFormerData[sat] = etvb;
		return reportTimeInteruption;


	}   // End of 'void CycleSlipEstimator2::getFilterData( const SatID& sat, ... '


		/** Model time-differenced code data
		 *
		 * ...
		 *
		 */
	SatIDSet CycleSlipEstimator2::modelTimeDifferencedData( 
																bool& codeOnly,
																bool& phaseOnly,
																satTypeValueMap& satTimeDiffData,
																satTypeValueMap& stvm, 
																bool estCS ) 
	{

		SatIDSet badSatSet;


			// clear the record the info, which is changing along with the 
			// equation system
		clearSatIndexRecorder();

			// Extract the specified ObsTypeSet, instead of the original
			// version only processing 'sysObsTypes' 
		SysTypeIDSetMap sysObsTypes2;

		satTypeValueMap obsData;

		if( codeOnly )
		{
			if( phaseOnly )
			{
				Exception e( getClassName() + "'PhaseOnly' and 'codeOnly' cann't be true at the same time");
				GPSTK_THROW(e);
			}  // End of 'if( phaseOnly )'

			sysObsTypes2 = sysCodeObsTypes;
		}
		else if( phaseOnly )
		{
			sysObsTypes2 = sysPhaseObsTypes;
		}
		else{
			sysObsTypes2 = sysObsTypes;
		} // End of 'if( codeOnly )'

			// Extract the specified types 
		obsData = satTimeDiffData.extractTypeID( sysObsTypes2 );

			// Num of available sats
		size_t numOfSats( obsData.numSats() );

		// debug code vvv
		std::cout << "numOfSats: " << numOfSats << std::endl;
		// debug code ^^^ 


			// Total num of elements in satTimeDiffData
		size_t numOfElements( obsData.numElements() );

			// Valid sat set
//		validSatSet.clear();	// clear the set of last epoch 	
//		validSatSet = satTimeDiffData.getSatID();

		// debug code vvv
		std::cout << "numOfElements: " << numOfElements << std::endl;
		// debug code ^^^ 


			// Num of measurements
			// phase, code and pseudo-ionospheric observables 
		size_t numIonoObs( ionoWeighted ? numOfSats : 0 );
		size_t numMeas( numOfElements + numIonoObs );

			// Num of unknowns, also the same order in the time-diff equation 
			// 3				receiver coordinate displacements(if it is moving)
			// 1				receiver clock offset variation 
			// numOfSats   ionospheric delay variation on the first frequency
			//					e.g. L1 for GPS  
			// numOfSats*n cycle-slip parameters, n is the number of frequency     

//		size_t numCoorVar( (staticReceiver?0:3) );

			// Compute the number of unknowns, which are common for code and 
			// phase observables: coordinates, clock receiver, ionosphere  
		numUnknowns = numCoorVar + 1 + numOfSats;
		
		numCommonUnks = numCoorVar + 1;

			// Record the number of Cycle-Slip parameters regarding to all the 
			// sat system
		size_t numAmbUnknowns( 0 );

		if( !codeOnly && estCS )
		{
				// Loop through the 'sysObsTypes'
			for( SysTypeIDSetMap::const_iterator itS = sysObsTypes.begin(); 
				  itS != sysObsTypes.end(); 
				  ++itS )
			{
					// System
				SatID::SatelliteSystem sys( itS -> first );
	
					// ObsTypes TypeIDSet
				TypeIDSet types( itS -> second );
	
					// Size of types
				size_t numTypes( types.size() );
	
					// Num of sats with this sys
				size_t numSats( satTimeDiffData.extractSatSystem( sys ).numSats() );
	
				try
				{
	
						// Num of Phase frequency
					size_t numPhaseFreq( sysObsTypeBandSet( sys, ObsID::otPhase).size() );
						// Add ambiguity parameters, numSats*(num of Phase frequency )  
					numUnknowns += numSats*numPhaseFreq; 
					numAmbUnknowns += numSats*numPhaseFreq;
				}
				catch(ValueNotFound& e)
				{
						// No phase type, error 
					GPSTK_THROW(e);
				}
					
			} // End of ' for( SysTypeIDSetMap::const_iterator itS ... '
		}  // End of ' if( !codeOnly ) '

		// debug code vvv
		std::cout << "numOfSats: " << numOfSats << " numUnknowns: " 
						<< numUnknowns << " numAmbUnknowns: " << numAmbUnknowns << std::endl;
		// debug code ^^^ 

			// Resize csVec and csCov
//		csVec.resize( numAmbUnknowns, 0.0 );
//		csCov.resize( numAmbUnknowns, numAmbUnknowns, 0.0 );

			// Handy copies 
		size_t rows( numMeas ), columns( numUnknowns );

		// debug code vvv
		std::cout << "rows and columns: " << rows << " " << columns  << std::endl;
		// debug code ^^^ 
			// Measurement vector
		Vector<double> y( rows, 0.0 );

			// Weight matrix for measurements  
		Matrix<double> rMatrix( rows, rows, 0.0 );

			// Design matrix
		Matrix<double> hMatrix( rows, columns, 0.0 );

		
			// Unit weight std  
//		GNSSObsSTDTables obsStd;
		double unitWeightStd( obsStd.getGNSSObsSTD( SatID::systemGPS, 
																  TypeID::prefitPC ) );
//		unitWeightStd = 0.1;

			//* Now fill design  matrix
			//* Order of observables can be arbitrary, but the order of unkonwns is 
			//* arranged as above
			
			// Row index
		size_t i(0);

			// sat index  
		size_t j(0);

			// accumulativ Num of phase frequency with the consideration of all 
			// sat system
		size_t acPhaseFreqNum(0);
		
		for( satTypeValueMap::const_iterator itStvm = satTimeDiffData.begin(); 
			  itStvm != satTimeDiffData.end(); 
			  ++itStvm )
		{

				// SatID 
			SatID sat( itStvm->first );
//			std::cout << sat << std::endl;
			typeValueMap tvm( itStvm->second );

				// Sat system
			SatID::SatelliteSystem sys( sat.system );
			TypeIDSet obsTypes( sysObsTypes2(sys) );

			for( TypeIDSet::const_iterator itObsTypes = obsTypes.begin();
				  itObsTypes != obsTypes.end(); 
				  ++itObsTypes )
			{
					// Code TypeID 
				TypeID type( *itObsTypes );

				// debug code vvv
//				std::cout << "type= " << type << std::endl;
				// debug code ^^^ 
				
				y(i) = tvm( type );
	
					// Zenith weight
				double observableStd( obsStd.getGNSSObsSTD(sys, type ) );
				double weight( unitWeightStd/observableStd );
				weight *= weight;
	
				rMatrix( i, i ) = tvm(TypeID::weight) * weight; 
//				rMatrix( i, i ) =  weight; 


					// ***Now fill design matrix

				if( !staticReceiver )
				{
						// Coefficients for the coordinate displacement deltadx/y/z
						// but here is an approximation:
					hMatrix( i, 0 ) = stvm(sat)( TypeID::dx );
					hMatrix( i, 1 ) = stvm(sat)( TypeID::dy );
					hMatrix( i, 2 ) = stvm(sat)( TypeID::dz );
				}

					// Coefficients for the variation of receiver clock  
				hMatrix( i, numCoorVar ) = 1.0;

					// Coefficients for the variation of iono delay and possible
					// cycle slips, j indicates the sat index
				//RinexObsType rot( type.ConvertToRinexObsType(SatID::systemGPS) );
				RinexObsID roi( type.ConvertToRinexObsID(sys) );

					// 'GetCarrierBand' is defined in the file 'TypeID.hpp'
					// but not a member function of TypeID class  
				int band( GetCarrierBand( roi ) );
				//std::cout << sat << " " << type << " band:" << band << "--->";
				int bandIndex( getBandIndex(sys, roi.type, band) );

//				std::cout <<"bandIndex: " << bandIndex << std::endl;
					
					// 'getMiu' function is defined in file 'GNSSconstants' class
				double miu( getMiu( sys, band ) );

					// Fill the efficient
				if( roi.type == ObsID::otPhase )
				{
						// For the variation of iono delay
					hMatrix( i, numCoorVar+1+j ) = -1 * miu;  

						// For the cycle slips
					//hMatrix( i, numCoorVar+1+(bandIndex+1)*numOfSats+j ) = 
					if( estCS )
					{
						hMatrix( i, numCoorVar+1+numOfSats+acPhaseFreqNum+bandIndex ) = 
							getWavelength( sat, band );
					} 

						// Record the Ambiguity index
					ambColSat[ acPhaseFreqNum+bandIndex ] = std::make_pair(sat, type);
				}
				else if( roi.type == ObsID::otRange )
				{
						// This means code observable 
					hMatrix( i, numCoorVar+1+j ) = miu; 
				}
				else
				{
					GPSTK_THROW( TypeIDNotFound(getClassName() + " unrecognised type!") );
				}

					// Record the row index of this sat
				rowSat[i] = std::make_pair(sat, type);

					//	Increment for row 
				i++;
	
			}   // End of 'for( TypeIDSet::const_iterator itObsTypes ... '


				// Add pseudo measurement of ionospheric delay variation
			if( ionoWeighted )
			{
				if( useTimeDifferencedLI )
				{
					double ion(0.0), value(0.0), ionVar(1.0), var(1.0);
	
						// Pick one LI type from sysLiTypes, the first one by default
					TypeIDSet liTypes( sysLITypes(sat.system) );
					TypeID type( *(liTypes.begin()));
	
						// Get band
					int b1( GetLITypeCarrierBand( type, 1 ) );
					int b2( GetLITypeCarrierBand( type, 2 ) );
	
						// Get miu 
					double miu1( getMiu(sat.system, b1) );
					double miu2( getMiu(sat.system, b2) );
	
					//std::cout << sat << " " <<  type <<  " b1: " << b1 << " b2: " << b2 
					//				<< " miu1: " << miu1 << " miu2: " << miu2 << std::endl; 
						// Convert type to differential form 
					TypeID dLI( type.ConvertToTimeDiffType(1) );
	
					getSmoothDeltaLI( sat, dLI, value, var );
	
						// Compute deltaLI and its variance
					ion = value/(miu2-miu1);
					ionVar = var/((miu2-miu1)*(miu2-miu1));
	
					rMatrix( i, i ) = unitWeightStd*unitWeightStd/ionVar;
	
//					std::cout << sat << " " <<  type << " ion: " << ion << "iStd: " << std::sqrt(ionVar) << std::endl;
				}
				else
				{
						// Empirical value
					y(i) = 0;
					double ionoWeight( unitWeightStd/(0.05) );
					rMatrix( i, i ) = ionoWeight*ionoWeight;
				} // End of 'if( useTimeDifferencedLI )'
	
					// Only one coefficient for this virtual obs  
				hMatrix( i, numCoorVar+1+j ) = 1.0;

					// Record the row index of this sat
				rowSat[i] = std::make_pair(sat, TypeID::ionoL1);

				i++;   // Donot forget increment for the row!!! 

					// Record this row 
//				rowSat[i] = std::make_pair(sat, type);

			} // End of 'if( ionoWeighted )'
				

				// Record the sat/iono pair
			ionColSat[j] = sat;

				// Preparation for next sat
//			i++;   // Donot forget!!! 
			j++;

				// Increment of acPhaseFreqNum
			if( !codeOnly && estCS )
			{
				try
				{
					acPhaseFreqNum += sysObsTypeBandSet( sys, ObsID::otPhase ).size();
				}
				catch( ValueNotFound& e )
				{
						// no phase type data 
					Exception u( getClassName() + e.what() );
					GPSTK_THROW(u);
				}
			} // End of 'if( !codeOnly )'

		}   // End of ' for( satTypeValueMap::const_iterator itStvm =  ... '


		// debug code vvv
		std::cout << "hMatrix: " << std::endl;
		std::cout << hMatrix << std::endl;
	//	exit(-1);
//		std::cout << "j: " << j << std::endl;
		// debug code ^^^

			// Now employ a solver 
		
		if( !phaseOnly ) // For cases: cose, and code+phase
		{
				// Adopt LMS solver, no priori info
			SolverWMS solver;
			solver.Compute( y, hMatrix, rMatrix );

				// Store the results
			solution = solver.solution; 
			covMatrix = solver.covMatrix;
		}
		else{	// Only phase observables TODO

			Exception e(getClassName() + ": cannot deal with phaseOnly data");
			GPSTK_THROW(e);
				
//				// Adopt kalmanfiter, with priori info
//			SimpleKalmanFilter kalmanFilter( solution, covMatrix );
//
//				// Dummy time update to initialise 'xhatminus' and 'xhat'
//			kalmanFilter.TimeUpdate();
//
//				// Phase measurement updates
//			kalmanFilter.MeasUpdate( y, hMatrix, inverse(rMatrix) );
//
//				// Store the results
//			solution = kalmanFilter.xhat;
//			covMatrix = kalmanFilter.P;
//
		} // End of 'if( !phaseOnly )'

			// Tag the solution 
			// Tag the ionosphere estimates
		for( size_t i=0; i<numOfSats; i++ )
		{
			SatID sat( ionColSat[i] );
			
			double iono( solution(numCoorVar + 1 + i) );

				// Insert this value 
			satIonoAmbEstimates[sat][TypeID::iono] = iono;

		} // End of 'for( int i=0; i<numUnknowns, i++ )'

		if( !codeOnly && estCS )
		{
				// This is important to allocate buffer
			csVec.resize( numAmbUnknowns, 0.0 );
			csCov.resize( numAmbUnknowns, numAmbUnknowns, 0.0 );

				// Store cycle slip estimates 
			for( i=0; i<numAmbUnknowns; ++i)
			{
				csVec(i) = solution( numCoorVar + 1 + numOfSats + i ); 
				for( j=0; j<numAmbUnknowns; ++j )
				{
					csCov(i, j) = covMatrix( numCoorVar + 1 + numOfSats + i, 
															  numCoorVar + 1 + numOfSats + j );  
				} // End of ' for( i=0; i<numAmbUnknowns; ++i) '
	
			} // End of 'for( i=0; i<2*numOfSats; ++i)'
		} // End of 'if( !codeOnly )'
		
			// Adjusted measurement vector
		yhat = hMatrix * solution;
		Qyhat = hMatrix * covMatrix * transpose( hMatrix );


			// Postfit residuals
		postfitResiduals.resize( rows, 1, 0.0 );
		postfitResiduals = yhat - y; 

			// *** Now store the postfitResiduals 
			
			// Also sat order in postfit residuals   
//		SatIDSet satSet( satTimeDiffData.getSatID() );
//		size_t index(0); 

//		for( SatIDSet::const_iterator itSat = satSet.begin(); 
//			  itSat != satSet.end(); 
//			  ++itSat )
//		{
//				// SatID 
//			SatID sat( *itSat );
//
//				// Get the value, this order depends on obsTypes 
//			for( TypeIDSet)
//				
//				// Preparation for the next sat 
//			index++;
//
//		}   // End of ' for( satTypeValueMap::const_iterator itSat ... '

//		// debug code vvv
//		std::cout << "solution: " << std::endl;
//		std::cout << solver.solution << std::endl;
//		std::cout << "postfitRes: " << std::endl;
//		std::cout << postfitResiduals << std::endl;
//		// debug code ^^^
//			
			// post unit weight std
		Matrix<double> vTpv( transpose(postfitResiduals) * 
								  rMatrix * postfitResiduals );

			// Degree of freedom  
		size_t df( rows - numUnknowns );
		
		dfCode = df;

		sigma0hat = std::sqrt( vTpv(0,0) / df );

		// debug code vvv
		std::cout << "sigma0: " << unitWeightStd << std::endl;
		std::cout << "sigma0hat: " << sigma0hat << std::endl;
		// debug code ^^^

//			// Examination of the variance factor 
//		double st( vTpv(0,0) /(unitWeightStd*unitWeightStd) );
//
//			// Chi2Distribution
//		Chi2Distribution chi2Obj;
//		double upperTailPro( chi2Obj.Q(st, df) );
//		std::cout << "upperTailPro: " << upperTailPro << std::endl;
//
//		if( upperTailPro < 0.0001 || upperTailPro > 0.9999 )
//		{
//			std::cout << "Incorrect adjust model: " << std::endl;
//		}

//		if( st > chisqrRight || st < chisqrLeft )
//		{
//				// Incorrect adjustment model
//			std::cout << 
//
//		}

//
//
//		if( codeOnly )
//		{
				// Compute Qvv
			Matrix<double> Q( inverse(rMatrix) );
			Matrix<double> Nbb( transpose( hMatrix ) * rMatrix * hMatrix );
			Matrix<double> invNbb( inverse(Nbb) );
			Matrix<double> temp( hMatrix * invNbb * transpose( hMatrix ) );
			Matrix<double> Qvv( Q - hMatrix * invNbb * transpose( hMatrix ) );
	//		// debug code vvv
	////		std::cout << "Qvv: " << std::endl;
	////		std::cout << Qvv << std::endl;
	////		exit(-1);
	//		// debug code ^^^
	//
	//	
	//			// Compute normalized residuals
			Vector<double> codeNormRes( numOfElements, 0.0 );
			size_t k(0);
	//		size_t numCodeTypes( obsTypes.size() );
			for( size_t i=0; i<rows; i++ )
			{
				double post(postfitResiduals(i, 0));
				SatID sat(rowSat[i].first);
				TypeID type( rowSat[i].second );
				satPostfitResiduals[sat][type] = post;
			
					// Convert to rinex obs type 
				RinexObsID roi( type.ConvertToRinexObsID( sat.system ) );
//				if( roi.type == ObsID::otRange )
				{
					double normRes( post/( sigma0hat* std::sqrt( Qvv(i,i) ) ) );
					codeNormRes(k) = normRes;
	
						// Threshold
					StudentDistribution stuObj;
					GaussianDistribution normal;
					//double upperTailPro( normal.Q( std::abs( normRes ) ) );
					double upperTailPro( stuObj.Q( std::abs( normRes ), df ) );
					
					std::cout <<  sat << " " << type << " normRes: " << normRes <<  " uppreTail: " << upperTailPro  << std::endl;
	
//					if( upperTailPro < 0.0079 )
					if( upperTailPro < normalTestAlpha )
//					if( upperTailPro < 0.0012 ) // iono weighted
					//if( std::abs(normRes) > t[df-1] )
					{
						if( !codeOnly && !estCS ) // CS detection
						{
								// Insert CS 
							if(type == prefitL1 )
							{
								stvm(sat)[TypeID::CSL1] = 1.0;	
							}

						} // End of 'if( !codeOnly && !estCS ) // CS detection'
							// Code Blunder
						badSatSet.insert(sat);
					}  // End of 'if( upperTailPro < 0.05 ) '
					k++;
				} // End of 'if( roi.type == ObsID::otRange ) '
	
			} // End of 'for( size_t i=0; i<rows; i++ )'
//		} // End of 'if( codeOnly )'
	
	//		std::cout << "codeNormRes: " << std::endl;
	//		std::cout << codeNormRes << std::endl;
	
	//		// debug code vvv
	//		Stats<double> statistic;
	//		statistic.Add(normRes);
	//		std::cout << "normRes ave: " << statistic.Average() << " var: " 
	//						<< statistic.StdDev() <<  std::endl;
	//		// debug code ^^^
	
			return badSatSet;

	}   // End of ' void CycleSlipEstimator2::modelTimeDifferencedCode( ...'


		/* Return post residual regarding to a specific sat and a specific 
		 * type. These types should be corresponding to the data member
		 * obsTypes.
		 *
		 * @param sat
		 * @param type  I postprefit type, lile: postprefitL1, postdLI
		 * 
		 */
	double CycleSlipEstimator2::getSatPostfitResidual( SatID sat, TypeID type )
	{

			// postfit residual value
		double postRes(0.0);

			// Convert postfitType(usr) to prefitType(engineer)
		TypeID prefitType = type.ConvertToPrefitTypeID();

		return satPostfitResiduals(sat)(prefitType);

		// debug vvv
//std::cout << "postType: " << type << " -> preType: " << prefitType << std::endl; 
		// debug ^^^ 

			// Find the index of sat in data member 'validSatIDSet' 
		SatIDSet::const_iterator itSatID = validSatSet.begin();
		size_t satIndex(0);
		while( *itSatID != sat )
		{ ++satIndex; ++itSatID;}
//		std::cout << sat << "is indexed by: " << satIndex << std::endl;

			// Find the index of converted type in data member 'obsTypes'
		size_t typeIndex(0);
		if( type != TypeID::postfitDeltaIono )
		{
			//TypeIDSet::const_iterator itObsTypes = obsTypes.begin();
			TypeIDSet::const_iterator itObsTypes = sysObsTypes(sat.system).begin();
			while( *itObsTypes != prefitType )
			{ ++typeIndex; ++itObsTypes;}
		}
		else{
			typeIndex = sysObsTypes[sat.system].size();
		} // End of ' if( type != TypeID::postfitDeltaIono ) '

			// Compute the index of this pre-type in postfitResiduals
		size_t row( satIndex*(sysObsTypes[sat.system].size() + 1) + typeIndex );
//		std::cout << row << " is the number in postfitresiduals" << std::endl;

		postRes = postfitResiduals(row, 1);

		return postRes;
	}  // End of 'double getPostfitResidual( SatID sat, TypeID type )'

 

		/// Return a  given a band in obsTypes
	void CycleSlipEstimator2::getBandList( SatID::SatelliteSystem sys )
	{
	
			// Form a Band list from obsTypes of given system  
		int currentBand(-1);

			// Get the obsTypes of this sys 
		TypeIDSet types;
		SysTypeIDSetMap::const_iterator it = sysObsTypes.find( sys );
		if( it != sysObsTypes.end() )
		{
			types = it -> second; 
				
				// Get the band list, stored in 'sysBandSet'
			for( TypeIDSet::const_iterator itType = types.begin();
				  itType != types.end();
				  ++itType )
			{
				TypeID type( *itType );
				//RinexObsType rot( type.ConvertToRinexObsType(sys) );
				RinexObsID roi( type.ConvertToRinexObsID(sys) );
	
					// Get the band
				currentBand = GetCarrierBand( roi );

				if( currentBand == -1 )
				{
					Exception e("CycleSlipEstimator2: unrecognised band");
					GPSTK_THROW( e );
				}
				
					// Insert this band into the set 
			 	sysObsTypeBandSet[ sys ][ roi.type ].insert( currentBand );
			
			} // End of 'for( TypeIDSet::const_iterator itType = types.begin();...' 

		}
		else
		{
			Exception e("CycleSlipEstimator2: no obstypes for some sys");
			GPSTK_THROW(e);
		}

	}   // End of ' void getBandList( int band ) '



	int CycleSlipEstimator2::getBandIndex( SatID::SatelliteSystem sys, 
													  ObsID::ObservationType ot,
													  int band )
	{
		int index(0);
		
		std::set<int> bandSet( sysObsTypeBandSet( sys, ot) );
		std::set<int>::const_iterator it = bandSet.begin(); 
		while( (*it) != band )
		{
			++it;
			++index;
		}
				
		return index;

	}   // End of ' '


	   /* Method to set the maximum buffer size for data, in samples.
       *
       * @param maxBufSize      Maximum buffer size for data, in samples.
       *
       * \warning You must not set a value under minBufferSize, which
       * usually is 5.
       */
   CycleSlipEstimator2& CycleSlipEstimator2::setMaxBufferSize(const int& maxBufSize)
   {
         // Don't allow buffer sizes less than minBufferSize
      if (maxBufSize >= minBufferSize)
      {
         maxBufferSize = maxBufSize;
      }
      else
      {
         maxBufferSize = minBufferSize;
      }

      return (*this);

   }  // End of method 'LICSDetector2::setMaxBufferSize()'

		/// CycleSlipEstimator2::SatTypeValueDeque::operator()
	CycleSlipEstimator2::TypeValueDeque& CycleSlipEstimator2::SatTypeValueDeque::operator()( const SatID& sat )
		throw( SatIDNotFound )
	{
		SatTypeValueDeque::iterator itSatLIData = (*this).find( sat );	

		if( itSatLIData != (*this).end() )
		{
			return itSatLIData -> second;
		}
		else
		{
			SatIDNotFound e(StringUtils::asString(sat) + "not found!");
			GPSTK_THROW(e);
		}  // End of ' if( itLIData != (*this).end() ) '

	} // End of ' CycleSlipEstimator2::TypeValueDeque& ... '

		/// CycleSlipEstimator2::TypeValueDeque::operator
	std::deque<double>& CycleSlipEstimator2::TypeValueDeque::operator()( const TypeID& ty )
		throw( TypeIDNotFound )
	{

	   TypeValueDeque::iterator itLIData = (*this).find( ty );

		if( itLIData != (*this).end() )
		{
			return itLIData -> second;
		}
		else
		{
			TypeIDNotFound e(StringUtils::asString(ty) + "not found!");	
			GPSTK_THROW(e);
		}  // End of ' if( itLIData != (itSatLIData).second.end() ) '

	} // End of ' TimeLIDeque& CycleSlipEstimator2::SatTimeLIDeque ... '


//		/// CycleSlipEstimator2::SatTimeLIDeque::operator()
//	std::map< CommonTime, double >& CycleSlipEstimator2::SatTimeLIDeque::operator()( const SatID& sat )
//		throw( ValueNotFound )
//	{
//		SatTimeLIDeque::iterator itLIData = (*this).find( sat );	
//
//		if( itLIData != (*this).end() )
//		{
//			return itLIData -> second;
//		}
//		else
//		{
//			ValueNotFound e("CycleSlipEstimator2: sat system doesn't exist");
//			GPSTK_THROW(e);
//		}  // End of ' if( itLIData != (*this).end() ) '
//
//	} // End of ' std::map< CommonTime, double >& operator()( const SatID& sat ) '
//
//
//
//		/// CycleSlipEstimator2::SatTimeLIDeque::operator()
//	double& CycleSlipEstimator2::SatTimeLIDeque::operator()( const SatID& sat, 
//																			  const CommonTime& time )
//		throw( ValueNotFound )
//	{
//		
//		SatTimeLIDeque::iterator itLIData = (*this).find( sat );
//		if( itLIData != (*this).end() )
//		{
//			std::map< CommonTime, double >::iterator itTimeLIData = (itLIData -> second).find( time );		
//			if( itTimeLIData != (itLIData -> second).end() )
//			{
//				return itTimeLIData -> second;
//			}
//			else
//			{
//				ValueNotFound e("CycleSlipEstimator2: time tag doesn't exist");
//				GPSTK_THROW(e);
//			}
//
//		}
//		else
//		{
//			ValueNotFound e("CycleSlipEstimator2: sat system doesn't exist");
//			GPSTK_THROW(e);
//		}  // End of ' if( itLIData != (*this).end() ) '
//
//	} // End of ' double& CycleSlipEstimator2::SatTimeLIDeque::operator() '


		/// CycleSlipEstimator2::SysObsTypeSet::operator()
	std::set<int>& CycleSlipEstimator2::SysObsTypeSet::operator()( const SatID::SatelliteSystem sys, 
																					  const ObsID::ObservationType ot )
		throw( SatIDNotFound, ValueNotFound )
	{
			// Find the bandSet related to the given sys and given ot  
		SysObsTypeSet::iterator itSTB = (*this).find( sys );
		if( itSTB != (*this).end() )
		{
			std::map<ObsID::ObservationType, std::set<int> >::iterator itTB = 
														 (itSTB -> second).find( ot );
			if( itTB != (itSTB -> second).end() )
			{
				return itTB -> second;
				
			} // End of 'if( itTB != (itSTB -> second).end() )'
			else
			{
				ValueNotFound e("CycleSlipEstimator2: obsType doesn't exist"); 
				GPSTK_THROW(e);
			}

		}
		else
		{
			SatIDNotFound e("CycleSlipEstimator2: sat system doesn't exist");
			GPSTK_THROW(e);
		}  // End of ' if( itSTB != sysObsTypeBandSet.end() ) '

	} // End of ' set<int>& SysObsTypeSet::operator() ... '

		/* Return average deltaLI and its variance from 'satTimeLIDeque' given
		 * a sat
		 *
		 * @param sat		I 
		 * @param adLI		I/O  average of deltaLI 
		 * @param var		I/O  variance of average of deltaLI 
		 *
		 */
	void CycleSlipEstimator2::getSmoothDeltaLI( const SatID& sat, const TypeID& ty, 
			   											 double& adLI, double& var )
	{
		TypeValueDeque tempData;
		try
		{
			tempData = satLIDataDeque( sat );
			std::deque<CommonTime> epochs( tempData.epochBuffer );
			std::deque<double> weights( tempData.eleWeightBuffer );
			std::deque<double> data( tempData(ty) );

				// Get its size
			size_t s( epochs.size() );
			if( s == 0 )
			{ Exception e("no data in buffer!!!"); GPSTK_THROW(e); };

			if( s != data.size() || s != weights.size() )
			{ Exception e("unmatched size!!!"); GPSTK_THROW(e); };

				// Simply Compute the average of data by the object of Stats
			Stats<double> stat; 
			for( std::deque<double>::const_iterator itd = data.begin(); 
				  itd != data.end(); 
				  ++itd )
			{
				stat.Add( *itd );
			} // End of ' for( std::deque<double>::const_iterator ... '

			adLI = stat.Average();
			var = stat.Variance();

				// Std of deltaLI in zenith direction
//			double lStd( obsStd.getGNSSObsSTD( sat.system, TypeID::prefitL1 ) );
//			double dLIVar( 4*lStd*lStd );

//			var = dLIVar/(s*s);
//				// Compute the variance of the weighted average  
//			double sum(0.0);
//			for( size_t i=0; i<s; i++ )
//			{
//				double c( 1.0/weights[i] );	
//				sum += c*c;
//			} // End of 'for( size_t i=0; i<s; i++ )'
//
//			if( sum != 0 )
//			{	
//				var = var*sum;
//			}
//			else
//			{
//				Exception e("Error in Computing weight");
//			} // End of 'if( sum != 0 )'

		}
		catch( ... )
		{
			Exception e( StringUtils::asString(sat) + "not found or any other problem" );
			GPSTK_THROW(e);
		}
	} // End of 'void CycleSlipEstimator2::getSmoothDeltaLI( ... ' 



			/** Cycle slip resolution
			 *
			 * @param csVec	I float estimates of cycle slip
			 * @param csCov   I
			 * @param gData   I/O
			 *
			 */
	void CycleSlipEstimator2::cycleSlipResolution( Vector<double>& csVec,
									  							  Matrix<double>& csCov, 
																  satTypeValueMap& gData )
	{

		try
		{
				// Fix the float cycle-slip estimates
				// MLAMBDA  
			ARMLambda mlambda;	
	
				// Resolve
			mlambda.resolve(csVec, csCov);
	
				// Fixed Solution 
			Vector<double> fixedCSVec( mlambda.getFixedAmbVec() );
			ratio = mlambda.getRatio();
	
				// Compute IB Success Rate
			SuccessRate sr( csCov );
			SR = sr.getSuccessRate();
			
	

			std::cout << "SR: " << SR  << " ratio: " << ratio <<  std::endl;
			if( SR > SuccessRateThreshold )
			{
				size_t s( csVec.size() );
				for( size_t i=0; i<s; i++ )
				{
					SatID sat( ambColSat[i].first );
					
						// Phase Obs TypeID e.g. prefitL1
					TypeID phaseType( ambColSat[i].second );

					double fixedVal( fixedCSVec(i) );

						// Convert phase TypeID to RinexObsID to get the band info
					RinexObsID roi( phaseType.ConvertToRinexObsID(sat.system) );

					int band( GetCarrierBand(roi) );
					
						// Insert fixed cycle slip 
					satFixedCS[sat][phaseType] = fixedVal;
					
//					if( fixedVal != 0 )
//					{
//						 gData(sat)[TypeID::CSL1] = 1.0;
//						 gData(sat)[TypeID::CSL2] = 1.0;
//					}
//
//					if( fixedVal == 0 )
//					{
//						 gData(sat)[TypeID::CSL1] = 0.0;
//						 gData(sat)[TypeID::CSL2] = 0.0;
//					}
//						// Make cycle slip correction
//					TypeID resultType( ConvertToTypeID(roi, sat) );
//					gData(sat)(resultType) += getWavelength(sat, band)*fixedVal;
					if( phaseType == TypeID::prefitL1 ) 
					{
//						gData(sat)[TypeID::CSL1] = 0.0;
						gData(sat)(TypeID::L1) -= getWavelength(sat, band)*fixedVal;
					} // End of 'if( phaseType == TypeID::prefitL1'
					
					if( phaseType == TypeID::prefitL2 )
					{
//						gData(sat)[TypeID::CSL2] = 0.0;
						gData(sat)(TypeID::L2) -= getWavelength(sat, band)*fixedVal;
					} // End of 'if( phaseType == TypeID::prefitL1'
				} // End of ' for( size_t i=0; i<s; i++ ) '
			}
//			else {
//				size_t s( csVec.size() );
//				for( size_t i=0; i<s; i++ )
//				{
//					SatID sat( ambColSat[i].first );
//								
//						// No CS for this sat
//					gData(sat)[TypeID::CSL1] = 0.0;
//					gData(sat)[TypeID::CSL2] = 0.0;
//				} 
//			} // End of 'if( SR > 0.9 )'
		}
		catch( Exception& e )
		{
			GPSTK_THROW( e );
		}

	} // End of ' void CycleSlipEstimator2::cycleSlipResolution( ... '

	void CycleSlipEstimator2::splitObsTypes( const SatID::SatelliteSystem& sys, 
														  const TypeIDSet& types )
	{
			// Loop through the 'types'
		for( TypeIDSet::const_iterator itType = types.begin(); 
			  itType != types.end(); 
			  ++itType )
		{
				// Convert to RinexObsID 
			TypeID type(*itType);
			RinexObsID roi( type.ConvertToRinexObsID( sys ) );

				// Classification or split the type
			if( roi.type == ObsID::otRange )
			{
				sysCodeObsTypes[sys].insert(type);
			}
			else if( roi.type == ObsID::otPhase )
			{
				sysPhaseObsTypes[sys].insert(type);
			} // End of 'if( roi.type == ObsID::otRange )'
			else{
				Exception e(getClassName() + "unrecognised obs type!");
				GPSTK_THROW(e);
			} // End of 'if( roi.type == ObsID::otRange )'

		} // End of 'for( TypeIDSet::const_iterator'


	} // End of 'void CycleSlipEstimator2::splitObsTypes( '


			/** Cycle slip detection: satellite by satellite and integrated detection
			 *
			 *	@param	I		satTimeDiffData 
			 * @param   I/O	stvm					GDS 
			 *
			 */
		void CycleSlipEstimator2::cycleSlipDetection( 
											satTypeValueMap& satTimeDiffData, 
											satTypeValueMap& stvm )
		{
//			SatIDSet badSats;
//
//			bool codeOnly(false);
//			bool phaseOnly(false);
//			badSats = modelTimeDifferencedData( codeOnly, phaseOnly, 
//															satTimeDiffData, stvm );
//
//
//			cycleSlipResolution( csVec, csCov, stvm );

			//exit(-1);
				// Remove 'tempBadSats' from 'satTimeDiffData'
//			satTimeDiffData.removeSatID( badSats );

				// Sat by sat detection 
				// key: the variance factor
			//satBySatDetection( satTimeDiffData, stvm );

				// Integrated detection 
			integratedDetection( satTimeDiffData, stvm );

			
		} // End of 'void CycleSlipEstimator2::cycleSlipDetection( ... '


		void CycleSlipEstimator2::satBySatDetection( 
											satTypeValueMap& satTimeDiffData,
											satTypeValueMap& stvm )
		{
				// Extract phase data
			//satTypeValueMap satPhaseData( satTimeDiffData.extractTypeID(sysPhaseObsTypes) );


				// Sat num 
			size_t satsNum( satTimeDiffData.numSats() );

			// Debug code vvv
//			for(int i=0; i<satsNum; i++ )
//			{
//				SatID sat(ionColSat[i]);
//				std::cout << sat << " " << i << std::endl; 
//			}
			// Debug code ^^^ 


				// Loop through 'satPhaseData'
			for( satTypeValueMap::const_iterator itData = satTimeDiffData.begin(); 
				  itData != satTimeDiffData.end(); 
				  ++itData )
			{
				SatID sat( itData -> first );
				typeValueMap tvm( itData -> second );
				std::cout << sat << std::endl;

					// Phase types of this sat 
				TypeIDSet phaseTypes( sysPhaseObsTypes(sat.system) );
				
				size_t numPhaseTypes(phaseTypes.size());

					// sat index in 'x' 
					// Get the index of the given sat and band in 
					// ionColSat
				size_t index(0);
				for(; index<satsNum; index++)
				{
					SatID satIonCol( ionColSat[index] );
						
						// If found
					if( satIonCol == sat ) break; 

				} // End of 'for( int i=0; i<rows; i++)'

					// The sat index in 'x'
				size_t in( numCoorVar + 1 + index );


				Vector<double> ybar(numPhaseTypes, 0);
				Vector<double> phaseVec(numPhaseTypes, 0);
				Matrix<double> Qybar(numPhaseTypes, numPhaseTypes, 0.0);
				Matrix<double> Qphase(numPhaseTypes, numPhaseTypes, 0.0);
				Matrix<double> hMatrix(numPhaseTypes, numPhaseTypes, 0.0);

					// 'A' matrix of this sat 
				Matrix<double> A( numPhaseTypes, numCoorVar+1+1, 0.0 );
				Vector<double> x( numCoorVar+1+1, 0.0 );
				Matrix<double> Qx( numCoorVar+1+1, numCoorVar+1+1, 0.0 );

					// Row index
				int i(0);	
					// index recorder of phase types in 'yhat'
				std::vector<int> indexRecorder;

				for( TypeIDSet::const_iterator itTypes = phaseTypes.begin(); 
					  itTypes != phaseTypes.end(); 
					  ++itTypes )
				{
					TypeID pType( *itTypes );
					std::cout << "Phase Type: " << pType << std::endl;

					try
					{
							// Phase observable of 'sat'
						double phase( tvm(pType) );

							// Get the current band 
						RinexObsID roi( pType.ConvertToRinexObsID(sat.system) );
						int band( GetCarrierBand( roi ) );
						double miu( getMiu( sat.system, band ) );

							// ***Fill matrix A 
						if( !staticReceiver )
						{
								// Coefficients for the coordinate displacement deltadx/y/z
								// but here is an approximation:
							A(i, 0) = stvm(sat)(TypeID::dx); 
							A(i, 1) = stvm(sat)(TypeID::dy); 
							A(i, 2) = stvm(sat)(TypeID::dz); 
						} 

							// Coefficients for the variation of receiver clock 
						A( i, numCoorVar ) = 1.0;

							// *** iono coefficient
						A( i, numCoorVar + 1 ) = -1 * miu;

							// hMatrix, only ambiguity term
						hMatrix( i, i ) = getWavelength( sat, band );  

					
							// Assignment 
						std::cout << "phase: " << phase << std::endl;
						phaseVec(i) = phase;

							// zenith weight
						double unitWeightStd( obsStd.getGNSSObsSTD( SatID::systemGPS, 
													 TypeID::prefitPC ) );
//						double unitWeightStd( sigma0hat ); 
						double phaseStd( obsStd.getGNSSObsSTD(sat.system, pType) );
						double q( phaseStd/unitWeightStd );
						q *= q;
						
						Qphase( i, i ) = q/tvm(TypeID::weight);

							// Record the index
						indexRecorder.push_back(index);
					}
					catch( ... )
					{
						Exception e(getClassName() + ": error in satBySatDetection()");
					} 
					
						// Increment of i
					i++;

				} // End of ' for( TypeID::const_iterator itTypes =  ... '

				//std::cout << "Qphase" << std::endl;
				//std::cout << Qphase << std::endl;
//
				std::cout << "martix A" << std::endl;
				std::cout << A << std::endl;


					// Get Qx and x 
				for( i=0; i<numCoorVar+1; i++ )
				{
					x(i) = solution(i);

						// First part: coor and clock
					for( int j=0; j<numCoorVar+1; j++)
					{
						Qx( i, j ) = covMatrix(i, j); 
					} // End of 'for( int j=0; j<numCoorVar+1+1; j++)'

						// Second part iono
					Qx( i, numCoorVar+1 ) = Qx( numCoorVar+1, i ) = covMatrix( i, in );

				}  // End of ' for( i=0; i<numCoorVar+1+1; ++i) '

				Qx( numCoorVar+1, numCoorVar+1 ) = covMatrix(in, in);
				x( numCoorVar+1 ) = solution(in);


				std::cout << "in" << std::endl;
				std::cout << in << std::endl;

				std::cout << "covMatrix" << std::endl;
				std::cout << covMatrix << std::endl;

				std::cout << "martix Qx" << std::endl;
				std::cout << Qx << std::endl;


				ybar = phaseVec - A*x; 
				Qybar = Qphase + A*Qx*transpose(A); 

				 
//					// Now get Qyhat of this sat
//				i = 0;
//				for( std::vector<int>::iterator itv = indexRecorder.begin(); 
//					  itv != indexRecorder.end(); 
//					  ++itv )
//				{
//					//std::cout << "index: " << *itv << std::endl;
//					int indexRow( *itv );
//
//					int j(0);
//					for( std::vector<int>::iterator itv2 = indexRecorder.begin(); 
//						  itv2 != indexRecorder.end(); 
//					     ++itv2 )
//					{
//						int indexCol( *itv2 );
//
//						std::cout << "(" << i << "," << j << ") Qphase: " << Qphase(i,j) << " Qyhat: " << Qyhat(indexRow, indexCol)  << std::endl;
//						Qybar( i, j ) = Qphase( i, j ) + Qyhat( indexRow, indexCol );
//						//Qybar( i, j ) =  Qyhat( indexRow, indexCol );
//							
//							// increment of j (col)
//						j++;
//					}  // End of 'for( std::vector<int>::iterator itv ... '
//					
//						// Increment of i (row)
//					i++;
//				} // End of 'for( std::vector<int>::iterator itv'

			//	std::cout << "Qphase" << std::endl;
			//	std::cout << Qphase << std::endl;

					// Weight matrix
				Matrix<double> rMatrix( inverse(Qybar) );

				//std::cout << "ybar" << std::endl;
				//std::cout << ybar << std::endl;

					// Estimate float CS of this sat
				SolverWMS solver;
				solver.Compute(ybar, hMatrix, rMatrix);
				
					// Store the solution
				Matrix<double> csVec( numPhaseTypes, 1, 0.0 );
				for( i=0; i<numPhaseTypes; i++ )
				{
					csVec( i, 0 ) = solver.solution(i); 
				}  // End of 'for( i=0; i<numPhaseTypes; i++ )'
				Matrix<double> csCov( solver.covMatrix );


				std::cout << "solution: " << std::endl;
				std::cout << solver.solution << std::endl;
//				std::cout << "covMatrix " << std::endl;
//				std::cout << solver.covMatrix << std::endl;

//					// *** look at postfit residuals first
//
//					// Compute Qvv fisrt 
//				Matrix<double> Q(Qybar);
//				Matrix<double> Qvv( Q - hMatrix * csCov * transpose(hMatrix) );
//				Matrix<double> residuals(numPhaseTypes, 1, 0.0);
//				residuals = hMatrix * solver.solution - ybar;
//
//				for( i=0; i<numPhaseTypes; i++ )
//				{
//					double post( residuals( i, 0 ) );
//					std::cout << "res: " << residuals <<  
//									"Qvv: " << Qvv(i,i) << std::endl;
//
//				}  //

					// ***Chi-square test
				Matrix<double> P( inverse( std::pow(sigma0hat, 2) * csCov ) );
				double statistic( (transpose(csVec)*P*csVec)(0,0) );	

				statistic = statistic / numPhaseTypes;

				std::cout << "statistics: " << statistic << std::endl;

					// Degree of freedom
				int degFreedom( numPhaseTypes );

				double right( chisqrRight[ degFreedom - 1 ]);
				double left( chisqrLeft[ degFreedom - 1 ] );

				std::cout << "r: " << right << " " << "l: " << left << std::endl;

//				if( statistic > right || statistic < left )
				right = F2[numPhaseTypes-1+(dfCode-1)*3];
				std::cout << "right: " << right << std::endl;
				
				if( statistic > right || statistic < 1.0e-4 )
				{
						// CS sat
					std::cout << "CS sat: " << sat << std::endl;
					stvm(sat)[TypeID::CSL1] = 1.0;
					stvm(sat)[TypeID::CSL2] = 1.0;
						
				}
				else
				{
					stvm(sat)[TypeID::CSL1] = 0.0;
					stvm(sat)[TypeID::CSL2] = 0.0;
				}

//					// Chi2Distribution object
//				Chi2Distribution chiObj;
//
////				double upperTailPro( chiObj.Q( statistic, degFreedom) );
//				std::cout << "upperTailPro: " << upperTailPro << std::endl;
//
////				if( upperTailPro < 0.025 || upperTailPro > 0.975 )
////				{
////				}  // End of 'if( upperTailPro < 0.025 || upperTailPro > 0.975 )'


			} // End of 'for( satTypeValueMap::const_iterator itData ... '

		} // End of 'void CycleSlipEstimator2::satBySatDetection( ... '


			/** Integrated detection 
			 * 
			 * @param satTimeDiffData
			 * @param gData
			 *
			 */ 
		void CycleSlipEstimator2::integratedDetection( 
													satTypeValueMap& satTimeDiffData, 
													satTypeValueMap& gData )
		{

			bool codeOnly(false);
			bool phaseOnly(false);
			modelTimeDifferencedData( codeOnly, phaseOnly, satTimeDiffData, gData, false );
			exit(-1);


		} // End of 'virtual void CycleSlipEstimator2::integratedDetection( ... '






			/** Hypothesis of Normalised residual
			 *
			 * @param  res			residual
			 * @param  q			covariance 
			 * @param  sigma		sigma0hat
			 *
			 */
		bool CycleSlipEstimator2::normalResidualTest( double& res, double& q, double sigma )
		{
			bool result(true);

			double normRes( res/(sigma * std::sqrt(q)) );

				// Threshold
			GaussianDistribution normal;
			double upperTailPro( normal.Q( std::abs( normRes) ) );

			if( upperTailPro < 0.0012 ) result = false;

			return result;

		} // End of 'bool CycleSlipEstimator2::normalResidualTest( ... '




}   // End of namespace
