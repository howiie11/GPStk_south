#pragma ident "$Id$"

/**
 * @file CycleSlipEstimator.cpp
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


#include "CycleSlipEstimator.hpp"
namespace gpstk
{
		// Return a string identifying this object
	std::string CycleSlipEstimator::getClassName() const
	{ return "CycleSlipEstimator"; }




      /* Returns a gnnsRinex object, adding the new data generated when
       * calling this object.
       *
       * @param gData    Data object holding the data.
       */
   gnssRinex& CycleSlipEstimator::Process(gnssRinex& gData)
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

   }  // End of method 'CycleSlipEstimator::Process()'



      /** Returns a satTypeValueMap object, adding the new data generated
       *  when calling this object.
       *
       * @param epoch     Time of observations.
       * @param gData     Data object holding the data.
       * @param epochflag Epoch flag.
       */
	satTypeValueMap& CycleSlipEstimator::Process( const CommonTime& epoch,
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

				//  debug code vvv
				std::cout << sat <<  std::endl;
				//  debug code ^^^ 

 					// Get filter data of this sat 
				bool timeInteruption(false);
 				timeInteruption = getSatFilterData( epoch, sat, tvm, 
																epochflag, satTimeDiffData ); 

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
//				Vector<double> cycleSlipVec( 2*numOfSats, 0.0 );
//				Matrix<double> cycleSlipCov( 2*numOfSats, 2*numOfSats, 0.0 );

				
				std::cout << "model time-diff data" << std::endl;

				modelTimeDifferencedData( satTimeDiffData, gData, cycleSlipVec, cycleSlipCov );

				std::cout << "fix cycle slips" << std::endl;
					// Fix the float cycle-slip estimates
					// MLAMBDA  
				ARMLambda mlambda;	

					// Resolve
				mlambda.resolve(cycleSlipVec, cycleSlipCov);

					// Compute IB Success Rate
				SuccessRate sr( cycleSlipCov );
				SR = sr.getSuccessRate();

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
 
 
 
 	}   // End of 'satTypeValueMap& CycleSlipEstimator::Process( ... '



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
	bool CycleSlipEstimator::getSatFilterData( const CommonTime& epoch, 
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
		double wNow( tvMap(TypeID::weight) );
		double wPrevious( satFormerData(sat)(TypeID::weight) );
		satTimeDiffData[sat][TypeID::weight] = wNow*wNow*wPrevious*wPrevious/( wNow*wNow + wPrevious*wPrevious); 

//			// Loop through the liTypes to compute time-differenced values
//		if( useTimeDifferencedLI )
//		{
//			for( TypeIDSet::const_iterator it = liTypes.begin(); 
//				  it != liTypes.end(); 
//				  ++it )
//			{
//				TypeID type( *it ), outType( *it );
//
//				if( type == TypeID::LI )
//				{
//					outType = TypeID::deltaLI;
//				}
//
//					// Time differenced value of this type 
//				double deltaLI(0.0);
//				deltaLI = tvMap(type) - satFormerData(sat)(type);
//
//				satLITimeDiffData[sat][outType] = deltaLI;
//
//
//					// Record deltaLI in etvb
//				etvb[outType] = deltaLI;
//
//					// Double time-differenced LI 
//				double deltaDeltaLI(0.0);
//				epochTypeValueBody::const_iterator iter = 
//												satFormerData(sat).find( TypeID::deltaLI );	
//				if( iter != satFormerData(sat).end() )
//				{
//					deltaDeltaLI = deltaLI - satFormerData(sat)(outType);
//					satLITimeDiffData[sat][ TypeID::deltaDeltaLI ] = deltaDeltaLI;
//				}
//				
//			}   // End of ' for( TypeIDSet::const_iterator it =  ... '
//
//		}   // End of ' if( useExternalIonoDelayInfo ) '


			// Update satFormerData data
		satFormerData[sat] = etvb;
		return reportTimeInteruption;


	}   // End of 'void CycleSlipEstimator::getFilterData( const SatID& sat, ... '


		/** Model time-differenced code data
		 *
		 * ...
		 *
		 */
	SatIDSet CycleSlipEstimator::modelTimeDifferencedData( satTypeValueMap& satTimeDiffData,
																			 satTypeValueMap& stvm,
																			 Vector<double>& csVec, 
																			 Matrix<double>& csCov )
	{

		SatIDSet badSatSet;

//			// Get band list first
//		getBandList();

			// Num of available sats
		size_t numOfSats( satTimeDiffData.numSats() );

		// debug code vvv
		std::cout << "numOfSats: " << numOfSats << std::endl;
		// debug code ^^^ 

			// Total num of elements in satTimeDiffData
		satTypeValueMap obsData( satTimeDiffData.extractTypeID( sysObsTypes ) );
		size_t numOfElements( obsData.numElements() );

			// Valid sat set
		validSatSet.clear();	// clear the set of last epoch 	
		validSatSet = satTimeDiffData.getSatID();

		// debug code vvv
		std::cout << "numOfElements: " << numOfElements << std::endl;
		// debug code ^^^ 

			// Num of measurements
			// phase, code and pseudo-ionospheric observables 
		size_t numMeas( numOfElements + numOfSats );

			// Num of unknowns, also the same order in the time-diff equation 
			// 3				receiver coordinate displacements(if it is moving)
			// 1				receiver clock offset variation 
			// numOfSats   ionospheric delay variation on the first frequency
			//					e.g. L1 for GPS 
			// numOfSats*n cycle-slip parameters, n is the number of frequency     

		size_t numCoorVar( (staticReceiver?0:3) );

			// Compute the number of unknowns 
		size_t numUnknowns( numCoorVar + 1 );

			// Record the number of Cycle-Slip parameters regarding to all the 
			// sat system
		size_t numAmbUnknowns( 0 );
			
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

				// Add ionospheric parameters, which are equal to the num of sats  
			numUnknowns += numSats;

				// Num of Phase frequency
			size_t numPhaseFreq( sysObsTypeBandSet( sys, ObsID::otPhase).size() );

				// Add ambiguity parameters, numSats*(num of Phase frequency )  
			numUnknowns += numSats*numPhaseFreq; 
			numAmbUnknowns += numSats*numPhaseFreq;
				
		} // End of ' for( SysTypeIDSetMap::const_iterator itS ... '

		// debug code vvv
		std::cout << "numOfSats: " << numOfSats << " numUnknowns: " 
						<< numUnknowns << " numAmbUnknowns: " << numAmbUnknowns << std::endl;
		// debug code ^^^ 

			// Resize csVec and csCov
		csVec.resize( numAmbUnknowns, 0.0 );
		csCov.resize( numAmbUnknowns, numAmbUnknowns, 0.0 );

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
		GNSSObsSTDTables obsStd;
		double unitWeightStd( obsStd.getGNSSObsSTD( SatID::systemGPS, 
																  TypeID::prefitPC ) );
//		unitWeightStd = 0.2;

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
			std::cout << sat << std::endl;
			typeValueMap tvm( itStvm->second );

				// Sat system
			SatID::SatelliteSystem sys( sat.system );
			TypeIDSet obsTypes( sysObsTypes(sys) );

			for( TypeIDSet::const_iterator itObsTypes = obsTypes.begin();
				  itObsTypes != obsTypes.end(); 
				  ++itObsTypes )
			{
					// Code TypeID 
				TypeID type( *itObsTypes );

				// debug code vvv
				std::cout << "type= " << type << std::endl;
				// debug code ^^^ 
				
				y(i) = tvm( type );
	
					// Zenith weight
				double observableStd( obsStd.getGNSSObsSTD(sys, type ) );
				double weight( unitWeightStd/observableStd );
				weight *= weight;
	
				rMatrix( i, i ) = tvm(TypeID::weight) * weight; 


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
				std::cout << sat << " " << type << " band:" << band << "--->";
				int bandIndex( getBandIndex(sys, roi.type, band) );

				std::cout <<"bandIndex: " << bandIndex << std::endl;
					
					// 'getMiu' function is defined in file 'GNSSconstants' class
				double miu( getMiu( sys, band ) );

					// Fill the efficient
				if( roi.type == ObsID::otPhase )
				{
						// For the variation of iono delay
					hMatrix( i, numCoorVar+1+j ) = -1 * miu;  

						// For the cycle slips
					//hMatrix( i, numCoorVar+1+(bandIndex+1)*numOfSats+j ) = 
					hMatrix( i, numCoorVar+1+numOfSats+acPhaseFreqNum+bandIndex ) = 
						getWavelength( sat, band );
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

					//	Increment for row 
				i++;
	
			}   // End of 'for( TypeIDSet::const_iterator itObsTypes ... '


				// Add virtual measurement of ionospheric delay variation
			y(i) = 0;
			double ionoWeight( unitWeightStd/(0.05) );
			rMatrix( i, i ) = ionoWeight*ionoWeight;

				// Only one coefficients for this virtual obs  
			hMatrix( i, numCoorVar+1+j ) = 1.0;
				
				// Preparation for next sat
			i++;   // Donot forget!!! 
			j++;

				// Increment of acPhaseFreqNum
			acPhaseFreqNum += sysObsTypeBandSet( sys, ObsID::otPhase ).size();

		}   // End of ' for( satTypeValueMap::const_iterator itStvm =  ... '


		// debug code vvv
		//std::cout << "hMatrix: " << std::endl;
		//std::cout << hMatrix << std::endl;
		//exit(-1);
		// debug code ^^^

			// Now employ a solver 
		SolverWMS solver;
		solver.Compute( y, hMatrix, rMatrix );

			// Store cycle slip estimates 
		for( i=0; i<numAmbUnknowns; ++i)
		{
			csVec(i) = solver.solution( numCoorVar + 1 + numOfSats + i ); 
			for( j=0; j<numAmbUnknowns; ++j )
			{
				csCov(i, j) = solver.covMatrix( numCoorVar + 1 + numOfSats + i, 
														  numCoorVar + 1 + numOfSats + j );  
			} // End of ' for( i=0; i<numAmbUnknowns; ++i) '

		} // End of 'for( i=0; i<2*numOfSats; ++i)'

			// Postfit residuals
		postfitResiduals.resize( rows, 0.0 );
		postfitResiduals = hMatrix * solver.solution - y; 

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
//			// post unit weight std
//		Matrix<double> vTpv( transpose(postfitResiduals) * 
//								  rMatrix * postfitResiduals );
//
//		double sigma0hat = std::sqrt( vTpv(0,0) / ( rows - numUnknowns ) );
//
//
//		// debug code vvv
//		std::cout << "sigma0: " << unitWeightStd << std::endl;
//		std::cout << "sigma0hat: " << sigma0hat << std::endl;
//		// debug code ^^^
//
//
//			// Compute Qvv
//		Matrix<double> Q( inverse(rMatrix) );
//		Matrix<double> Nbb( transpose( hMatrix ) * rMatrix * hMatrix );
//		Matrix<double> invNbb( inverse(Nbb) );
//		Matrix<double> Qvv( Q - hMatrix * invNbb * transpose( hMatrix ) );
		// debug code vvv
//		std::cout << "Qvv: " << std::endl;
//		std::cout << Qvv << std::endl;
//		exit(-1);
		// debug code ^^^

	
//			// Compute normalized residuals
//		Vector<double> normRes( numOfElements, 0.0 );
//		size_t k(0);
//		size_t numCodeTypes( obsTypes.size() );
//		for( size_t i=0; i<rows; i++ )
//		{
//			if( (i%(numCodeTypes+1)) == numCodeTypes ) continue;
//			normRes(k) = postfitResiduals( i, 0 ) / Qvv( i, i );
//			k++;
//		}

//		std::cout << "normRes: " << std::endl;
//		std::cout << normRes << std::endl;

//		// debug code vvv
//		Stats<double> statistic;
//		statistic.Add(normRes);
//		std::cout << "normRes ave: " << statistic.Average() << " var: " 
//						<< statistic.StdDev() <<  std::endl;
//		// debug code ^^^

		return badSatSet;

	}   // End of ' void CycleSlipEstimator::modelTimeDifferencedCode( ...'


		/* Return post residual regarding to a specific sat and a specific 
		 * type. These types should be corresponding to the data member
		 * obsTypes.
		 *
		 * @param sat
		 * @param type
		 * 
		 */
	double CycleSlipEstimator::getPostfitResidual( SatID sat, TypeID type )
	{
			// postfit residual value
		double postRes(0.0);

			// Convert postfitType(usr) to prefitType(engineer)
		TypeID prefitType = type.ConvertToPrefitTypeID();

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

		postRes = postfitResiduals(row);

		return postRes;
	}  // End of 'double getPostfitResidual( SatID sat, TypeID type )'

 

		/// Return a  given a band in obsTypes
	void CycleSlipEstimator::getBandList( SatID::SatelliteSystem sys )
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
					Exception e("CycleSlipEstimator: unrecognised band");
					GPSTK_THROW( e );
				}
				
					// Insert this band into the set 
			 	sysObsTypeBandSet[ sys ][ roi.type ].insert( currentBand );
			
			} // End of 'for( TypeIDSet::const_iterator itType = types.begin();...' 

		}
		else
		{
			Exception e("CycleSlipEstimator: no obstypes for some sys");
			GPSTK_THROW(e);
		}

	}   // End of ' void getBandList( int band ) '



	int CycleSlipEstimator::getBandIndex( SatID::SatelliteSystem sys, 
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

	std::set<int>& CycleSlipEstimator::SysObsTypeSet::operator()( const SatID::SatelliteSystem sys, 
															const ObsID::ObservationType ot )
		throw( ValueNotFound )
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
				ValueNotFound e("CycleSlipEstimator: obsType doesn't exist");
				GPSTK_THROW(e);
			}

		}
		else
		{
			ValueNotFound e("CycleSlipEstimator: sat system doesn't exist");
			GPSTK_THROW(e);
		}  // End of ' if( itSTB != sysObsTypeBandSet.end() ) '

	} // End of ' set<int>& SysObsTypeSet::operator() ... '

}   // End of namespace
