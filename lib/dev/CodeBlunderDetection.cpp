#pragma ident "$Id$"

/**
 * @file CodeBlunderDetection.cpp
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

#include "CodeBlunderDetection.hpp"
#include "GNSSObsSTDTables.hpp"
#include "GNSSconstants.hpp"
#include "SolverWMS.hpp"
#include "Stats.hpp"

namespace gpstk
{
		// Return a string identifying this object
	std::string CodeBlunderDetection::getClassName() const
	{ return "CodeBlunderDetection"; }




      /* Returns a gnnsRinex object, adding the new data generated when
       * calling this object.
       *
       * @param gData    Data object holding the data.
       */
   gnssRinex& CodeBlunderDetection::Process(gnssRinex& gData)
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
	satTypeValueMap& CodeBlunderDetection::Process( const CommonTime& epoch,
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

				// clear sat time-differenced LI data
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

					// Test code vvv
//				tvm[ TypeID::weight ] = 1.0;
					// Test code ^^^ 

					// Check required code data 
				double testValue(0.0);
				try
				{

						// Loop through the codeTypes to get their values
					for( TypeIDSet::const_iterator it = codeTypes.begin();
						  it != codeTypes.end();
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
				bool isTimeContinuous(true);
 				isTimeContinuous = !getSatFilterData( epoch, sat, 
																  tvm,	epochflag ); 

 			}   // End of ' for( it = gData.begin();  ... '

				// Remove satellites with missing data
			gData.removeSatID(satRejectedSet);


				// Now, let's model time-differenced code observations 
			size_t numOfSats( satCodeTimeDiffData.numSats() );
			if( numOfSats >= 4 )
			{
//				timeDiffCodeModel();
				std::cout << "Do estimation!!!" << std::endl;

					// Returned value
				SatIDSet blunderCodeSatSet; 
				blunderCodeSatSet = modelTimeDifferencedCode( gData );
			}   
			else
			{
				std::cout << "not enough sats" << std::endl;
			}


				// Clear satCodeTimeDiffData, which is generated at every epoch 
			satCodeTimeDiffData.clear();


		
			return gData;
			

//			// debug code vvv
//			
//			SatID sat1(1, SatID::systemGPS );
//			CommonTime time( satFormerData( sat1 ).epoch );
//			std::cout << "Test C1 value: " << satFormerData( sat1 )( TypeID::C1) << std::endl;
//		
//			// debug code ^^^ 
 
 		}
 
 		catch( Exception& u )
 		{
 				// Thrown an exception if something unexpected happens
 			ProcessingException e( getClassName() + ":" 
 											+ u.what() );
 
 			GPSTK_THROW(e);
 
 		}   // End of 'try-catch' block
 
 
 
 	}   // End of 'satTypeValueMap& CodeBlunderDetection::Process( ... '



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
	bool CodeBlunderDetection::getSatFilterData( const CommonTime& epoch, 
																  const SatID& sat, 
																  typeValueMap& tvMap,
																  const short& epochFlag )
	{

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
			// Loop through the codeTypes to compute time-differenced values
		for( TypeIDSet::const_iterator it = codeTypes.begin();
			  it != codeTypes.end();
			  ++it )
		{
			TypeID type( *it );

			
				// Time differenced value of this type 
			satCodeTimeDiffData[sat][type] = tvMap(type) - 
														satFormerData(sat)(type);

		
		}   // End of ' for( TypeIDSet::const_iterator it = obsTypes.begin(); ... '

			// Insert weight info into satCodeTimeDiffData with
			// the consideration of elevation 
		double wNow( tvMap(TypeID::weight) );
		double wPrevious( satFormerData(sat)(TypeID::weight) );
		satCodeTimeDiffData[sat][TypeID::weight] = 
			wNow*wPrevious/( wNow + wPrevious); 

		
			// Loop through the liTypes to compute time-differenced values
		if( useTimeDifferencedLI )
		{
			for( TypeIDSet::const_iterator it = liTypes.begin(); 
				  it != liTypes.end(); 
				  ++it )
			{
				TypeID type( *it ), outType( *it );

				if( type == TypeID::LI )
				{
					outType = TypeID::deltaLI;
				}

					// Time differenced value of this type 
				double deltaLI(0.0);
				deltaLI = tvMap(type) - satFormerData(sat)(type);

				satLITimeDiffData[sat][outType] = deltaLI;


					// Record deltaLI in etvb
				etvb[outType] = deltaLI;

					// Double time-differenced LI 
				double deltaDeltaLI(0.0);
				epochTypeValueBody::const_iterator iter = 
												satFormerData(sat).find( TypeID::deltaLI );	
				if( iter != satFormerData(sat).end() )
				{
					deltaDeltaLI = deltaLI - satFormerData(sat)(outType);
					satLITimeDiffData[sat][ TypeID::deltaDeltaLI ] = deltaDeltaLI;
				}
				
			}   // End of ' for( TypeIDSet::const_iterator it =  ... '

		}   // End of ' if( useExternalIonoDelayInfo ) '


			// Update satFormerData data
		satFormerData[sat] = etvb;

		return reportTimeInteruption;


	}   // End of 'void CycleSlipEstimator::getFilterData( const SatID& sat, ... '


		/** Model time-differenced code data
		 *
		 * ...
		 *
		 */
	SatIDSet CodeBlunderDetection::modelTimeDifferencedCode( 
			satTypeValueMap& stvm )
	{

		SatIDSet badSatSet;

			// Num of available sats
		size_t numOfSats( satCodeTimeDiffData.numSats() );

		// debug code vvv
		std::cout << "numOfSats: " << numOfSats << std::endl;
		// debug code ^^^ 

			// Total num of elements in satCodeTimeDiffData
		satTypeValueMap obsData( satCodeTimeDiffData.extractTypeID( codeTypes ) );
		size_t numOfElements( obsData.numElements() );
		// debug code vvv
		std::cout << "numOfElements: " << numOfElements << std::endl;
		// debug code ^^^ 

			// Num of measurements
		size_t numMeas( numOfElements + numOfSats );

			// Num of unknowns 
			// 3				receiver coordinate displacement(if it is moving)
			// 1				receiver clock offset variation 
			// numOfSats   ionospheric delay variation on the first frequency
			//					eg. usually L1 for GPS 
		size_t numUnknowns( numOfSats + 1 + (staticReceiver?0:3) ) ;

		size_t numCoorVar( (staticReceiver?0:3) );

		// debug code vvv
		//std::cout << "numOfSats: " << numOfSats << "numUnknowns: " 
		//				<< numUnknowns << std::endl;
		// debug code ^^^ 

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
		unitWeightStd = 0.2;

			//* Now fill in matrix
			//* y = [P1,s1 P2,s1 P3,s1 ... P1,sn P2,sn P3,sn]
			
			// Row index
		size_t i(0);

			// Column index  
		size_t j(0);
		
		for( satTypeValueMap::const_iterator itStvm = satCodeTimeDiffData.begin(); 
			  itStvm != satCodeTimeDiffData.end(); 
			  ++itStvm )
		{

				// SatID 
			SatID sat( itStvm->first );
			typeValueMap tvm( itStvm->second );


			for( TypeIDSet::const_iterator itCodeTypes = codeTypes.begin();
				  itCodeTypes != codeTypes.end(); 
				  ++itCodeTypes )
			{
					// Code TypeID 
				TypeID type( *itCodeTypes );

				// debug code vvv
				std::cout << "type= " << type << std::endl;
				// debug code ^^^ 
				
				y(i) = tvm( type );
	
					// Zenith weight
				double codeStd( obsStd.getGNSSObsSTD(SatID::systemGPS, type ) );
				double weight( unitWeightStd/codeStd );
				weight *= weight;
	
				rMatrix( i, i ) = tvm(TypeID::weight) * weight; 


					// ***Now fill design matrix
					

				if( !staticReceiver )
				{
						// Coefficients for the coordinate displacement deltadx/y/z
						// but here we use that of dx/y/z as an approximation
					hMatrix( i, 0 ) = stvm(sat)( TypeID::dx );
					hMatrix( i, 1 ) = stvm(sat)( TypeID::dy );
					hMatrix( i, 2 ) = stvm(sat)( TypeID::dz );
				}

					// Coefficients for the variation of receiver colock
				hMatrix( i, numCoorVar ) = 1.0;

					// Coefficients for the variation of iono delay
					// j indicates the sat index

					// Note the index for function getFreqBand(), which depends
					// on the string of type
					// TypeID::prefitP1 <---> prefitResidualCodeP1 
					// so the starting index is 19
				hMatrix( i, numCoorVar+1+j ) = 
					getMiu( sat.system, type.getFreqBand(19) );


					//	Increment for row 
				i++;


	
			}   // End of ' for( typeValueMap::const_iterator itTvm ... '

				// Add virtual measurement of ionospheric delay variation
			y(i) = 0;
			double ionoWeight( unitWeightStd/(0.10) );
			rMatrix( i, i ) = ionoWeight*ionoWeight;

				// Only one coefficients for this virtual obs  
			hMatrix( i, numCoorVar+1+j ) = 1.0;

				
				// Preparation for next sat
			i++;   // Donot forget!!! 
			j++;

		}   // End of ' for( satTypeValueMap::const_iterator itStvm =  ... '


		// debug code vvv
		std::cout << "hMatrix: " << std::endl;
		std::cout << hMatrix << std::endl;
		// debug code ^^^

			// Now employ a solver 
		SolverWMS solver;
		solver.Compute( y, hMatrix, rMatrix );

			// Postfit residuals
		Matrix<double> postfitResiduals( rows, 1, 0.0 );
		postfitResiduals = hMatrix * solver.solution - y; 


		// debug code vvv
		std::cout << "solution: " << std::endl;
		std::cout << solver.solution << std::endl;
		std::cout << "postfitRes: " << std::endl;
		std::cout << postfitResiduals << std::endl;
		// debug code ^^^
			
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
//
//	
//			// Compute normalized residuals
//		Vector<double> normRes( numOfElements, 0.0 );
//		size_t k(0);
//		size_t numCodeTypes( codeTypes.size() );
//		for( size_t i=0; i<rows; i++ )
//		{
//			if( (i%(numCodeTypes+1)) == numCodeTypes ) continue;
//			normRes(k) = postfitResiduals( i, 0 ) / Qvv( i, i );
//			k++;
//		}
//
//		// debug code vvv
//		Stats<double> statistic;
//		statistic.Add(normRes);
//		std::cout << "normRes ave: " << statistic.Average() << " var: " 
//						<< statistic.StdDev() <<  std::endl;
//		// debug code ^^^





		return badSatSet;

	}   // End of ' void CodeBlunderDetection::modelTimeDifferencedCode( ...'


}   // End of namespace
