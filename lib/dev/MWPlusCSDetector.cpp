#pragma ident "Id$"

/**
 * @file MWPlusCSDetector.cpp
 * See the detail explanation of this class in its header file.
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
//
// See modifications of this class in its header file. 
//
//============================================================================


#include "MWPlusCSDetector.hpp"


namespace gpstk
{

		// Returns a string identifying this object.
	std::string MWPlusCSDetector::getClassName() const
	{ return "MWPlusCSDetector"; }

		// static members of this class 
	const double MWPlusCSDetector::pCodeStd = 0.5; // 0.5m
	const double MWPlusCSDetector::carrierWaveStd = 0.005; // 0.005m

	const double MWPlusCSDetector::weightFactor = 10000.0; // 100^2

	const double MWPlusCSDetector::miu1 = 1;
	const double MWPlusCSDetector::miu2 = L1_FREQ_GPS*L1_FREQ_GPS/(L2_FREQ_GPS*L2_FREQ_GPS);
	const double MWPlusCSDetector::unitWeightVar = 1.0;

	const double MWPlusCSDetector::e = L1_FREQ_GPS/(L1_FREQ_GPS - L2_FREQ_GPS);
	const double MWPlusCSDetector::f = L2_FREQ_GPS/(L1_FREQ_GPS - L2_FREQ_GPS);

		// Default construct, setting default parameter. 
	MWPlusCSDetector::MWPlusCSDetector() : staticReceiver(true),firstTime(true),
							obsType1(TypeID::prefitP1),obsType2(TypeID::prefitP2),
							obsType3(TypeID::prefitL1),obsType4(TypeID::prefitLdelta),
							csType(TypeID::CSL1),ratioThres(3.0),eleThreshold(30.0),
							obsType5(TypeID::LI), minBuffer(5), maxBuffer(10)
	{
		Init();
	}  // End of constrctor function

		// Initializing method 
	void MWPlusCSDetector::Init( void )
	{
			// By Default
//		deltaPfP1Var = 2*pCodeStd*pCodeStd;
//		deltaPfP2Var = 2*pCodeStd*pCodeStd;
//		deltaPfL1Var = 2*carrierWaveStd*carrierWaveStd;
//
//		deltaPfLWVar = 2*( carrierWaveStd * carrierWaveStd * ( (77.0*77.0)+(60.0*60.0) )/(17*17) );

		if( staticReceiver )
		{
			for(int i=0; i<3; i++) deltaPosVar[i] = 1E-6;
		}
		else {
			
			for(int i=0; i<3; i++) deltaPosVar[i] = 10000; // (100 m)**2
			//for(int i=0; i<3; i++) deltaPosVar[i] = 0.000001; // (100 m)**2

		}  // End of ' if( staticReceiver ) '

	}  // End of ' MWPlusCSDetector::Init() '

      /* Returns a gnnsRinex object, adding the new data generated when
       * calling this object.
       *
       * @param gData    Data object holding the data.
       */
   gnssRinex& MWPlusCSDetector::Process(gnssRinex& gData)
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

   }  // End of method 'MWCSDetector::Process()'


      /* Returns a satTypeValueMap object, adding the new data generated
       * when calling this object.
       *
       * @param epoch     Time of observations.
       * @param gData     Data object holding the data.
       * @param epochflag Epoch flag.
       */
   satTypeValueMap& MWPlusCSDetector::Process( const CommonTime& epoch,
                                           satTypeValueMap& gData,
                                           const short& epochflag )
      throw(ProcessingException)
   {
		try
		{
			
//			std::cout << getClassName() << std::endl;

			// vvvvvvvvvvvvvvvvvvvvv debug code vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
			commonData.epoch = epoch;
			// ^^^^^^^^^^^^^^^^^^^^^ debug code ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			double pfP1(0.0);
			double pfP2(0.0);
			double pfL1(0.0);
			double pfLW(0.0);
			double CSFlag(0.0);

				// LI 
			double li(0.0);

				// Weight
			double w(0.0);

				// Elevation
			double ele(0.0);

			SatIDSet satRejectedSet;

				// Loop through all the satellites
			satTypeValueMap::iterator it;
			for( it = gData.begin(); it != gData.end(); ++it )
			{
				try
				{

						// Try to extract values
					pfP1 = (*it).second(obsType1);
					pfP2 = (*it).second(obsType2);
					pfL1 = (*it).second(obsType3);
					pfLW = (*it).second(obsType4);
					CSFlag = (*it).second(csType);

						// LI
					li = (*it).second(obsType5);

						// Weight of this sat
					w = (*it).second(TypeID::weight);

						// Elevation
					ele = (*it).second(TypeID::elevation);

				}
				catch(...)
				{
						// If some value is missing, then schedule this satellite
						// for removal
					satRejectedSet.insert( (*it).first );
					continue;

				}  // End of try catch block

					// We will reject sats with low elevation
				if( ele < eleThreshold )
				{
					//satRejectedSet.insert( (*it).first );
					continue;
				}  // End of ' if( ele < eleThreshold ) ... ' 




					// Get filter data of this sat at this epoch
				getFilterData( (*it).first, epochflag, pfP1, pfP2, 
																	pfL1, pfLW, CSFlag, w, li );

			}  // End of ' for( it = gData.begin; it != gData.end(); ... '


					// Now it's time to judge cycle slip through an equation sysytem
			size_t numOfSats( obsData.size() );

//			std::cout << "size of obsData: " << obsData.size() << std::endl;
			
			// vvvvvvvvvvvvvvvvvv debug code vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
				commonData.numSats = numOfSats;
			// ^^^^^^^^^^^^^^^^^^ debug code ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			if( numOfSats >= 1 )
			{

					// Here means we have enough satellites at present 
					// Detect cycle slip
				if ( IntegratedDetection( gData, numOfSats ) )
				{ 
//					std::cout << "Detect CS on L1 of some sats ! " << std::endl;
				}
				else
				{
//					std::cout << " Did not detect CS on L1 for all sats ! " << std::endl;
				}

			}  // End of ' if( numOfSats > 4 ) ' 
	
				// Remove satellites with missing data
			gData.removeSatID(satRejectedSet);

				// Clear obsData for this epoch
			obsData.clear();

			return gData;
		}
		catch(Exception& u)
		{
				// Throw an exception if something unexpected happens
			ProcessingException e( getClassName() + ":" + u.what() );

			GPSTK_THROW(e);
		}  // End of try-catch block

	}  // End of ' satTypeValueMap& MWCSDetector::Process ... '

      /** Method to get filter data of specifying sat 
       *
       * @param sat       SatID.
       * @param epochflag Epoch flag.
       * @param pfp1       
       * @param pfp2     
       * @param pfl1
		 * @param pflw
		 * @param csflag
		 * @param weight
       */
   void MWPlusCSDetector::getFilterData(const SatID& sat,
													 const short& epochflag,
													 const double& pfp1,
													 const double& pfp2,
													 const double& pfl1,
													 const double& pflw, 
													 const double& csflag,
													 const double& weight,
													 const double& li_ )
	{
			// Judge whether this is a new arc according to MWCSDetector
		if( csflag == 0.0 )
		{

				// Here we get a consecutive arc start
			satFilterData[sat].windowSize++;

				// Judge whether we have at least two consecutive epoch data
			if( satFilterData[sat].windowSize > 0 )
			{
					// Size of present deltaLI deque
				size_t s( satFilterData[sat].deltaLIDeque.size() );
				
				if( s >= minBuffer )
				{

						// The deltaLI observable
					Stats<double> statistic;
					ComputeStat( statistic, satFilterData[sat].deltaLIDeque );
				   obsData[sat].deltaLIAve = statistic.Average();
					obsData[sat].deltaLIVar = statistic.Variance();

						// Other observables
					obsData[sat].deltaPrefitP1 = pfp1 - 
														  satFilterData[sat].formerPrefitP1;
	
					obsData[sat].deltaPrefitP2 = pfp2 - 
														  satFilterData[sat].formerPrefitP2;
					
					obsData[sat].deltaPrefitL1 = pfl1 - 
														  satFilterData[sat].formerPrefitL1;
	
					obsData[sat].deltaPrefitLW = pflw - 
														  satFilterData[sat].formerPrefitLW;
	
					obsData[sat].weightOfDeltaPreP = 
						weight * satFilterData[sat].formerWeight / ( weight + 
						satFilterData[sat].formerWeight );



				}

					// Store present deltaLI 
				double delta_li( li_ - satFilterData[sat].formerLI );
				satFilterData[sat].deltaLIDeque.push_back( delta_li);
					
					// Check the size again
				s = satFilterData[sat].deltaLIDeque.size();
				if( s >= maxBuffer )
				{
					satFilterData[sat].deltaLIDeque.pop_front();
			   }

					
			}  // End of ' if( satFilterData[sat].windowSize > 1 ) ... '

		}
		else {
				
				// Here we are at a new arc for this sat
			satFilterData[sat].windowSize = 0;

				// Clear the deque for deltaLI 
			satFilterData[sat].deltaLIDeque.clear(); 


		}  // End of ' if( csflag == 0.0 ) '


			// No matter what the arc is, we need to store the present data 
		satFilterData[sat].formerPrefitP1 = pfp1;
		satFilterData[sat].formerPrefitP2 = pfp2;
		satFilterData[sat].formerPrefitL1 = pfl1;
		satFilterData[sat].formerPrefitLW = pflw;
		satFilterData[sat].formerWeight = weight;
		satFilterData[sat].formerLI = li_;


	}  // End of ' virtual double MWPlusCSDetector:: ... '


		/** Integrated detection method
		 * 
		 * Returns a satTypeValueMap
		 *
		 */
	bool MWPlusCSDetector::IntegratedDetection( satTypeValueMap& gData,
															  size_t& satsNum)
	{
		bool detectionResult(false);
			// Step 1: Get measurement vector, weight matrix and design matrix

			// For every sat, there are 4 types of obs, namely
			//		delataPrefitP1, delataPrefitP1,
			//		delataPrefitL1, delataPrefitLW
			//		3 more delta_dx delta_dy delta_dz
		//size_t numMeas( 4*satsNum + 3 );
		size_t numMeas( 5*satsNum + 3 );
		size_t rows( numMeas );


			// The colums should be equal to unknowns		#
			//		delta dx dy dz									3
			//		delta receiver clock offset				1
			//		delta iono delay on L1 of all sats		satsNum
			//		cycle slip on L1 frequency of all sats	satsNum
		size_t numUnknowns( 3 + 1 + satsNum + satsNum );
		size_t cols( numUnknowns );

			// Declare a Vector for obs
		Vector<double> y( rows, 0.0 );

			// Declare a weight matrix
		Matrix<double> wMatrix(rows, rows, 0.0);

			// Declare a design matrix
		Matrix<double> dMatrix(rows, cols, 0.0);

			// Virtual obs for delta dx, dy, dz
		y(0) = 0.0;
		y(1) = 0.0;
		y(2) = 0.0;

			// And related elements in design matrix
		dMatrix( 0, 0 ) = 1.0;
		dMatrix( 1, 1 ) = 1.0;
		dMatrix( 2, 2 ) = 1.0;

			// For static receiver, we use its sequential adjustment results
			// not simply the default value
//		if( staticReceiver )
//		{
//			if( firstTime )
//			{
//				// For the first epoch, we just use its default value of deltaPosVar
//				// Then turn off the flag
//				firstTime = false;
//			}
//			else{
//					// Record the previous results
//				deltaPosVar[0] = solver.covMatrix( 0, 0 );
//				deltaPosVar[1] = solver.covMatrix( 1, 1 );
//				deltaPosVar[2] = solver.covMatrix( 2, 2 );
//			}
//		}  // End of ' if( staticReceiver ) '
		
		// Fill the elements correspond to delta dx dy dz in wMatrix
		for(int i=0; i<3; i++)
		{
			wMatrix( i, i ) = unitWeightVar/deltaPosVar[i];
		} 

		double commonWeightOfP( unitWeightVar / ( pCodeStd * pCodeStd ) );
		size_t i(3);

		std::map<SatID,obs>::const_iterator ite;
		for( ite = obsData.begin();
			  ite != obsData.end();
			  ++ite )
		{
				// delataPrefitP1
			y(i) = obsData[(*ite).first].deltaPrefitP1;
			wMatrix( i, i ) = obsData[(*ite).first].weightOfDeltaPreP *
									commonWeightOfP;

				// delataPrefitP2
			y( i + 1 ) = obsData[(*ite).first].deltaPrefitP2;
			wMatrix( i+1, i+1 ) = obsData[(*ite).first].weightOfDeltaPreP * 
										 commonWeightOfP;

				// delataPrefitL1 
			y( i + 2 ) = obsData[(*ite).first].deltaPrefitL1;
			wMatrix( i+2, i+2 ) = obsData[(*ite).first].weightOfDeltaPreP *
										 commonWeightOfP * weightFactor;

				// delataPrefitLW
			y( i + 3 ) = obsData[(*ite).first].deltaPrefitLW;
			wMatrix( i+3, i+3 ) = obsData[(*ite).first].weightOfDeltaPreP *
										 commonWeightOfP * weightFactor / ( e*e + f*f );

			y( i + 4 ) = obsData[(*ite).first].deltaLIAve / ( miu2 - miu1 );
			double temp_coe( (miu2 - miu1)*(miu2 - miu1) );
			wMatrix( i+4, i+4 ) = unitWeightVar*temp_coe/(obsData[(*ite).first].deltaLIVar);

				 // dMatrix
//			for( size_t k=i; k<i+4; k++ )
			for( size_t k=i; k<i+5; k++ )
			{
//					// Columns corresponding to coordinates
//				dMatrix( k, 0 ) = (gData((*ite).first)(TypeID::dx));
//				dMatrix( k, 1 ) = (gData((*ite).first)(TypeID::dy));
//				dMatrix( k, 2 ) = (gData((*ite).first)(TypeID::dz));
//
//					// Columns corresponding to receiver clock error dT
//				dMatrix( k, 3 ) = 1;
//
//					// Columns corresponding to Iono
//					// We should test each obs type and give it right coe...
//					
//						// deltaPrefitP1/P2
//				if( (k-i) == 0 ) dMatrix( k, 4+(i-3)/4 ) = miu1;
//				if( (k-i) == 1 ) dMatrix( k, 4+(i-3)/4 ) = miu2;
//
//						// deltaPrefitL1/LW
//				if( (k-i) == 2 ) dMatrix( k, 4+(i-3)/4 ) = -miu1;
//				if( (k-i) == 3 ) dMatrix( k, 4+(i-3)/4 ) =-( e*miu1 - f*miu2 );
//
//					// Columns coresponding to cycle slips
//				if( (k-i) == 2 ) dMatrix( k, 3+1+satsNum+( (i-3)/4 ) ) 
//																	  = L1_WAVELENGTH_GPS;

					// For phase and code measurements
				if( (k-i) < 4 )
				{
						// Columns corresponding to coordinates
					dMatrix( k, 0 ) = (gData((*ite).first)(TypeID::dx));
					dMatrix( k, 1 ) = (gData((*ite).first)(TypeID::dy));
					dMatrix( k, 2 ) = (gData((*ite).first)(TypeID::dz));
	
						// Columns corresponding to receiver clock error dT
					dMatrix( k, 3 ) = 1;
	
						// Columns corresponding to Iono
						// We should test each obs type and give it right coe...
						
							// deltaPrefitP1/P2
					if( (k-i) == 0 ) dMatrix( k, 4+(i-3)/5 ) = miu1;
					if( (k-i) == 1 ) dMatrix( k, 4+(i-3)/5 ) = miu2;
	
							// deltaPrefitL1/LW
					if( (k-i) == 2 ) dMatrix( k, 4+(i-3)/5 ) = -miu1;
					if( (k-i) == 3 ) dMatrix( k, 4+(i-3)/5 ) =-( e*miu1 - f*miu2 );
	
						// Columns coresponding to cycle slips
					if( (k-i) == 2 ) dMatrix( k, 3+1+satsNum+( (i-3)/5 ) ) 
																		  = L1_WAVELENGTH_GPS;


				}  // End of ' if( (k-i) < 4 ) '
				else {  // For iono observable  
					dMatrix( k, 4+(i-3)/5 ) = 1.0;
				}

			}  // End of ' for( size_t k=i; k<i+4; k++ ) ... '
			
				// Update for next sat
//			i = i + 4;
			i = i + 5;

		}  // End of ' for( ite = obsData.begin(); ... '
				
			// Step 2: Solve equation system

			// Use a SolveWMS obj to get the solution
		solver.Compute( y, dMatrix, wMatrix);

	//	std::cout << std::fixed << std::setprecision(3) << std::endl;
	//	std::cout << "y: " << std::endl << y << std::endl;
	//	std::cout << "my dMatrix: " << std::endl << dMatrix << std::endl;

	// vvvvvvvvvvvvvvvvvvvvvvvvvvv debug code vvvvvvvvvvvvvvvvvvvvvvvvvvvv


		// post fit 
		Vector<double> postfitObs(rows, 0.0);
		postfitObs = dMatrix * solver.solution - y;

		int satIndex(0);
		for( ite = obsData.begin(); ite != obsData.end(); ++ite )
		{
			commonData.stvmap[ (*ite).first ][TypeID::dx] = solver.solution(0);
			commonData.stvmap[ (*ite).first ][TypeID::dy] = solver.solution(1);
			commonData.stvmap[ (*ite).first ][TypeID::dz] = solver.solution(2);

			commonData.stvmap[ (*ite).first ][TypeID::cdt] = solver.solution(3);

			commonData.stvmap[ (*ite).first ][TypeID::ionoL1] 
								  = solver.solution(3+1+satIndex);

			commonData.stvmap[ (*ite).first ][TypeID::CSL1]
								  = solver.solution(3+1+satsNum+satIndex);

//			commonData.stvmap[ (*ite).first ][TypeID::iono]
//								  = postfitObs(3+5*satIndex+4);

//			commonData.stvmap[ (*ite).first ][TypeID::dLat]
//								  = postfitObs(0);

//			commonData.stvmap[ (*ite).first ][TypeID::dLon]
//								  = postfitObs(1);

//			commonData.stvmap[ (*ite).first ][TypeID::dH]
//								  = postfitObs(2);
			
			satIndex++;

		}  // End of '  for( ite = obsData.begin(); ... '



	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^ debug code ^^^^^^^^^^^^^^^^^^^^^^^^^^^^

			// Step 3: Detect cycle slip with lambda method
		
			// Declare a Vector for cycle slip vector and matrix for its cov
		Vector<double> csVec( satsNum, 0.0 );
		Matrix<double> csCov( satsNum, satsNum, 0.0 );

		for( size_t i=0; i<satsNum; i++)
		{
			csVec(i) = solver.solution(3+1+satsNum+i);
			for( size_t j=0; j<satsNum; j++ )
			{
				csCov( i, j ) = solver.covMatrix( 3+1+satsNum+i, 3+1+satsNum+j);
			}  // End of ' for( int j=0; j<satsNum; j++ ) ... '
		}  // End of ' for( int i=0; i<satsNum; i++) ... '

			// MLAMBDA
		ARMLambda mlambda;

			// Ratio
		double ratio(0.0);

			// Resolve cycle slip
		mlambda.resolve( csVec, csCov );

		ratio = mlambda.getRatio();


		Vector<double> fixedCSVec = mlambda.getFixedAmbVec();
		

		// vvvvvvvvvvvvvvvvvv debug code vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
		commonData.ratio = ratio;
		ite = obsData.begin();
		for( size_t i=0; i<satsNum; i++ )
		{
			
			commonData.stvmap[ (*ite).first ][TypeID::BL1] = fixedCSVec(i);				
			++ite;

		}  // End of ' for( size_t i=0; i<satsNum; i++ ) '

		// ^^^^^^^^^^^^^^^^^^ debug code ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//		std::cout << "ratio: " << ratio << std::endl;

		if( ratio > ratioThres )
		{
				// Here means that we successfully resolved cycle slip to integers
//			Vector<double> fixedCSVec = mlambda.getFixedAmbVec();
//			std::cout << "fixed CS Vector " << std::endl << fixedCSVec << std::endl;

//			Vector<double> stateFlag( satsNum, 0.0 );
			
			
			// vvvvvvvvvvvvvvvvvvvvvvvv  VALIDATION TEST  vvvvvvvvvvvvvvvvvvvvvvv
			
			// Added by Lei Zhao, 2016/05/03 ~ 2016/05/04, sgg, WHU 
			// By means of ARRound class and chi-square test  
				
//				// ** ARRound
//			double cutDec(1000.0);
//			ARRound ambRes(1000,0.3,0.3);
//
//			for( size_t i=0; i<satsNum; i++ )
//			{
//				double bw( csVec(i) );
//				double bwSig( std::sqrt(csCov(i,i)) );
//				double decision( ambRes.getDecision( bw, bwSig ) );
//				
//				std::cout << "decision: " << decision << std::endl;
//
//					// Look for the largest fixing decision
//				if( decision > cutDec )
//				{
//					fixedCSVec(i) = std::floor( bw + 0.5 );
//					stateFlag(i) = 1.0;
//				}  // End of ' if( decision > cutDec ) '
//				
//			}  // End of ' for( size_t i=0; i<satsNum; i++ ) '

//				// ** Chi-square test
//					
//					// Get Qvv
//			Matrix<double> Qvv( numMeas, numMeas, 0.0 );
//			Matrix<double> Q( numMeas, numMeas, 0.0 );
//					
//					// Let's try to invert weight matrix
//			try
//			{
//				Q = inverseChol( wMatrix );
//			}
//			catch(...)
//			{
//				ProcessingException e( getClassName() + ": Validation error !!! " );
//
//				GPSTK_THROW(e);
//			}
//
//			Qvv = Q - dMatrix * solver.covMatrix * transpose(dMatrix);
//			
//			// std::cout << "Qvv: " << Qvv << std::endl;
//			
//				// Update solution with fixed CSL1
//				// And get the statistic 
//
//			Vector<double> fixedSolution(numUnknowns,0.0);
//
//			for( size_t i=0; i<numUnknowns; i++ )
//			{
//				if( i < 3+1+satsNum )
//				{
//					fixedSolution(i) = solver.solution(i);
//				}
//				else {
//					if( stateFlag(  i - (3+1+satsNum) ) == 1.0 )
//					{
//						fixedSolution(i) = fixedCSVec( i - (3+1+satsNum) );
//					}
//				
//				}  // End of ' if( i < 3+1+satsNum ) '
//
//			}  // End of ' for( size_t i=0; i<satsNum; i++ ) '
//
//				// Get the statistic 
//			double statistic(0.0);
//
//			Vector<double> postfit(numMeas,0.0);
//			postfit = dMatrix * solver.solution - y;
//			
//			for( size_t i=0; i<numMeas; i++ )
//			{
//				statistic += postfit(i) * postfit(i) / Qvv(i,i);
//			}  // End of ' for( size_t i=0; i<numMeas; i++ ) ' 
//
//			std::cout << "statistic: " << statistic << std::endl;
//
//				// Chi-square test
//					
//					// Degree of freedom
//			int degFreedom( numMeas - numUnknowns );
//
//					// Chi2Distribution object
//			Chi2Distribution chiObj;
//
//					// For this statistic, if the upper tail of the chi-square
//					// is less than some significance, eg. 0.001, this statistic
//					// must locate in the rejection region, which means that we 
//					// should reject the H0: statistic <= limit(degFreedom)
//				
//			double upperTailPro( chiObj.Q( statistic, (int)(degFreedom) ) );
//
//			std::cout << "upperTailPro: " << upperTailPro << std::endl;
//			
//			if( upperTailPro < 0.05 ){
//
//				detectionResult = false;
//				return detectionResult;
//			}

			// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^ VALIDATION TEST ^^^^^^^^^^^^^^^^^^^^
			
			
				// Then we should check satellite by satellite
			ite = obsData.begin();
			for( size_t i=0; i<satsNum; i++ )
			{

				if( (fixedCSVec(i) != 0) )
				{
					// vvvvvvvvvvvvvvvvvvvvvvvv debug code vvvvvvvvvvvvvvvvvvvvvvv
					( detectedCS[ commonData.epoch ] ).insert((*ite).first);
					// ^^^^^^^^^^^^^^^^^^^^^^^^ debug code ^^^^^^^^^^^^^^^^^^^^^^^
						// This means cycle slip
					gData[ (*ite).first ][TypeID::CSL1] = 1.0;
					gData[ (*ite).first ][TypeID::CSL2] = 1.0;
					
					if( !detectionResult )
					{
						detectionResult = true;
					}
				}
				else
				{
					gData[ (*ite).first ][TypeID::CSL1] = 0.0;
					gData[ (*ite).first ][TypeID::CSL2] = 0.0;
				}
				++ite;
			}  // End of ' for( size_t i=0; i<satsNum; i++ ) ... '
		}

		return detectionResult;
	}  // End of ' satTypeValueMap& IntegratedDetection( ... '

void MWPlusCSDetector::ComputeStat( Stats<double>& stat, 
												std::deque<double>& deltaLI )
{
	for(std::deque<double>::iterator it = deltaLI.begin();
		 it != deltaLI.end();
		 ++it )
	{
		stat.Add( (*it) );
	}  // End of ' for(std::deque<double>::iterator it = deltaLI.begin(); '
}  // End of ' void MWPlusCSDetector::ComputeStat( Stats<double>& stat, ... '





}  // End of namespace

