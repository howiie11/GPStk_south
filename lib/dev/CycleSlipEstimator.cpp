#pragma ident "$Id$"

/**
 * @file CycleSlipEstimator.cpp 
 * This is a class to estimate cycle slips using time differenced code and phase
 * observables, as well as pseudo ionosphere observable 
 *
 */ 

//============================================================================
// 
//  This file is part of GPSTk, the GPS Toolkit.
//
//	 For more information, please refer to the header file.
//
//============================================================================


#include "CycleSlipEstimator.hpp"

namespace gpstk
{
		// Return a string identifying this object
	std::string CycleSlipEstimator::getClassName() const
	{ return "CycleSlipEstimator"; }

		/* Common constructor 
		 *
		 * @param staticRec
		 *
		 */
//	CycleSlipEstimator::CycleSlipEstimator( bool staticRec )
//		: staticReceiver( staticRec ) : firstTime( true ) 
//	{
//			// Call initializing method
//		Init();
//
//	}   // End of ' CycleSlipEstimator::CycleSlipEstimator( bool staticRec ) '


		// Initializing method
//	void CycleSlipEstimator::Init( void )
//	{
//			// By Default, GPS code and phase observables are set 
//
//	}   // End of 'void CycleSlipEstimator::Init( void )'


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
  satTypeValueMap& CycleSlipEstimator::Process( 
												const CommonTime& epoch,
                                    satTypeValueMap& gData,
                                    const short& epochflag )
     throw(ProcessingException)
	{
		try
		{
				// Variable boxes for obeservations of different systems 
			double GP1(0.0);
			double GP2(0.0);
			double GL1(0.0);
			double GL2(0.0);

			SatIDSet satRejectedSet;

				// Loop through all the satellites
			satTypeValueMap::iterator it;
			for( it = gData.begin(); it != gData.end(); ++it )
			{
				try
				{
					//code1 = (*it).second( codeType1 );
					//code2 = (*it).second( codeType2 );
					
//						// Loop through sysObsTypes 
//					std::map<SatID::SatelliteSystem, TypeIDSet>:: const_iterator it;
//					for( it = sysObsTypes.begin(); 
//						  it != sysObsTypes.end();
//						  ++it )
//					{
//							// Sys
//						SatID::SatelliteSystem sys = (*it).first;
//
//							// 
//
//					}	// End of ' for( it = sysObsTypes.begin(); '
					
				}
				catch(...)
				{
						// If some value is missing, then schedule this satellite
						// for removal
					satRejectedSet.insert( (*it).first );
					continue;
				}   // End of 'try-catch' block

					// Get filter data of this sat at this epoch 
				//getFilterData( (*it).first, epochflag, code1, code2 );
			}   // End of ' for( it = gData.begin();  ... '

		}

		catch( Exception& u )
		{
				// Thrown an exception if something unexpected happens
			ProcessingException e( getClassName() + ":" 
											+ u.what() );

			GPSTK_THROW(e);

		}   // End of 'try-catch' block



	}   // End of 'virtual satTypeValueMap& CycleSlipEstimator::Process( ... '




}   // End of namespace gpstk 
