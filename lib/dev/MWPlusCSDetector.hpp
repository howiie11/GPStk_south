#pragma ident "$Id$"

/**
 *	@file MWPlusCSDetector.hpp
 *	This class augments MWCSDetector by the means of taking use of MWCSDetector
 * results and lambda method
 */

#ifndef GPSTK_MWPLUSCSDetector_HPP
#define GPSTK_MWPLUSCSDetector_HPP

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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2009, 2011
//
//============================================================================
//
// Versions: 
//
// 2016/04/13
//	Lei Zhao created this program, in sgg, WHU
//
//	2016/05/03 ~ 05/04
// Add validation part 1. ARRound 2. chi-square test
// By Lei Zhao, sgg, WHU
//
//============================================================================


#include <deque>

#include "ProcessingClass.hpp"
#include "SolverWMS.hpp"
#include "ARMLambda.hpp"
#include "Chi2Distribution.hpp"
#include "ARRound.hpp"
#include "Stats.hpp"


namespace gpstk
{

		/** @addtogroup GPSsolutions */
      //@{


      /** This is a class to detect cycle slips using MW observables.
       *
       * This class is meant to be used with the GNSS data structures objects
       * found in "DataStructures" class.
       *
       * A typical way to use this class follows:
       *
       * @code
       *   RinexObsStream rin("ebre0300.02o");
       *
       *   gnssRinex gRin;
       *   ComputeMelbourneWubbena getMW;
       *   MWCSDetector markCSMW;
		 *	  MWPlusCSDetector markCSMWPlus;
       *
       *   while(rin >> gRin)
       *   {
       *      gRin >> getMW >> markCSMW >> markCSMWPlus;
       *   }
       * @endcode
       *
       * The "MWCSDetector" object will visit every satellite in the GNSS data
       * structure that is "gRin" and will decide if a cycle slip has happened
       * in the given observable.
       *
       * The algorithm will use MW observables, and the LLI1 and LLI2 indexes.
       * The result (a 1 if a cycle slip is found, 0 otherwise) will be stored
       * in the data structure both as the CSL1 and CSL2 indexes.
       *
       * In taking the decision, this algorithm will use criteria as the
       * maximum interval of time between two successive epochs and the
       * maximum number of Melbourne-Wubbena wavelenghts allowed above or
       * below the MW combination average for that arc.
       *
       * The default values are usually fine, but you may change them with the
       * appropriate methods. This is of special importance for the maximum
       * interval time, that should be adjusted for your sampling rate. It is
       * 61 seconds by default, which is appropriate for 30 seconds per sample
       * RINEX observation files.
       *
       * When used with the ">>" operator, this class returns the same
       * incoming data structure with the cycle slip indexes inserted along
       * their corresponding satellites. Be warned that if a given satellite
       * does not have the observations required, it will be summarily deleted
       * from the data structure.
       *
       * You should be aware that the Melbourne-Wubbena combination is based
       * on a mix of code and phase observations, so it is very noisy.
       * Therefore, it has a tendency to yield a high number of false
       * positives if you are not careful with its parameters. Because of
       * this, the default parameters are very conservative, i.e., the
       * detector is NOT very sensitive by default.
       *
       * \warning Cycle slip detectors are objets that store their internal
       * state, so you MUST NOT use the SAME object to process DIFFERENT data
       * streams.
       *
       */
	class MWPlusCSDetector : public ProcessingClass
	{
	public:

			/// Default constructor, setting default parameters
		MWPlusCSDetector();

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

         /** Returns a gnnsSatTypeValue object, adding the new data generated
          *  when calling this object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData)
         throw(ProcessingException)
      { (*this).Process(gData.header.epoch, gData.body); return gData; };

         /** Returns a gnnsRinex object, adding the new data generated when
          *  calling this object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssRinex& Process(gnssRinex& gData)
         throw(ProcessingException);

			/// Set receiver state
		virtual MWPlusCSDetector& setReceiverStatic( bool state)
		{ staticReceiver = state; return (*this); }


			/// Set ratio threshold
		virtual MWPlusCSDetector& setRatioThreshold( double thres )
		{ ratioThres = thres; return (*this); }


		
			/// Set elevation threshold
		virtual MWPlusCSDetector& setElevationThreshold( double thres )
		{ eleThreshold = thres; return (*this); }


			/// Some Rinex data files provide C1 instead of P1. Use this method
			/// in those cases.
		virtual MWPlusCSDetector& useC1(void)
		{ obsType1 = TypeID::prefitC; return (*this); };


			/// Returns a string identifying this object.
		virtual std::string getClassName(void) const;

			/// Destructor
		virtual ~MWPlusCSDetector() {};

		// vvvvvvvvvvvvvvvvvv debug data struct vvvvvvvvvvvvvvvvvvvvvvvv
		
		struct debugData {
			debugData() : presentTime(CommonTime::BEGINNING_OF_TIME),
							  indicator(false) 
			{}
				// Highlighted data
			CommonTime presentTime;
			bool indicator;

		};

		struct debugData2{

			debugData2() : ratio(0.0), epoch(CommonTime::BEGINNING_OF_TIME),
							  numSats(0), indicator(false) 
			{}
				// Highlighted Data
			bool indicator;
			double ratio;
			CommonTime epoch;
			size_t numSats;
			satTypeValueMap stvmap;
		};

		std::map<SatID, debugData> satDebugData;

		debugData2 commonData;
		
		std::map<CommonTime, SatIDSet> detectedCS;
		
		// ^^^^^^^^^^^^^^^^^^ debug data struct ^^^^^^^^^^^^^^^^^^^^^^^^
 

	private:
		
			/// Observation types used in this class
		TypeID obsType1;
		TypeID obsType2;
		TypeID obsType3;
		TypeID obsType4;
		TypeID obsType5;

			/// Cycle slip type
		TypeID csType;

		
			/// Variance of observations

		double deltaPosVar[3];
		double deltaPfP1Var;
		double deltaPfP2Var;
		double deltaPfL1Var;
		double deltaPfLWVar;



			/// Ratio threshold
		double ratioThres;

			/// Elevation threshold
		double eleThreshold;

			/// deltaLI buffer control
		size_t minBuffer;
		size_t maxBuffer;

			/// Receiver state
		bool staticReceiver;

			/// First epoch flag for static receiver
		bool firstTime;
			/// Initializing method 
		void Init(void);

			/// Variance of original observables
		static const double pCodeStd;
		static const double carrierWaveStd;

			/// Other static members
		static const double miu1;
		static const double miu2;
		static const double unitWeightVar;
		static const double e;
		static const double f;

			// Set default factor that multiplies phase weights
			// If code sigma is 1 m and phase sigma is 1 cm, the ratio is 100:1
		static const double weightFactor;

			/// SolerLMS obj
		SolverWMS solver;

			/// A structure used to store filter data for a SV
		struct filterData
		{
				// Default constructor initializating the data in the structure
			filterData() : windowSize(0), formerPrefitP1(0.0),
								formerPrefitP2(0.0), formerPrefitL1(0.0),
								formerPrefitLW(0.0), formerWeight(0.0),
								formerLI(0.0)
			{};

			int windowSize;
			double formerPrefitP1;
			double formerPrefitP2;
			double formerPrefitL1;
			double formerPrefitLW;
			double formerWeight;
			
			double formerLI;

			std::deque<double> deltaLIDeque;
		};
		
			/// This struct is designed for the convenience
		struct obs
		{
			obs() : deltaPrefitP1(0.0), deltaPrefitP2(0.0),
					  deltaPrefitL1(0.0), deltaPrefitLW(0.0),
					  weightOfDeltaPreP(0.0), deltaLIAve(0.0),
					  deltaLIVar(0.0)
			{ };

			double deltaPrefitP1;
			double deltaPrefitP2;
			double deltaPrefitL1;
			double deltaPrefitLW;

				// Weight of delta prefitP 
			double weightOfDeltaPreP;

				// LI
			double deltaLIAve;
			double deltaLIVar;

		};


			/// Map holding the information regarding every satellite
		std::map<SatID, filterData> satFilterData;

			/// Map holding the obs regarding every satellite. 
		std::map<SatID,obs> obsData;


         /** Method to get filter data of specifying sat 
          *
          * @param sat       SatID.
          * @param epochflag Epoch flag.
          * @param pfp1       
          * @param pfp2     
          * @param pfl1
			 * @param pflw
			 * @param csflag
          */
      virtual void getFilterData(const SatID& sat,
                                 const short& epochflag,
                                 const double& pfp1,
                                 const double& pfp2,
                                 const double& pfl1,
											const double& pflw, 
											const double& csflag,
											const double& weight,
											const double& li_ );


			/** Integrated detection method
			 * 
			 * Returns a satTypeValueMap
			 *
			 */
		bool IntegratedDetection( satTypeValueMap& gData,
										  size_t& satsNum );

			/** Statistial computation
			 *
			 */
		void ComputeStat( Stats<double>& stat, std::deque<double>& deltaLI );

	};  // End of class 'MWPlusCSDetector'


}  // End of namespace gpstk




#endif  // GPSTK_MWPLUSCSDetector_HPP
