#pragma ident "$Id: CorrectUPDsMGEX.cpp 2939 2012-08-10 19:55:11Z shjzhang $"

/**
* @file CorrectUPDsMGEX.hpp
* Class to correct observables with satellite upds.
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
//  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//  Shoujian Zhang - Wuhan University. 2011, 2012
//
//============================================================================


#include "CorrectUPDsMGEX.hpp"

#include "CivilTime.hpp"

namespace gpstk
{
         // Index initially assigned to this class
      int CorrectUPDsMGEX::classIndex = 4900000;
      

         // Returns an index identifying this object.
      int CorrectUPDsMGEX::getIndex() const
      { return index; }


         // Returns a string identifying this object.
      std::string CorrectUPDsMGEX::getClassName() const
      { return "CorrectUPDsMGEX"; }

   
         /* Returns a satTypeValueMap object, adding the new data generated
          *  when calling this object.
          *
          * @param time      Epoch corresponding to the data.
          * @param gData     Data object holding the data.
          */
      satTypeValueMap& CorrectUPDsMGEX::Process( const CommonTime& time,
                                             satTypeValueMap& gData )
         throw(ProcessingException)
      {
         try
         {
            SatIDSet satRejectedSet;

               // Loop through all the satellites
            for (satTypeValueMap::iterator stv = gData.begin(); 
                 stv != gData.end(); 
                 ++stv)
            {

               try
               {
                     // Get satellite upds from 'RinexUPDStore'
                  satUPD = (*pUPDStore).getValue( (*stv).first, time );

               }
               catch(InvalidRequest& e)
               {
                     // If some problem appears, then schedule this satellite
                     // for removal
						 satRejectedSet.insert( (*stv).first );

                   continue;    // Skip this SV if problems arise

               }

                  // First, correct from cycles to meters.
                  // Warning: 
                  // you should check the unit of the UPDs in the input file.
                  // if it is cycles, then convert it to meters firstly, 
                  // or if it is meters, you can use it directly.
                  //
                  // Here, change the unit to meters firstly.
                  // 2015/09/15
					// Debug code vvv
						// I just want to output the time with upd
					CivilTime ct( time );
					cout << (*stv).first  << "upd time: " << endl;
					cout << ct.hour << " " << ct.minute << " " 
						<< ct.minute << endl;
					cout << satUPD << endl;
					// Debug code ^^^ 

					SatID sat( (*stv).first );
					if( sat.system == SatID::systemGPS )
					{
							// Compute upd on original frequency
	               double f1 = 1575.42e6;
		            double f2 = 1227.60e6;

			         double updSatWL(satUPD.updSatMW), updSatLC(satUPD.updSatLC);
				      double updSatL1, updSatL2;

							// Get updSatL1/updSatL2, unit:cycles
						updSatL1 = updSatLC - f2/(f1-f2)*updSatWL;
						updSatL2 = updSatLC - f1/(f1-f2)*updSatWL;
                 
							// Convert from cycles to meters
						updSatL1 = updSatL1 * 0.190293672798;
						updSatL2 = updSatL2 * 0.244210213425;

						(*stv).second[TypeID::updSatL1] = updSatL1;
						(*stv).second[TypeID::updSatL2] = updSatL2;


							// Change unit from cycle to meter!
						satUPD.updSatMW = satUPD.updSatMW*0.861918400322;
						satUPD.updSatLC = satUPD.updSatLC*0.106953378142;
					}
					else if( sat.system == SatID::systemGalileo )
					{

							// Compute upd on original frequency
//	               double f1 = 1575.42e6;
//		            double f2 = 1227.60e6;

			         double updSatWL(satUPD.updSatMW), updSatLC(satUPD.updSatLC);
				      double updSatL1, updSatL5;

							// Get updSatL1/updSatL2, unit:cycles
//						updSatL1 = updSatLC - f2/(f1-f2)*updSatWL;
//						updSatL2 = updSatLC - f1/(f1-f2)*updSatWL;

						updSatL1 = updSatLC - L5_FREQ_GAL/(L1_FREQ_GAL-L5_FREQ_GAL)*updSatWL;
                 
						updSatL5 = updSatLC - L1_FREQ_GAL/(L1_FREQ_GAL-L5_FREQ_GAL)*updSatWL;

							// Convert from cycles to meters
						updSatL1 = updSatL1 * L1_WAVELENGTH_GAL ;
						updSatL5 = updSatL5 * L5_WAVELENGTH_GAL;

						(*stv).second[TypeID::updSatL1] = updSatL1;
						(*stv).second[TypeID::updSatL5] = updSatL5;


							// Change unit from cycle to meter!
						satUPD.updSatMW = satUPD.updSatMW*WL_WAVELENGTH_GAL_L1L5;
						satUPD.updSatLC = satUPD.updSatLC*LC_WAVELENGTH_GAL_L1L5;
					}
					else 
					{
						Exception e("Unsupproted system!!!");
						GPSTK_THROW(e);
					}

//                  // Now, correct the Melboune-Wubbena combination observables
               (*stv).second[TypeID::updSatMW] = satUPD.updSatMW;
//               (*stv).second[TypeID::updSatWL] = satUPD.updSatMW;
//					(*stv).second[TypeID::updSatLC] = satUPD.updSatLC;
		

//               double f1 = 1575.42e6;
//               double f2 = 1227.60e6;
//
//               double updSatWL, updSatLC;
//               double updSatL1, updSatL2;
//
//                  // Change unit from meter to cycles
//               updSatWL = satUPD.updSatMW/0.861918400322;
//               updSatLC = satUPD.updSatLC/0.106953378142;
//
//                  // Get updSatL1/updSatL2, unit:cycles
//               updSatL1 = updSatLC - f2/(f1-f2)*updSatWL;
//               updSatL2 = updSatLC - f1/(f1-f2)*updSatWL;
//                 
//                  // Convert from cycles to meters
//               updSatL1 = updSatL1 * 0.190293672798;
//               updSatL2 = updSatL2 * 0.244210213425;
//
//               (*stv).second[TypeID::updSatL1] = updSatL1;
//               (*stv).second[TypeID::updSatL2] = updSatL2;


            }  // End of 'for (stv = gData.begin(); stv != gData.end(); ++stv)'

               // Remove satellites with missing data
            gData.removeSatID(satRejectedSet);

               // Return
            return gData;

         }
         catch(Exception& u)
         {

               // Throw an exception if something unexpected happens
            ProcessingException e( getClassName() + ":"
               + StringUtils::asString( getIndex() ) + ":"
               + u.what() );

            GPSTK_THROW(e);

         }

      }  // End of method 'CorrectUPDsMGEX::Process()'

}

