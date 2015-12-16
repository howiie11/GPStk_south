#pragma ident "$Id: RinexUPDHeader.cpp 2897 2011-09-14 20:08:46Z shjzhang $"

/**
 * @file RinexUPDHeader.cpp
 * Encapsulate header of satellite upd file header data, including I/O
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
//  Shoujian Zhang, Wuhan University, 2012
//
//============================================================================

#include "StringUtils.hpp"
#include "CommonTime.hpp"
#include "RinexUPDStream.hpp"
#include "RinexUPDHeader.hpp"
#include "TimeString.hpp"
#include "Epoch.hpp"

#define debug 0

using namespace std;

namespace gpstk
{
   using namespace StringUtils;

   const string RinexUPDHeader::versionString =        "RINEX VERSION / TYPE";
   const string RinexUPDHeader::runByString =          "PGM / RUN BY / DATE";
   const string RinexUPDHeader::commentString =        "COMMENT";
   const string RinexUPDHeader::sysString =            "SYS / # / OBS TYPES";
   const string RinexUPDHeader::timeSystemString =     "TIME SYSTEM ID";
   const string RinexUPDHeader::leapSecondsString =    "LEAP SECONDS";
   const string RinexUPDHeader::sysDCBString =         "SYS / DCBS APPLIED";
   const string RinexUPDHeader::sysPCVString =         "SYS / PCVS APPLIED";
   const string RinexUPDHeader::numDataString =        "# / TYPES OF DATA";
   const string RinexUPDHeader::stationNameString =    "STATION NAME / NUM";
   const string RinexUPDHeader::stationClockRefString ="STATION CLK REF";
   const string RinexUPDHeader::analysisCenterString = "ANALYSIS CENTER";
   const string RinexUPDHeader::numClockRefString =    "# OF CLK REF";
   const string RinexUPDHeader::analysisClkRefrString ="ANALYSIS CLK REF";
   const string RinexUPDHeader::numReceiversString =   "# OF SOLN STA / TRF";
   const string RinexUPDHeader::solnStateString =      "SOLN STA NAME / NUM";
   const string RinexUPDHeader::numSolnSatsString =    "# OF SOLN SATS";
   const string RinexUPDHeader::prnListString =        "PRN LIST";
   const string RinexUPDHeader::endOfHeaderString =    "END OF HEADER";


      // Get the record from the satellite upd file
   void RinexUPDHeader::reallyGetRecord(FFStream& ffs)
      throw(exception, FFStreamError, StringException)
   {
      RinexUPDStream& strm = dynamic_cast<RinexUPDStream&>(ffs);
      
         // if header is already read, just return
      if(strm.headerRead) return;

         // clear the storage
      clear();

      string line;
      while(!(valid & endOfHeaderValid)) 
      {
            // Get a line
         strm.formattedGetLine(line);
         stripTrailing(line);

            // Debug
         if(debug) 
         {
            cout << "RinexUPD Header Line " << line << endl;
         }

            // Check line length
         if(line.length() < 60 || line.length() > 80) 
         {
            FFStreamError e("Invalid line length");
            GPSTK_THROW(e);
         }

            // Parse the line
         try 
         {
            string label(line, 60, 20);
            if(label == versionString) 
            {
               version = asDouble(line.substr(0,9));
               if( line[20] != 'B' && line[20] != 'C') 
               {
                  FFStreamError e("Invalid file type: " + line.substr(20,1));
                  GPSTK_THROW(e);
               }

                  // TD system
               valid |= versionValid;
            }
            else if(label == runByString) 
            {
               program = strip(line.substr(0,20));
               runby = strip(line.substr(20,20));

               valid |= runByValid;
            }
            else if(label == commentString) 
            {
               commentList.push_back(strip(line.substr(0,60)));

               valid |= commentValid;
            }
            else if(label == sysString) 
            {
                 // ???
               valid |= sysValid;
            }
            else if(label == timeSystemString) 
            {
               string ts(upperCase(line.substr(3,3)));
               timeSystem.fromString(ts);
               valid |= timeSystemValid;
            }
            else if(label == leapSecondsString) 
            {
               leapSeconds = asInt(line.substr(0,6));
               valid |= leapSecondsValid;
            }
            else if(label == sysDCBString) 
            {
                 // ???
               valid |= sysDCBValid;
            }
            else if(label == sysPCVString) 
            {
               if(line[0] == 'G')
               {
                  pcvsSystem = RinexSatID(-1,RinexSatID::systemGPS);
               }
               else if(line[0] == 'R')
               {
                  pcvsSystem = RinexSatID(-1,RinexSatID::systemGlonass);
               }
               else 
               {
                  FFStreamError e("Invalid pcvs system : " + line.substr(0,1));
                  GPSTK_THROW(e);
               }

               pcvsProgram = strip(line.substr(1,17));
               pcvsSource = strip(line.substr(20,40));

               valid |= sysPCVValid;
            }
            else if(label == numDataString) 
            {
               int n(asInt(line.substr(0,6)));

               for(int i=0; i<n; ++i)
               {
                  dataTypes.push_back(line.substr(10+i*6,2));
               }
               valid |= numDataValid;
            }
            else if(label == stationNameString) 
            {
               string label(strip(line.substr(0,4)));
               stationID[label] = strip(line.substr(5,20));
               valid |= stationNameValid;
            }
            else if(label == stationClockRefString) 
            {
               valid |= stationClockRefValid;
            }
            else if(label == analysisCenterString) 
            {
               analCenterDesignator = strip(line.substr(0,3));
               analysisCenter = strip(line.substr(5,55));
               valid |= analysisCenterValid;
            }
            else if(label == numClockRefString) 
            {
               valid |= numClockRefValid;
            }
            else if(label == analysisClkRefrString) 
            {
               valid |= analysisClkRefrValid;
            }
            else if(label == numReceiversString) 
            {
               numSolnStations = asInt(line.substr(0,6));
               terrRefFrame = strip(line.substr(10,50));
               valid |= numReceiversValid;
            }
            else if(label == solnStateString) 
            {
               string label(strip(line.substr(0,4)));
               stationID[label] = strip(line.substr(5,20));
               stationX[label] = strip(line.substr(25,11));
               stationY[label] = strip(line.substr(37,11));
               stationZ[label] = strip(line.substr(49,11));
               valid |= solnStateValid;
            }
            else if(label == numSolnSatsString) 
            {
               numSolnSatellites = asInt(line.substr(0,6));
               valid |= numSolnSatsValid;
            }
            else if(label == prnListString) 
            {
               int i,prn;
               string label;
               for(i=0; i<15; ++i) 
               {
                  label = line.substr(4*i,3);
                  if(label == string("   ")) break;
                  prn = asInt(line.substr(4*i+1,2));
                  if(line[4*i] == 'G')
                     satList.push_back(RinexSatID(prn,RinexSatID::systemGPS));
                  else if(line[4*i] == 'R')
                     satList.push_back(RinexSatID(prn,RinexSatID::systemGlonass));
                  else 
                  {
                     FFStreamError e("Invalid sat (PRN LIST): /" + label + "/");
                     GPSTK_THROW(e);
                  }
               }
               // TD how to check numSolnSatsValid == satList.size() ?
               valid |= prnListValid;
            }
            else if(label == endOfHeaderString) 
            {
               valid |= endOfHeaderValid;
            }
            else 
            {
               FFStreamError e("Invalid line label: " + label);
               GPSTK_THROW(e);
            }

            if(debug) cout << "Valid is " << hex << valid << fixed << endl;

         }  // end parsing the line
         catch(FFStreamError& e) { GPSTK_RETHROW(e); }

      }  // end while end-of-header not found

      if(debug) cout << "Header read; Valid is " << hex << valid << fixed << endl;

      // is this header valid?
      if( (valid & allRequiredValid) != allRequiredValid) 
      {
         cout << "Header is invalid on input (valid is x" << hex << valid
              << dec << ").\n";
         dumpValid(cout);
         FFStreamError e("Invalid header");
         GPSTK_THROW(e);
      }

      strm.headerRead  = true;

   }  // end RinexUPDHeader::reallyGetRecord()


   void RinexUPDHeader::reallyPutRecord(FFStream& ffs) const
      throw(exception, FFStreamError, StringException)
   {
      try 
      {
         RinexUPDStream& strm = dynamic_cast<RinexUPDStream&>(ffs);

            // is this header valid?
         if( (valid & allRequiredValid) != allRequiredValid) 
         {
            FFStreamError e("Invalid header");
            GPSTK_THROW(e);
         }

         int i;
         string line;
         try 
         {
            line = rightJustify(asString(version,2), 9);
            line += string(11,' ');
            line += string("CLOCK") + string(15,' ');
            line += string("GPS") + string(17,' ');      // TD fix
            line += versionString;         // "RINEX VERSION / TYPE"
            strm << line << endl;
            strm.lineNumber++;

            line = leftJustify(program,20);
            line += leftJustify(runby,20);
            CommonTime dt;
            static_cast<Epoch>(dt).setLocalTime();
            string dat = printTime(dt,"%02m/%02d/%04Y %02H:%02M:%02S");
            line += leftJustify(dat, 20);
            line += runByString;           // "PGM / RUN BY / DATE"
            strm << line << endl;
            strm.lineNumber++;

            if(valid & sysValid) 
            {
               line = string(60,' ');  // TD
               line += sysString;             // "SYS / # / OBS TYPES"
               strm << line << endl;
               strm.lineNumber++;
            }

            if(valid & timeSystemValid) 
            {
               line = string(60,' ');  // TD
               line += timeSystemString;      // "TIME SYSTEM ID"
               strm << line << endl;
               strm.lineNumber++;
            }

            for(i=0; i<commentList.size(); ++i) 
            {
               line = leftJustify(commentList[i],60);
               line += commentString;         // "COMMENT"
               strm << line << endl;
               strm.lineNumber++;
            }

            if(valid & leapSecondsValid) 
            {
               line = rightJustify(asString(leapSeconds), 6);
               line += string(54,' ');
               line += leapSecondsString;     // "LEAP SECONDS"
               strm << line << endl;
               strm.lineNumber++;
            }

            if(valid & sysDCBValid) 
            {
               line = string(60,' ');  // TD
               line += sysDCBString;          // "SYS / DCBS APPLIED"
               strm << line << endl;
               strm.lineNumber++;
            }

            if(valid & sysPCVValid) 
            {
               line = string("  ");
               line[0] = pcvsSystem.systemChar();
               line += leftJustify(pcvsProgram,17);
               line += string(" ");
               line += leftJustify(pcvsSource,40);
               line += sysPCVString;          // "SYS / PCVS APPLIED"
               strm << line << endl;
               strm.lineNumber++;
            }

            line = rightJustify(asString(dataTypes.size()), 6);
            for(i=0; i<dataTypes.size(); ++i)
            {
               line += string(4,' ') + dataTypes[i];
            }
            line += string(60-line.size(),' ');
            line += numDataString;         // "# / TYPES OF DATA"
            strm << line << endl;
            strm.lineNumber++;

            line = analCenterDesignator;
            line += string(2,' ');
            line += leftJustify(analysisCenter,55);
            line += analysisCenterString;  // "ANALYSIS CENTER"
            strm << line << endl;
            strm.lineNumber++;

            line = rightJustify(asString(numSolnStations), 6);
            line += string(4,' ');
            line += leftJustify(terrRefFrame,50);
            line += numReceiversString;    // "# OF SOLN STA / TRF"
            strm << line << endl;
            strm.lineNumber++;

            map<string,string>::const_iterator it, jt;
            for(it=stationID.begin(); it != stationID.end(); ++it) 
            {
               string label(it->first),field;
               line = label;
               line += string(1,' ');
               line += leftJustify(it->second,20);
               jt = stationX.find(label);
               field = jt->second;
               line += rightJustify(field, 11);
               line += string(1,' ');
               jt = stationY.find(label);
               field = jt->second;
               line += rightJustify(field, 11);
               line += string(1,' ');
               jt = stationZ.find(label);
               field = jt->second;
               line += rightJustify(field, 11);
               line += solnStateString;       // "SOLN STA NAME / NUM"
               strm << line << endl;
               strm.lineNumber++;
            }

            line = rightJustify(asString(numSolnSatellites), 6);
            line += string(54,' ');
            line += numSolnSatsString;     // "# OF SOLN SATS"
            strm << line << endl;
            strm.lineNumber++;

            line = string();
            for(i=0; i<satList.size(); ++i) 
            {
               string satstr(" ");
               satstr[0] = satList[i].systemChar();
               satstr += rightJustify(asString(satList[i].id), 2);
               if(satstr[1] == ' ') satstr[1] = '0';
               line += satstr + string(1,' ');
               if(((i+1) % 15) == 0 || i==satList.size()-1) 
               {
                  line += string(60-line.size(),' ');
                  line += prnListString;         // "PRN LIST"
                  strm << line << endl;
                  strm.lineNumber++;
                  line = string();
               }
            }

            line = string(60,' ');
            line += endOfHeaderString;     // "END OF HEADER"
            strm << line << endl;
            strm.lineNumber++;
         }
         catch(FFStreamError& e) { GPSTK_RETHROW(e); }
         catch(StringException& e) { GPSTK_RETHROW(e); }

      }
         catch(Exception& e) { GPSTK_RETHROW(e); }
         catch(exception& e) { Exception g(e.what()); GPSTK_THROW(g); }
   }


   void RinexUPDHeader::dump(ostream& os, short detail) const throw()
   {
      int i;
      os << "Dump RinexUPD Header:\n";
      os << " Version = " << fixed << setprecision(2) << version
         << " Prgm /"     << program << "/ Run By /" << runby << "/" << endl;
      os << " There are " << dataTypes.size() << " data types, as follows:";

      for(i=0; i<dataTypes.size(); ++i)
      {
         os << " " << dataTypes[i];
      }

      os << endl;
      os << " Leap seconds is " << leapSeconds << endl;
      os << " Analysis center: /" << analCenterDesignator
         << "/ /" << analysisCenter << "/" << endl;
      os << " Terrestrial Reference Frame " << terrRefFrame << endl;
      os << " PCVs: " << pcvsSystem << " /" << pcvsProgram << "/ /"
         << pcvsSource << "/" << endl;
      os << " Comments:\n";
      for(i=0; i<commentList.size(); ++i)
      {
         os << "    " << commentList[i] << endl;
      }
      os << " There are " << stationID.size() << " stations." << endl;
      os << " There are " << satList.size() << " satellites." << endl;

      if(detail > 0) 
      {
         os << " Stations:  identifier     X(mm)       Y(mm)       Z(mm)\n";
         map<string,string>::const_iterator it, jt;
         for(it=stationID.begin(); it!=stationID.end(); ++it) 
         {
            string label(it->first),field;
            os << "     " << label << "   " << it->second;
            jt = stationX.find(label);
            field = jt->second;
            os << rightJustify(field,12);
            jt = stationY.find(label);
            field = jt->second;
            os << rightJustify(field,12);
            jt = stationZ.find(label);
            field = jt->second;
            os << rightJustify(field,12) << endl;
         }
         os << " Sat list:\n";
         for(i=0; i<satList.size(); ++i) 
         {
            os << " " << satList[i];
            if(((i+1)%15) == 0 || i == satList.size()-1) os << endl;
         }

         if(detail >= 2) dumpValid(os);
      }

      os << "End of RinexUPD header dump." << endl;

   }  // end RinexUPDHeader::dump()

   void RinexUPDHeader::dumpValid(ostream& os) const throw()
   {
      if( (valid & allValid) == allValid) return;

      string tag("  Invalid or missing header line: ");
      os << "Dump invalid or missing header records:\n";

      if(!(valid & versionValid))          os << tag << versionString << endl;
      if(!(valid & runByValid))            os << tag << runByString << endl;
      if(!(valid & commentValid))          os << tag << commentString << endl;
      if(!(valid & sysValid))              os << tag << sysString << endl;
      if(!(valid & timeSystemValid))       os << tag << timeSystemString << endl;
      if(!(valid & leapSecondsValid))      os << tag << leapSecondsString << endl;
      if(!(valid & sysDCBValid))           os << tag << sysDCBString << endl;
      if(!(valid & sysPCVValid))           os << tag << sysPCVString << endl;
      if(!(valid & numDataValid))          os << tag << numDataString << endl;
      if(!(valid & stationNameValid))      os << tag << stationNameString << endl;
      if(!(valid & stationClockRefValid))  os << tag << stationClockRefString << endl;
      if(!(valid & analysisCenterValid))   os << tag << analysisCenterString << endl;
      if(!(valid & numClockRefValid))      os << tag << numClockRefString << endl;
      if(!(valid & analysisClkRefrValid))  os << tag << analysisClkRefrString << endl;
      if(!(valid & numReceiversValid))     os << tag << numReceiversString << endl;
      if(!(valid & solnStateValid))        os << tag << solnStateString << endl;
      if(!(valid & numSolnSatsValid))      os << tag << numSolnSatsString << endl;
      if(!(valid & prnListValid))          os << tag << prnListString << endl;
      if(!(valid & endOfHeaderValid))      os << tag << endOfHeaderString << endl;

      os << "End of invalid or missing dump" << endl;
   }

}  // namespace
