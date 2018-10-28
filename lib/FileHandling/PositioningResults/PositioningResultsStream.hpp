#pragma ident "$Id$"

/**
 * @file PositioningResultsStream.hpp
 * gpstk::PositioningResultsStream - PositioningResults[abc] format file stream
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
//  Copyright 2004, The University of Texas at Austin
//
//============================================================================

//============================================================================
//
//This software developed by Applied Research Laboratories at the University of
//Texas at Austin, under contract to an agency or agencies within the U.S. 
//Department of Defense. The U.S. Government retains all rights to use,
//duplicate, distribute, disclose, or release this software. 
//
//Pursuant to DoD Directive 523024 
//
// DISTRIBUTION STATEMENT A: This software has been approved for public 
//                           release, distribution is unlimited.
//
//=============================================================================
// Date     :  2018/10/28
// Version  :  0.0
// Author(s):  Lei Zhao, a Ph.D. candiate
// School of Geodesy and Geomatics, Wuhan University
//=============================================================================


#ifndef POSITIONINGRESULTSSTREAM_HPP
#define POSITIONINGRESULTSSTREAM_HPP

#include "FFTextStream.hpp"
#include "PositioningResultsHeader.hpp"

namespace gpstk
{
   /// @addtogroup PositioningResults
   //@{

       /// This class performs file I/O on an PositioningResults file for the PositioningResultsHeader
       /// and PositioningResultsData classes.
       /// Note that the file format (a, b or c) is stored in the PositioningResultsHeader (only).
       /// On input it is set by PositioningResultsHeader::reallyGetRecord() by the file content;
       /// for output it may be set (PositioningResultsHeader::setVersion()) before streaming.
   class PositioningResultsStream : public FFTextStream
   {
   public:
         /// Default constructor
      PositioningResultsStream() 
         : wroteEOF(false),
           writingMode(false),
           lastLine(std::string())
         {}
      
         /// Common constructor: open (default: to read)
         /// @param filename the name of the ASCII PositioningResults format file to be opened
         /// @param mode the ios::openmode to be used
      PositioningResultsStream(const char* filename,
                std::ios::openmode mode=std::ios::in)
            : FFTextStream(filename, mode)
            { open(filename, mode); }

         /// destructor; override to force 'close'
      virtual ~PositioningResultsStream()
      {
         if(writingMode && !wroteEOF) close();
      }

         /// override close() to write EOF line
      virtual void close(void) throw(Exception)
      {
         try {
            // if writing, add the final line
            if(writingMode && !wroteEOF) {
               (*this) << "EOF\n"; 
               wroteEOF = true;
            }
            FFTextStream::close();
         }
         catch(std::exception& e) {
            Exception ge(e.what());
            GPSTK_THROW(ge);
         }
      }

         /// override open() to reset the header
         /// @param filename the name of the ASCII PositioningResults format file to be opened
         /// @param mode the ios::openmode to be used
      virtual void open(const char* filename, std::ios::openmode mode)
      {
         FFTextStream::open(filename, mode);
         header = PositioningResultsHeader();
         warnings.clear();

         // for close() later
         wroteEOF = writingMode = false;
         if( (mode & std::ios::out) && !(mode & std::ios::in) )
            writingMode = true;

         // this is necessary in order for PositioningResultsData::reallyGetRecord() to
         // process the last line in the file when there is no EOF record...why?
         if(mode & std::ios::in) exceptions(std::ifstream::failbit);
      }

         ///@name data members
         //@{
      PositioningResultsHeader header;     ///< PositioningResultsHeader for this file
      bool wroteEOF;        ///< True if the final 'EOF' has been read.
      bool writingMode;     ///< True if the stream is open in 'out', not 'in', mode
      CommonTime currentEpoch;   ///< Time from last epoch record read
      std::string lastLine;      ///< Last line read, perhaps not yet processed
      std::vector<std::string> warnings; ///< warnings produced by reallyGetRecord()s
         //@}
   }; // class PositioningResultsStream
   
   //@}
   
} // namespace gpstk

#endif // PositioningResultsSTREAM_INCLUDE
