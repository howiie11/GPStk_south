#pragma ident "$Id$"

/**
 * @file PositioningResultsData.hpp
 * Encapsulate PositioningResults file data, versions a,b,c, including I/O
 */

#ifndef GPSTK_POSITIONINGRESULTSDATA_HPP
#define GPSTK_POSITIONINGRESULTSDATA_HPP

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

#include "PositioningResultsBase.hpp"
#include "CommonTime.hpp"
#include <iomanip>
#include "Triple.hpp"

namespace gpstk
{
   /** @addtogroup PositioningResultsephem */
   //@{

      /**
       * This class encapsulates data for satellite orbits and clocks, including
       * positions, velocities and other orbit and estimation information read
       * as found in I/O of PositioningResults format (versions a, b, or c) files.
       *
       * This class is used in conjuction with class PositioningResultsStream, which handles the I/O,
       * and PositioningResultsHeader, which holds information from the PositioningResults file header.
       * Note that the version of PositioningResults is stored ONLY in the PositioningResultsHeader object.
       * This version is set when an PositioningResults header is read into PositioningResultsHeader, and it may
       * be set by the user using PositioningResultsHeader::setVersion().
       * On output, PositioningResultsStream uses the version stored in the PositioningResultsHeader to determine
       * how PositioningResultsData (this object) is output.
       *
       * @code
       * PositioningResultsStream ss("igr14080.sp3");
       * PositioningResultsHeader sh;
       * PositioningResultsData sd;
       *
       * ss >> sh;
       *
       * while (ss >> sd)
       * {
       *    // Interesting stuff...
       * }    
       *
       * PositioningResultsStream ssout("myfile.sp3", ios::out);
       * sh.setVersion(PositioningResultsHeader::PositioningResultsc);
       * ssout << sh;
       * for(...) {
       *    // perhaps modify sd
       *    ssout << sd
       * }
       * @endcode
       *
       * @sa gpstk::PositioningResultsHeader and gpstk::PositioningResultsStream for more information.
       * @sa petest.cpp for an example.
       */
   class PositioningResultsData : public PositioningResultsBase
   {
   public:
         /// Constructor.
      PositioningResultsData() : time(CommonTime::BEGINNING_OF_TIME)
         {}
     
         /// Destructor
      virtual ~PositioningResultsData() {}
     
         // The next four lines is our common interface
         /// PositioningResultsData is "data" so this function always returns true.
      virtual bool isData() const {return true;}

         /// Debug output function.
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
       virtual void dump(std::ostream& s=std::cout, bool includeC=true) const throw() {};
#pragma clang diagnostic pop
         ///@name data members
         //@{
      CommonTime time; ///< Time of epoch for this record
      Triple coordinate;     ///< The three-vector for position | velocity (m | dm/s).

         //@}
      
   protected:

         /// Writes the formatted record to the FFStream \a s.
         /// @warning This function is currently unimplemented
      virtual void reallyPutRecord(FFStream& s) const 
         throw(std::exception, FFStreamError,
               gpstk::StringUtils::StringException) {};

         /**
          * This function reads a record from the given FFStream.
          * If an error is encountered in retrieving the record, the 
          * stream is reset to its original position and its fail-bit is set.
          * @throws StringException when a StringUtils function fails
          * @throws FFStreamError when exceptions(failbit) is set and
          *  a read or formatting error occurs.  This also resets the
          *  stream to its pre-read position.
          */
      virtual void reallyGetRecord(FFStream& s) 
         throw(std::exception, FFStreamError,
               gpstk::StringUtils::StringException);
   };

   //@}

}  // namespace

#endif

