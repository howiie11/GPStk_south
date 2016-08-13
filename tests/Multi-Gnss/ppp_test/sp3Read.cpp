
   // Class to store satellite precise navigation data
#include "SP3EphemerisStore.hpp"

using namespace std;
using namespace gpstk;

int main(void)
{
      // Declare a "SP3EphemerisStore" object to handle precise ephemeris
   SP3EphemerisStore SP3EphList;

      // Set flags to reject satellites with bad or absent positional
      // values or clocks
   SP3EphList.rejectBadPositions(true);
   SP3EphList.rejectBadClocks(true);
		
		// Load SP3 file
   SP3EphList.loadFile("ObsData/gbm18781.sp3");


	return 0;


}  // End of ' int main(void) '
