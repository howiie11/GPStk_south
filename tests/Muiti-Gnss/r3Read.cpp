#include <iostream>

#include "Rinex3ObsStream.hpp"
#include "Rinex3ObsHeader.hpp"

using namespace std;
using namespace gpstk;

int main(void)
{
		// Create the input file stream
	Rinex3ObsStream rin("ObsData/brst0050.16o");
	
	return 0;
}
