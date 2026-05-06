//
// AirportFlow implementation - now uses dynamic flows from apt.dat only
//
#include "airportFlow.hpp"

namespace world
{
    AirportFlowRegistry AirportFlowRegistry::createWithDefaultFlows()
    {
        // This function is deprecated - the system now uses dynamic TrafficFlow 
        // from apt.dat (lines 1100-1102) exclusively. Hardcoded flows have been removed.
        // The generic parallel runway selection in localController.hpp handles
        // airports without apt.dat flow definitions.
        return AirportFlowRegistry();
    }
}
