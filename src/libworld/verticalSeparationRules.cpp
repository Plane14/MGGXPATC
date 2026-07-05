//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#include "verticalSeparationRules.hpp"

using namespace std;

namespace world
{
    VerticalSeparationRules::VerticalSeparationRules()
    {
    }

    float VerticalSeparationRules::getRequiredVerticalSeparation(float altitude1, float altitude2) const
    {
        const float lowerAltitude = min(altitude1, altitude2);
        const float higherAltitude = max(altitude1, altitude2);
        
        if (isAtOrAboveFL290(lowerAltitude))
        {
            return SEPARATION_AT_OR_ABOVE_FL290_FEET();
        }
        
        return SEPARATION_BELOW_FL290_FEET();
    }

    bool VerticalSeparationRules::isVerticalSeparationAdequate(float altitude1, float altitude2) const
    {
        const float separation = abs(altitude1 - altitude2);
        const float required = getRequiredVerticalSeparation(altitude1, altitude2);
        return separation >= required;
    }

    bool VerticalSeparationRules::isAtOrAboveFL290(float altitudeFeet) const
    {
        return altitudeFeet >= RVSM_MINIMUM_ALTITUDE_FEET();
    }

    bool VerticalSeparationRules::checkRVSMCompliance(float altitude1, float altitude2) const
    {
        // RVSM requires 1000ft separation between FL290 and FL410
        if (altitude1 >= RVSM_MINIMUM_ALTITUDE_FEET() && altitude1 < 41000.0f &&
            altitude2 >= RVSM_MINIMUM_ALTITUDE_FEET() && altitude2 < 41000.0f)
        {
            return isVerticalSeparationAdequate(altitude1, altitude2);
        }
        return true; // Not in RVSM airspace, no special compliance needed
    }

    float VerticalSeparationRules::getVerticalSeparation(std::shared_ptr<Flight> flight1, std::shared_ptr<Flight> flight2) const
    {
        if (!flight1 || !flight2)
        {
            return 0.0f;
        }

        const float alt1 = flight1->aircraft() ? flight1->aircraft()->altitude().feet() : 0.0f;
        const float alt2 = flight2->aircraft() ? flight2->aircraft()->altitude().feet() : 0.0f;
        
        return abs(alt1 - alt2);
    }
}