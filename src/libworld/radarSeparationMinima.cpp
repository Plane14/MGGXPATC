//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#include "radarSeparationMinima.hpp"

using namespace std;

namespace world
{
    RadarSeparationMinima::RadarSeparationMinima()
    {
    }

    float RadarSeparationMinima::getRequiredSeparation(FacilityType facilityType) const
    {
        switch (facilityType)
        {
        case FacilityType::Approach:
            return SEPARATION_APPROACH_NM();
        case FacilityType::Center:
            return SEPARATION_ENROUTE_NM();
        case FacilityType::Tower:
            return SEPARATION_TOWER_NM();
        case FacilityType::Unknown:
        default:
            return SEPARATION_ENROUTE_NM(); // Default to enroute standard
        }
    }

    bool RadarSeparationMinima::isSeparationAdequate(float distanceNm, FacilityType facilityType) const
    {
        return distanceNm >= getRequiredSeparation(facilityType);
    }

    float RadarSeparationMinima::getRadarSeparation(std::shared_ptr<Flight> flight1, std::shared_ptr<Flight> flight2) const
    {
        return getDistanceNm(flight1, flight2);
    }

    float RadarSeparationMinima::getDistanceNm(std::shared_ptr<Flight> flight1, std::shared_ptr<Flight> flight2) const
    {
        if (!flight1 || !flight2)
        {
            return 0.0f;
        }

        const auto pos1 = flight1->position();
        const auto pos2 = flight2->position();
        
        if (pos1 == GeoPoint::empty || pos2 == GeoPoint::empty)
        {
            return 0.0f;
        }

        const double distanceMeters = GeoMath::getDistanceMeters(pos1, pos2);
        return static_cast<float>(distanceMeters / METERS_IN_1_NAUTICAL_MILE);
    }
}