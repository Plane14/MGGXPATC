//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#include "separationManager.hpp"
#include "wakeTurbulenceCalculator.hpp"
#include "verticalSeparationRules.hpp"
#include "radarSeparationMinima.hpp"
#include "conflictDetector.hpp"

using namespace std;

namespace world
{
    SeparationManager::SeparationManager()
    {
        m_wakeTurbulenceCalculator = make_unique<WakeTurbulenceCalculator>();
        m_verticalSeparationRules = make_unique<VerticalSeparationRules>();
        m_radarSeparationMinima = make_unique<RadarSeparationMinima>();
    }

    SeparationResult SeparationManager::calculateSeparation(
        shared_ptr<Flight> leader,
        shared_ptr<Flight> follower,
        FacilityType facilityType) const
    {
        SeparationResult result;

        if (!leader || !follower)
        {
            return result;
        }

        // Calculate wake turbulence separation
        const auto leaderWakeClass = m_wakeTurbulenceCalculator->inferWakeClass(leader);
        const auto followerWakeClass = m_wakeTurbulenceCalculator->inferWakeClass(follower);
        result.wakeTurbulenceSeconds = static_cast<float>(
            m_wakeTurbulenceCalculator->getWakeTurbulenceSeparationSeconds(leaderWakeClass, followerWakeClass)
        );

        // Calculate vertical separation
        result.verticalSeparationFeet = m_verticalSeparationRules->getVerticalSeparation(leader, follower);

        // Calculate radar separation
        result.radarSeparationNm = m_radarSeparationMinima->getRadarSeparation(leader, follower);

        return result;
    }

    bool SeparationManager::checkConflict(
        shared_ptr<Flight> flight1,
        shared_ptr<Flight> flight2,
        FacilityType facilityType) const
    {
        ConflictDetector detector;
        ConflictDetector::DetectionParams params;
        
        // Set appropriate thresholds based on facility type
        if (facilityType == FacilityType::Approach)
        {
            params.horizontalThresholdNm = 3.0f;
        }
        else
        {
            params.horizontalThresholdNm = 5.0f;
        }

        const auto conflict = detector.checkConflict(flight1, flight2, params);
        return conflict.hasConflict;
    }

    WakeTurbulenceCategory SeparationManager::getWakeTurbulenceCategory(shared_ptr<Flight> flight) const
    {
        if (!flight)
        {
            return WakeTurbulenceCategory::Medium;
        }

        const auto wakeClass = m_wakeTurbulenceCalculator->inferWakeClass(flight);
        
        switch (wakeClass)
        {
        case WakeTurbulenceCalculator::WakeClass::Light:
            return WakeTurbulenceCategory::Light;
        case WakeTurbulenceCalculator::WakeClass::Medium:
            return WakeTurbulenceCategory::Medium;
        case WakeTurbulenceCalculator::WakeClass::Heavy:
            return WakeTurbulenceCategory::Heavy;
        case WakeTurbulenceCalculator::WakeClass::Super:
            return WakeTurbulenceCategory::Super;
        default:
            return WakeTurbulenceCategory::Medium;
        }
    }

    int SeparationManager::getWakeTurbulenceSeparationSeconds(WakeTurbulenceCategory leader, WakeTurbulenceCategory follower) const
    {
        return m_wakeTurbulenceCalculator->getWakeTurbulenceSeparationSeconds(
            static_cast<WakeTurbulenceCalculator::WakeClass>(leader),
            static_cast<WakeTurbulenceCalculator::WakeClass>(follower)
        );
    }
}