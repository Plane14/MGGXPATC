//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#include "conflictResolver.hpp"
#include "verticalSeparationRules.hpp"
#include <cmath>

using namespace std;

namespace world
{
    ConflictResolver::ConflictResolver()
    {
    }

    ConflictResolver::Resolution ConflictResolver::resolveConflict(const ConflictDetector::ConflictInfo& conflict) const
    {
        Resolution resolution;
        resolution.isRequired = conflict.hasConflict;
        resolution.affectedFlight = conflict.flight1;
        resolution.conflictingFlight = conflict.flight2;
        resolution.timeToConflictSeconds = conflict.timeToCPASeconds;

        if (!conflict.hasConflict)
        {
            return resolution;
        }

        // Select the best resolution strategy
        resolution.strategy = selectResolutionStrategy(conflict);

        // Calculate specific resolution based on strategy
        switch (resolution.strategy)
        {
        case ResolutionStrategy::Vertical:
            return calculateVerticalResolution(conflict.flight1, conflict.flight2);
        case ResolutionStrategy::Horizontal:
            return calculateHorizontalResolution(conflict.flight1, conflict.flight2);
        case ResolutionStrategy::Speed:
            return calculateSpeedResolution(conflict.flight1, conflict.flight2);
        default:
            return resolution;
        }
    }

    int ConflictResolver::getFlightPriority(shared_ptr<Flight> flight) const
    {
        if (!flight)
        {
            return 0;
        }

        int priority = 0;

        // Priority based on wake turbulence category
        if (flight->aircraft())
        {
            const auto category = flight->aircraft()->category();
            if (category == Aircraft::Category::Heavy)
            {
                priority += 100; // Heavy aircraft get highest priority
            }
            else if (category == Aircraft::Category::LightProp ||
                     category == Aircraft::Category::Prop)
            {
                priority += 10; // Light aircraft get lower priority
            }
            else
            {
                priority += 50; // Medium aircraft get medium priority
            }

            // Priority based on altitude (higher altitude = more important in enroute)
            priority += static_cast<int>(flight->aircraft()->altitude().feet() / 1000.0f);
        }

        return priority;
    }

    ConflictResolver::ResolutionStrategy ConflictResolver::selectResolutionStrategy(const ConflictDetector::ConflictInfo& conflict) const
    {
        // If time to conflict is very short, prefer vertical resolution
        if (conflict.timeToCPASeconds.count() < 60)
        {
            return ResolutionStrategy::Vertical;
        }

        // If vertical separation is already adequate, use horizontal
        if (conflict.predictedVerticalSeparationFt >= 2000.0f)
        {
            return ResolutionStrategy::Horizontal;
        }

        // Default to vertical resolution
        return ResolutionStrategy::Vertical;
    }

    ConflictResolver::Resolution ConflictResolver::calculateVerticalResolution(
        shared_ptr<Flight> flight1,
        shared_ptr<Flight> flight2) const
    {
        Resolution resolution;
        resolution.strategy = ResolutionStrategy::Vertical;
        resolution.isRequired = true;

        if (!flight1 || !flight2)
        {
            return resolution;
        }

        // Determine which flight to affect based on priority
        const int priority1 = getFlightPriority(flight1);
        const int priority2 = getFlightPriority(flight2);

        if (priority1 >= priority2)
        {
            resolution.affectedFlight = flight2;
            resolution.conflictingFlight = flight1;
        }
        else
        {
            resolution.affectedFlight = flight1;
            resolution.conflictingFlight = flight2;
        }

        // Calculate target altitude
        const float alt1 = flight1->aircraft() ? flight1->aircraft()->altitude().feet() : 0.0f;
        const float alt2 = flight2->aircraft() ? flight2->aircraft()->altitude().feet() : 0.0f;

        VerticalSeparationRules vertRules;
        const float requiredSep = vertRules.getRequiredVerticalSeparation(alt1, alt2);

        if (resolution.affectedFlight == flight1)
        {
            resolution.climb = alt1 < alt2;
            resolution.descend = alt1 >= alt2;
            resolution.targetAltitudeFeet = alt2 + requiredSep;
        }
        else
        {
            resolution.climb = alt2 < alt1;
            resolution.descend = alt2 >= alt1;
            resolution.targetAltitudeFeet = alt1 + requiredSep;
        }

        return resolution;
    }

    ConflictResolver::Resolution ConflictResolver::calculateHorizontalResolution(
        shared_ptr<Flight> flight1,
        shared_ptr<Flight> flight2) const
    {
        Resolution resolution;
        resolution.strategy = ResolutionStrategy::Horizontal;
        resolution.isRequired = true;

        if (!flight1 || !flight2)
        {
            return resolution;
        }

        // Determine which flight to affect based on priority
        const int priority1 = getFlightPriority(flight1);
        const int priority2 = getFlightPriority(flight2);

        if (priority1 >= priority2)
        {
            resolution.affectedFlight = flight2;
            resolution.conflictingFlight = flight1;
        }
        else
        {
            resolution.affectedFlight = flight1;
            resolution.conflictingFlight = flight2;
        }

        // Calculate heading change (simple 30 degree offset)
        if (resolution.affectedFlight->aircraft())
        {
            const float currentHeading = resolution.affectedFlight->aircraft()->heading();
            resolution.targetHeadingDegrees = fmod(currentHeading + 30.0f, 360.0f);
        }

        return resolution;
    }

    ConflictResolver::Resolution ConflictResolver::calculateSpeedResolution(
        shared_ptr<Flight> flight1,
        shared_ptr<Flight> flight2) const
    {
        Resolution resolution;
        resolution.strategy = ResolutionStrategy::Speed;
        resolution.isRequired = true;

        if (!flight1 || !flight2)
        {
            return resolution;
        }

        // Determine which flight to affect based on priority
        const int priority1 = getFlightPriority(flight1);
        const int priority2 = getFlightPriority(flight2);

        if (priority1 >= priority2)
        {
            resolution.affectedFlight = flight2;
            resolution.conflictingFlight = flight1;
        }
        else
        {
            resolution.affectedFlight = flight1;
            resolution.conflictingFlight = flight2;
        }

        // Reduce speed by 10%
        if (resolution.affectedFlight->aircraft())
        {
            const float currentSpeed = resolution.affectedFlight->aircraft()->groundSpeedKt();
            resolution.targetSpeedKt = currentSpeed * 0.9f;
        }

        return resolution;
    }
}