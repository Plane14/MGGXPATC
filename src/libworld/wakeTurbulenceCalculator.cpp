//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#include "wakeTurbulenceCalculator.hpp"
#include <algorithm>
#include <cctype>

using namespace std;

namespace world
{
    WakeTurbulenceCalculator::WakeTurbulenceCalculator()
    {
    }

    string WakeTurbulenceCalculator::uppercaseCopy(const string& value)
    {
        string result = value;
        transform(result.begin(), result.end(), result.begin(), [](unsigned char c) {
            return static_cast<char>(toupper(c));
        });
        return result;
    }

    WakeTurbulenceCalculator::WakeClass WakeTurbulenceCalculator::inferWakeClass(shared_ptr<Flight> flight) const
    {
        if (!flight || !flight->aircraft())
        {
            return WakeClass::Medium;
        }

        const string modelIcao = uppercaseCopy(flight->aircraft()->modelIcao());
        
        // Super wake class (A388, A225)
        if (modelIcao.rfind("A388", 0) == 0 || modelIcao.rfind("A225", 0) == 0)
        {
            return WakeClass::Super;
        }

        // Heavy wake class prefixes
        static const vector<string> heavyPrefixes = {
            "A30", "A33", "A34", "A35", "A38",
            "B74", "B76", "B77", "B78",
            "C17", "C5", "C130", "DC1", "IL7", "IL9", "MD1",
            "A400", "KC13", "KC10", "KC46", "KC30",
            "E3", "E6", "E8", "B52", "B1", "P8", "P3", "C2"
        };

        for (const auto& prefix : heavyPrefixes)
        {
            if (modelIcao.rfind(prefix, 0) == 0)
            {
                return WakeClass::Heavy;
            }
        }

        const auto category = flight->aircraft()->category();
        if (category == Aircraft::Category::Heavy)
        {
            return WakeClass::Heavy;
        }
        if (category == Aircraft::Category::LightProp ||
            category == Aircraft::Category::Prop ||
            category == Aircraft::Category::Helicopter)
        {
            return WakeClass::Light;
        }

        return WakeClass::Medium;
    }

    WakeTurbulenceCalculator::SeparationProfile WakeTurbulenceCalculator::getSeparationProfile(shared_ptr<Flight> flight) const
    {
        SeparationProfile profile;
        profile.wakeClass = inferWakeClass(flight);
        profile.rotorcraft = flight && flight->aircraft() &&
            flight->aircraft()->category() == Aircraft::Category::Helicopter;

        switch (profile.wakeClass)
        {
        case WakeClass::Super:
            profile.referenceArrivalSpeedKt = 165.0f;
            profile.departureRollSeconds = 55.0f;
            profile.lineupSeconds = 14.0f;
            profile.crossingSeconds = 26.0f;
            break;
        case WakeClass::Heavy:
            profile.referenceArrivalSpeedKt = 155.0f;
            profile.departureRollSeconds = 45.0f;
            profile.lineupSeconds = 12.0f;
            profile.crossingSeconds = 24.0f;
            break;
        case WakeClass::Light:
            profile.referenceArrivalSpeedKt = 100.0f;
            profile.departureRollSeconds = 22.0f;
            profile.lineupSeconds = 6.0f;
            profile.crossingSeconds = 14.0f;
            break;
        case WakeClass::Medium:
        default:
            profile.referenceArrivalSpeedKt = 145.0f;
            profile.departureRollSeconds = 32.0f;
            profile.lineupSeconds = 8.0f;
            profile.crossingSeconds = 18.0f;
            break;
        }

        if (profile.rotorcraft)
        {
            profile.referenceArrivalSpeedKt = 70.0f;
            profile.departureRollSeconds = 10.0f;
            profile.lineupSeconds = 4.0f;
            profile.crossingSeconds = 10.0f;
        }

        if (flight && flight->aircraft())
        {
            const auto actualGroundSpeed = static_cast<float>(flight->aircraft()->groundSpeedKt());
            if (actualGroundSpeed > 60.0f && !flight->aircraft()->altitude().isGround())
            {
                profile.referenceArrivalSpeedKt = actualGroundSpeed;
            }
        }

        return profile;
    }

    int WakeTurbulenceCalculator::getWakeTurbulenceSeparationSeconds(WakeClass leader, WakeClass follower) const
    {
        // Time-based wake turbulence separation (ICAO Doc 4444 Table 8-1)
        // Heavy: 2 minutes, Large: 2 minutes, Small: 2 minutes, following Heavy: 2 minutes
        // All time-based separations are 2 minutes (120 seconds)
        return 120;
    }

    float WakeTurbulenceCalculator::requiredTakeoffGapSeconds(shared_ptr<Flight> departure, shared_ptr<Flight> arrival) const
    {
        if (!departure || !arrival)
        {
            return 0.0f;
        }

        const auto departureProfile = getSeparationProfile(departure);
        const auto arrivalProfile = getSeparationProfile(arrival);
        
        // For time-based separation, use 2 minutes
        const float wakeSeconds = departureProfile.rotorcraft
            ? 0.0f
            : 120.0f; // 2 minutes time-based
        
        return departureProfile.departureRollSeconds + wakeSeconds + 5.0f;
    }

    float WakeTurbulenceCalculator::requiredLuawGapSeconds(shared_ptr<Flight> departure, shared_ptr<Flight> arrival) const
    {
        if (!departure)
        {
            return 0.0f;
        }

        const auto departureProfile = getSeparationProfile(departure);
        return departureProfile.lineupSeconds + requiredTakeoffGapSeconds(departure, arrival);
    }

    float WakeTurbulenceCalculator::requiredCrossingGapSeconds(shared_ptr<Flight> crossing, shared_ptr<Flight> arrival) const
    {
        if (!crossing || !arrival)
        {
            return 0.0f;
        }

        const auto crossingProfile = getSeparationProfile(crossing);
        const auto arrivalProfile = getSeparationProfile(arrival);
        
        // For time-based separation, use 2 minutes
        const float wakeSeconds = crossingProfile.rotorcraft
            ? 0.0f
            : 120.0f; // 2 minutes time-based
        
        return crossingProfile.crossingSeconds + wakeSeconds + 5.0f;
    }
}