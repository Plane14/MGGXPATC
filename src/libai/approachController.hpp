//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once

#include <unordered_set>
#include <chrono>

#include "libworld.h"
#include "clearanceFactory.hpp"
#include "intentTypes.hpp"
#include "intentFactory.hpp"
#include "aiControllerBase.hpp"
#include "libai.hpp"

using namespace std;
using namespace world;

namespace ai
{
    class ApproachController : public AIControllerBase
    {
    private:
        // Flights that have been issued an approach clearance by this controller
        unordered_set<int> m_approachClearanceIssued;
        // Last progress timestamp to throttle approach-clearance checks
        chrono::microseconds m_lastProgressTimestamp = chrono::microseconds(0);
        static constexpr int64_t PROGRESS_INTERVAL_US = 2000000LL; // 2 seconds

    public:
        ApproachController(shared_ptr<HostServices> _host, int _id, Actor::Gender _gender, shared_ptr<ControllerPosition> _position) :
            AIControllerBase(_host, _id, _gender, _position)
        {
        }

        void progressTo(chrono::microseconds timestamp) override
        {
            // Run base-class handoff logic first (Approach -> Tower)
            AIControllerBase::progressTo(timestamp);

            // Throttle to avoid spamming
            if ((timestamp - m_lastProgressTimestamp).count() < PROGRESS_INTERVAL_US)
            {
                return;
            }
            m_lastProgressTimestamp = timestamp;

            // Monitor radar-tracked arrival flights and issue approach clearances
            // when they are within the approach sector but not yet cleared.
            for (const auto& flight : m_radarTrackedFlights)
            {
                if (!flight || !flight->aircraft())
                {
                    continue;
                }

                // Only manage arrival-phase flights
                if (flight->phase() != Flight::Phase::Arrival)
                {
                    continue;
                }

                // Skip if approach clearance already issued by us
                if (hasKey(m_approachClearanceIssued, flight->id()))
                {
                    continue;
                }

                // Skip if aircraft already has an approach clearance in its flight record
                if (flight->tryFindClearance<ApproachClearance>(Clearance::Type::ApproachClearance))
                {
                    m_approachClearanceIssued.insert(flight->id());
                    continue;
                }

                // Check if aircraft is within approach issue range (roughly 20-30 NM from airport)
                try
                {
                    auto airport = host()->getWorld()->getAirport(flight->plan()->arrivalAirportIcao());
                    if (!airport)
                    {
                        continue;
                    }

                    const GeoPoint aircraftLoc = flight->aircraft()->location();
                    const double distanceMeters = GeoMath::getDistanceMeters(
                        aircraftLoc, airport->header().datum());
                    const double distanceNm = distanceMeters / METERS_IN_1_NAUTICAL_MILE;

                    // Issue approach clearance when within ~25 NM but not yet on final
                    if (distanceNm > 5.0 && distanceNm < 28.0)
                    {
                        const string& runwayName = flight->plan()->arrivalRunway();
                        if (runwayName.empty())
                        {
                            continue;
                        }

                        auto approachType = inferApproachType(flight);
                        float altitudeFloor = 3000.0f;
                        if (approachType == ApproachClearance::ApproachType::Visual)
                        {
                            altitudeFloor = 0.0f;
                        }
                        else if (approachType == ApproachClearance::ApproachType::RNP ||
                                 approachType == ApproachClearance::ApproachType::RNAV)
                        {
                            altitudeFloor = 2500.0f;
                        }

                        auto clearance = C.approachClearance(
                            flight,
                            runwayName,
                            approachType,
                            altitudeFloor,
                            false);

                        if (clearance)
                        {
                            vector<TrafficAdvisory> traffic;
                            transmit(I.towerClearedForApproach(clearance, traffic, 0));
                            m_approachClearanceIssued.insert(flight->id());
                            host()->writeLog(
                                "AICONT|APP issued approach clearance to flight[%s] runway[%s] type[%s] floor[%.0f]",
                                flight->callSign().c_str(),
                                runwayName.c_str(),
                                clearance->approachTypeString().c_str(),
                                altitudeFloor);
                        }
                    }
                }
                catch (const exception& e)
                {
                    host()->writeLog(
                        "AICONT|APP WARNING: exception while managing flight[%s]: %s",
                        flight->callSign().c_str(),
                        e.what());
                }
            }

            // Clean up entries for flights that have left our radar tracking
            vector<int> toErase;
            for (int flightId : m_approachClearanceIssued)
            {
                bool stillTracked = false;
                for (const auto& f : m_radarTrackedFlights)
                {
                    if (f && f->id() == flightId)
                    {
                        stillTracked = true;
                        break;
                    }
                }
                if (!stillTracked)
                {
                    toErase.push_back(flightId);
                }
            }
            for (int id : toErase)
            {
                m_approachClearanceIssued.erase(id);
            }
        }

    private:
        static ApproachClearance::ApproachType inferApproachType(shared_ptr<Flight> flight)
        {
            const string& name = flight->plan()->approachName();
            string upper = name;
            transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) { return static_cast<char>(toupper(c)); });

            if (upper.find("ILS") != string::npos)   return ApproachClearance::ApproachType::ILS;
            if (upper.find("VIS") != string::npos)    return ApproachClearance::ApproachType::Visual;
            if (upper.find("RNAV") != string::npos)   return ApproachClearance::ApproachType::RNAV;
            if (upper.find("RNP") != string::npos)    return ApproachClearance::ApproachType::RNP;
            if (upper.find("VOR") != string::npos)    return ApproachClearance::ApproachType::VOR;
            if (upper.find("NDB") != string::npos)    return ApproachClearance::ApproachType::NDB;
            if (upper.find("GPS") != string::npos)    return ApproachClearance::ApproachType::GPS;
            return ApproachClearance::ApproachType::ILS;
        }
    };
}
