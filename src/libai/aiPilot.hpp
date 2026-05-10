// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once

#include <chrono>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <random>
#include <atomic>
#include "libworld.h"
#include "worldHelper.hpp"
#include "basicManeuverTypes.hpp"
#include "maneuverFactory.hpp"
#include "intentTypes.hpp"
#include "intentFactory.hpp"
#include "aiAircraft.hpp"
#include "libai.hpp"

using namespace std;
using namespace world;

namespace ai
{
    class AIPilot : public Pilot
    {
    private:
        WorldHelper m_helper;
        shared_ptr<ManeuverFactory> m_maneuverFactory;
        shared_ptr<IntentFactory> m_intentFactory;
        ManeuverFactory& M;
        IntentFactory& I;
        shared_ptr<Airport> m_departureAirport;
        shared_ptr<FlightPlan> m_flightPlan;
        shared_ptr<AIAircraft> m_aircraft;
        int m_departureTowerKhz = 0;
        int m_departureKhz = 0;
        int m_arrivalGroundKhz = 0;
        uint64_t m_lastReceivedIntentId = 0;
        shared_ptr<ControllerPosition> m_lastReceivedControl;
        DeclineReason m_lastDeclineReason = DeclineReason::None;
        bool m_wasTakeoffClearanceReadBack = false;
        bool m_continueApproach = false;
        int m_departureNumberInLine = 0;
        bool m_prepareForImmediateTakeoff = false;
        bool m_holdShortForDeparture = false;
        chrono::microseconds m_linedUpTimestamp = chrono::microseconds(0);
        chrono::microseconds m_finalReportedTimestamp = chrono::microseconds(0);
        bool m_arrivalTowerCheckInDone = false;
        bool m_departureRadarCheckInDone = false;
        bool m_arrivalRadarCheckInDone = false;
        bool m_stoppedBeforeTakeoff = false;
        bool m_doUnrestrictedClimbout = false;
    public:
        AIPilot(
            shared_ptr<HostServices> _host, 
            int _id, 
            Actor::Gender _gender, 
            shared_ptr<Flight> _flight, 
            shared_ptr<ManeuverFactory> _maneuverFactory,
            shared_ptr<IntentFactory> _intentFactory
        ) : Pilot(_host, _id, _gender, _flight),
            m_helper(_host),
            m_maneuverFactory(_maneuverFactory),
            M(*_maneuverFactory),
            m_intentFactory(_intentFactory),
            I(*_intentFactory),
            m_aircraft(dynamic_pointer_cast<AIAircraft>(_flight->aircraft()))
        {
            //_host->writeLog("AIPilot::AIPilot() - enter");

            m_flightPlan = _flight->plan();
            m_departureAirport = _host->getWorld()->getAirport(_flight->plan()->departureAirportIcao());

            aircraft()->onCommTransmission([this](shared_ptr<Intent> intent) {
                handleCommTransmission(intent);
            });

            //_host->writeLog("AIPilot::AIPilot() - exit");
        }
    public:
        shared_ptr<Maneuver> getFlightCycle() override
        {
            return maneuverFlightCycle();
        }
        shared_ptr<Maneuver> getFinalToGate(const Runway::End& landingRunway) override
        {
            return maneuverFinalToGate(landingRunway);
        }
        shared_ptr<Maneuver> getHelipadFinalToGate(shared_ptr<ParkingStand> landingStand)
        {
            return maneuverHelipadArrivalToGate(landingStand);
        }
        void progressTo(chrono::microseconds timestamp) override 
        {
            //TODO
        }
        string getStatusString() const override
        {
            return "<twrkhz=" + to_string(m_departureTowerKhz) + ">";
        }
    private:
        void scheduleRadarCheckInAfterHandoff(shared_ptr<ControllerPosition> targetControl, const string& reasonTag)
        {
            if (!targetControl)
            {
                return;
            }

            const string suffix = to_string(flight()->id()) + "_" + reasonTag;
            host()->getWorld()->deferBy("pilot_handoff_tune_" + suffix, chrono::seconds(1), [this, targetControl]() {
                aircraft()->setFrequency(targetControl->frequency());
            });

            if (targetControl->type() == ControllerPosition::Type::Local ||
                targetControl->type() == ControllerPosition::Type::Ground)
            {
                return;
            }

            host()->getWorld()->deferBy("pilot_handoff_checkin_" + suffix, chrono::seconds(2), [this, targetControl]() {
                auto checkIn = I.pilotCheckInWithRadar(flight(), targetControl);
                if (checkIn)
                {
                    targetControl->frequency()->enqueuePushToTalk(chrono::milliseconds(250), checkIn);
                }
            });
        }

        void handleCommTransmission(shared_ptr<Intent> intent)
        {
            if (intent->direction() == Intent::Direction::ControllerToPilot && intent->subjectFlight() == flight())
            {
                host()->writeLog("TRANSMISSION HANDLED BY PILOT [%s]", flight()->callSign().c_str(), intent->code());
                shared_ptr<Clearance> newClearance;
                m_lastReceivedIntentId = intent->id();
                m_lastReceivedControl = intent->subjectControl();

                switch (intent->code())
                {
                case DeliveryIfrClearanceReplyIntent::IntentCode:
                    newClearance = dynamic_pointer_cast<DeliveryIfrClearanceReplyIntent>(intent)->clearance();
                    break;
                case DeliveryIfrClearanceReadbackCorrectIntent::IntentCode:
                    {
                        auto readbackCorrect = dynamic_pointer_cast<DeliveryIfrClearanceReadbackCorrectIntent>(intent);
                        auto clearance = readbackCorrect->clearance();
                        // auto delivery = clearance->header().issuedBy;
                        // auto intentFactory = host()->services().get<IntentFactory>();
                        // auto handoffReadback = intentFactory->pilotHandoffReadback(flight(), delivery, readbackCorrect->groundKhz());
                        //delivery->frequency()->enqueueTransmission(handoffReadback);
                        clearance->setReadbackCorrect();
                    }
                    break;
                case GroundPushAndStartReplyIntent::IntentCode:
                    newClearance = dynamic_pointer_cast<GroundPushAndStartReplyIntent>(intent)->approval();
                    break;
                case GroundDepartureTaxiReplyIntent::IntentCode:
                    newClearance = dynamic_pointer_cast<GroundDepartureTaxiReplyIntent>(intent)->clearance();
                    break;
                case GroundHoldShortRunwayIntent::IntentCode:
                    m_lastDeclineReason = dynamic_pointer_cast<GroundHoldShortRunwayIntent>(intent)->reason();
                    break;
                case GroundRunwayCrossClearanceIntent::IntentCode:
                    newClearance = dynamic_pointer_cast<GroundRunwayCrossClearanceIntent>(intent)->clearance();
                    break;
                case GroundSwitchToTowerIntent::IntentCode:
                    m_departureTowerKhz = dynamic_pointer_cast<GroundSwitchToTowerIntent>(intent)->towerKhz();
                    intent->subjectControl()->frequency()->enqueuePushToTalk(
                        chrono::milliseconds(150),
                        I.pilotHandoffReadback(flight(), intent->subjectControl(), m_departureTowerKhz, intent->id()));
                    break;
                case ControllerHandoffIntent::IntentCode:
                    {
                        auto handoff = dynamic_pointer_cast<ControllerHandoffIntent>(intent);
                        const auto targetControl = handoff ? handoff->targetControl() : nullptr;
                        const int newFrequencyKhz = handoff ? handoff->newFrequencyKhz() : 0;
                        intent->subjectControl()->frequency()->enqueuePushToTalk(
                            chrono::milliseconds(150),
                            I.pilotHandoffReadback(flight(), intent->subjectControl(), newFrequencyKhz, intent->id()));
                        scheduleRadarCheckInAfterHandoff(targetControl, "generic");
                    }
                    break;
                case TowerDepartureCheckInReplyIntent::IntentCode:
                    m_departureNumberInLine = dynamic_pointer_cast<TowerDepartureCheckInReplyIntent>(intent)->numberInLine();
                    m_prepareForImmediateTakeoff = dynamic_pointer_cast<TowerDepartureCheckInReplyIntent>(intent)->prepareForImmediateTakeoff();
                    break;
                case TowerDepartureHoldShortIntent::IntentCode:
                    m_holdShortForDeparture = true;
                    m_lastDeclineReason = dynamic_pointer_cast<TowerDepartureHoldShortIntent>(intent)->reason();
                    break;
                case TowerLineUpAndWaitIntent::IntentCode:
                    newClearance = dynamic_pointer_cast<TowerLineUpAndWaitIntent>(intent)->approval();
                    m_holdShortForDeparture = false;
                    break;
                case TowerClearedForTakeoffIntent::IntentCode:
                    {
                        auto takeOffClearance = dynamic_pointer_cast<TowerClearedForTakeoffIntent>(intent)->clearance();
                        m_departureKhz = takeOffClearance->departureKhz();
                        newClearance = takeOffClearance;
                        m_holdShortForDeparture = false;
                    }
                    break;
                case TowerContinueApproachIntent::IntentCode:
                    m_continueApproach = true;
                    break;
                case TowerClearedForLandingIntent::IntentCode:
                    {
                        auto landingClearance = dynamic_pointer_cast<TowerClearedForLandingIntent>(intent)->clearance();
                        m_arrivalGroundKhz = landingClearance->groundKhz();
                        flight()->setArrivalRunway(landingClearance->runway());
                        m_continueApproach = false;
                        m_finalReportedTimestamp = chrono::microseconds(0);
                        m_arrivalTowerCheckInDone = false;
                        newClearance = landingClearance;
                    }
                    break;
                case TowerGoAroundIntent::IntentCode:
                    flight()->removeClearance(Clearance::Type::LandingClearance);
                    m_continueApproach = false;
                    m_finalReportedTimestamp = chrono::microseconds(0);
                    m_arrivalTowerCheckInDone = false;
                    newClearance = dynamic_pointer_cast<TowerGoAroundIntent>(intent)->request();
                    break;
                case ControllerRadarContactIntent::IntentCode:
                    break;
                case GroundArrivalTaxiReplyIntent::IntentCode:
                    newClearance = dynamic_pointer_cast<GroundArrivalTaxiReplyIntent>(intent)->clearance();
                    break;
                }

                if (newClearance)
                {
                    flight()->addClearance(newClearance);
                }
            }
        }

        static string normalizeWaypointName(string value)
        {
            value.erase(remove_if(value.begin(), value.end(), [](unsigned char c) {
                return !isalnum(c);
            }), value.end());

            transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
                return static_cast<char>(toupper(c));
            });

            return value;
        }

        static bool tryFindWaypointLocation(
            const shared_ptr<FlightPlan>& plan,
            const string& waypointName,
            GeoPoint& location,
            const GeoPoint& referencePoint = GeoPoint::empty)
        {
            if (!plan || waypointName.empty())
            {
                return false;
            }

            const string normalizedWaypointName = normalizeWaypointName(waypointName);
            bool found = false;
            double bestDistanceMeters = numeric_limits<double>::max();
            GeoPoint bestLocation = GeoPoint::empty;

            for (const auto& routeWaypoint : plan->knownWaypoints())
            {
                if (routeWaypoint.location == GeoPoint::empty)
                {
                    continue;
                }

                if (normalizeWaypointName(routeWaypoint.name) == normalizedWaypointName)
                {
                    if (referencePoint == GeoPoint::empty)
                    {
                        location = routeWaypoint.location;
                        return true;
                    }

                    const double candidateDistanceMeters = GeoMath::getDistanceMeters(referencePoint, routeWaypoint.location);
                    if (!found || candidateDistanceMeters < bestDistanceMeters)
                    {
                        found = true;
                        bestDistanceMeters = candidateDistanceMeters;
                        bestLocation = routeWaypoint.location;
                    }
                }
            }

            if (found)
            {
                location = bestLocation;
                return true;
            }

            return false;
        }

        int procedureLegDurationSeconds(FlightPlan::LegType legType) const
        {
            // Scale fallback durations by approach speed (proxy for aircraft speed class).
            const float approachKt = max(80.0f, m_aircraft->performanceProfile().approachSpeedKt);
            const float speedScale = 145.0f / approachKt;
            switch (legType)
            {
            case FlightPlan::LegType::TakeOff:
                return max(15, static_cast<int>(30.0f * speedScale));
            case FlightPlan::LegType::Sid:
                return max(40, static_cast<int>(75.0f * speedScale));
            case FlightPlan::LegType::EnRoute:
                return max(60, static_cast<int>(120.0f * speedScale));
            case FlightPlan::LegType::Star:
                return max(45, static_cast<int>(90.0f * speedScale));
            case FlightPlan::LegType::Approach:
                return max(30, static_cast<int>(60.0f * speedScale));
            case FlightPlan::LegType::Landing:
                return max(15, static_cast<int>(30.0f * speedScale));
            case FlightPlan::LegType::GoAround:
                return max(20, static_cast<int>(45.0f * speedScale));
            default:
                return max(10, static_cast<int>(20.0f * speedScale));
            }
        }

        int procedureLegDurationSeconds(
            FlightPlan::LegType legType,
            const shared_ptr<FlightPlan::Leg>& leg,
            const GeoPoint& fromPoint,
            const GeoPoint& toPoint) const
        {
            const float targetGroundSpeed = max(1.0f, procedureLegGroundSpeed(legType, leg));
            const double speedMetersPerSecond = targetGroundSpeed * METERS_IN_1_NAUTICAL_MILE / 3600.0;
            if (speedMetersPerSecond <= 0.0)
            {
                return procedureLegDurationSeconds(legType);
            }

            const double distanceMeters = GeoMath::getDistanceMeters(fromPoint, toPoint);
            const int estimatedSeconds = static_cast<int>(distanceMeters / speedMetersPerSecond + 0.5);
            return max(10, estimatedSeconds);
        }

        float procedureLegVerticalSpeed(FlightPlan::LegType legType, const shared_ptr<FlightPlan::Leg>& leg) const
        {
            const auto& performance = m_aircraft->performanceProfile();
            const bool helicopter = m_aircraft->category() == Aircraft::Category::Helicopter;

            switch (legType)
            {
            case FlightPlan::LegType::TakeOff:
                if (helicopter)
                {
                    return max(450.0f, performance.initialClimbRocFpm * 0.35f);
                }
                return 1500.0f;
            case FlightPlan::LegType::Sid:
                {
                    if (helicopter)
                    {
                        return max(650.0f, performance.initialClimbRocFpm * 0.5f);
                    }
                    float baseClimbRate = max(800.0f, leg->targetAltitude() > 0 ? leg->targetAltitude() / 10.0f : 1500.0f);
                    // Fighters with unrestricted climbout: much steeper climb (4500-6000 fpm)
                    if (m_doUnrestrictedClimbout)
                    {
                        float unrestrictedRate = max(4500.0f, min(6000.0f, performance.descentRateFpm * 3.0f));
                        return unrestrictedRate;
                    }
                    return baseClimbRate;
                }
            case FlightPlan::LegType::EnRoute:
                return 0.0f;
            case FlightPlan::LegType::Star:
                if (helicopter)
                {
                    return -max(450.0f, performance.descentRateFpm * 1.2f);
                }
                return -max(700.0f, leg->targetAltitude() > 0 ? leg->targetAltitude() / 10.0f : 1200.0f);
            case FlightPlan::LegType::Approach:
                {
                    // Use Eurocontrol performance-based descent rate for approach
                    // Base on aircraft's nominal descent rate capability with minimum for steep approach
                    const float baseDescentRate = performance.descentRateFpm;
                    const float minApproachDescent = helicopter ? 250.0f : 700.0f;
                    const float performanceBasedDescent = leg->targetAltitude() > 0
                        ? max(minApproachDescent, baseDescentRate * 0.8f)
                        : max(minApproachDescent, baseDescentRate);
                    return -performanceBasedDescent;
                }
            case FlightPlan::LegType::Landing:
                {
                    if (helicopter)
                    {
                        return -max(300.0f, min(600.0f, performance.descentRateFpm * 0.55f));
                    }
                    return -max(650.0f, min(900.0f, performance.descentRateFpm * 0.45f));
                }
            case FlightPlan::LegType::GoAround:
                {
                    if (helicopter)
                    {
                        return max(800.0f, performance.initialClimbRocFpm * 0.4f);
                    }
                    const bool fighter = m_aircraft->category() == Aircraft::Category::Fighter;
                    if (fighter)
                    {
                        return max(3000.0f, performance.initialClimbRocFpm * 0.5f);
                    }
                    return max(1200.0f, min(2500.0f, performance.initialClimbRocFpm * 0.45f));
                }
            default:
                return 0.0f;
            }
        }

        float procedureLegGroundSpeed(FlightPlan::LegType legType, const shared_ptr<FlightPlan::Leg>& leg) const
        {
            const auto& performance = m_aircraft->performanceProfile();
            const auto constrainedSpeed = [leg](float defaultSpeed) {
                return leg->targetSpeed() > 0.0f
                    ? min(defaultSpeed, leg->targetSpeed())
                    : defaultSpeed;
            };

            const bool helicopter = m_aircraft->category() == Aircraft::Category::Helicopter;
            const float approachSpeed = helicopter ? max(45.0f, performance.approachSpeedKt) : max(90.0f, performance.approachSpeedKt);

            if (helicopter)
            {
                switch (legType)
                {
                case FlightPlan::LegType::TakeOff:
                    return constrainedSpeed(max(35.0f, performance.takeoffInitialClimbSpeedKt * 0.75f));
                case FlightPlan::LegType::Sid:
                    return constrainedSpeed(max(55.0f, min(105.0f, performance.takeoffInitialClimbSpeedKt)));
                case FlightPlan::LegType::EnRoute:
                    return constrainedSpeed(max(80.0f, min(135.0f, approachSpeed + 45.0f)));
                case FlightPlan::LegType::Star:
                    return constrainedSpeed(max(65.0f, min(115.0f, approachSpeed + 25.0f)));
                case FlightPlan::LegType::Approach:
                    return constrainedSpeed(approachSpeed);
                case FlightPlan::LegType::Landing:
                    return constrainedSpeed(max(10.0f, min(45.0f, approachSpeed * 0.55f)));
                case FlightPlan::LegType::GoAround:
                    return constrainedSpeed(max(55.0f, performance.takeoffInitialClimbSpeedKt));
                default:
                    return leg->targetSpeed();
                }
            }

            switch (legType)
            {
            case FlightPlan::LegType::TakeOff:
                return max(120.0f, leg->targetSpeed() > 0.0f ? leg->targetSpeed() : approachSpeed + 20.0f);
            case FlightPlan::LegType::Sid:
                return constrainedSpeed(max(150.0f, min(230.0f, approachSpeed + 50.0f)));
            case FlightPlan::LegType::EnRoute:
                return constrainedSpeed(max(200.0f, min(280.0f, approachSpeed + 100.0f)));
            case FlightPlan::LegType::Star:
                return constrainedSpeed(max(130.0f, min(210.0f, approachSpeed + 30.0f)));
            case FlightPlan::LegType::Approach:
                return constrainedSpeed(approachSpeed);
            case FlightPlan::LegType::Landing:
                return constrainedSpeed(max(70.0f, approachSpeed - 20.0f));
            case FlightPlan::LegType::GoAround:
                return max(140.0f, leg->targetSpeed() > 0.0f ? leg->targetSpeed() : approachSpeed + 20.0f);
            default:
                return leg->targetSpeed();
            }
        }

        bool hasProcedureLeg(FlightPlan::LegType legType) const
        {
            const auto& legs = m_flightPlan->legs();
            return any_of(legs.begin(), legs.end(), [legType](const shared_ptr<FlightPlan::Leg>& leg) {
                return leg && leg->type() == legType;
            });
        }

        int firstProcedureLegIndex(FlightPlan::LegType legType) const
        {
            const auto& legs = m_flightPlan->legs();
            for (int i = 0; i < static_cast<int>(legs.size()); ++i)
            {
                if (legs[i] && legs[i]->type() == legType)
                {
                    return i;
                }
            }

            return -1;
        }

        shared_ptr<Maneuver> maneuverProcedureLeg(FlightPlan::LegType legType, Flight::Phase phase, const string& stepId)
        {
            auto cursor = flight()->planCursor();
            if (!cursor)
            {
                return M.instantAction([]{});
            }

            return DeferredManeuver::create(Maneuver::Type::Flight, stepId, [this, cursor, legType, phase, stepId]() {
                if (!cursor->activateNextLegOfType(legType))
                {
                    host()->writeLog(
                        "AIPILO|procedure leg step[%s] skipped: flight[%s] has no remaining leg of type[%d]",
                        stepId.c_str(),
                        flight()->callSign().c_str(),
                        static_cast<int>(legType));
                    return M.instantAction([]{});
                }

                vector<shared_ptr<Maneuver>> steps;
                do
                {
                    auto leg = cursor->activeLeg();
                    if (!leg || leg->type() != legType)
                    {
                        break;
                    }

                    const int legIndex = cursor->activeLegIndex();
                    const float targetGroundSpeed = procedureLegGroundSpeed(legType, leg);
                    const float targetVerticalSpeed = procedureLegVerticalSpeed(legType, leg);
                    GeoPoint fromPoint;
                    GeoPoint toPoint;
                    const GeoPoint referencePoint = m_aircraft->location();
                    const bool hasFromPoint = tryFindWaypointLocation(m_flightPlan, leg->fromNavaid(), fromPoint, referencePoint);
                    const bool hasToPoint = tryFindWaypointLocation(
                        m_flightPlan,
                        leg->toNavaid(),
                        toPoint,
                        hasFromPoint ? fromPoint : referencePoint);
                    const bool hasGeometricLeg = hasFromPoint && hasToPoint;
                    // Use aircraft's current location for initial heading calculation
                    // This ensures smooth transitions between leg types (e.g., STAR->Approach)
                    // where the aircraft may not be exactly at the leg's from-point
                    const float targetHeading = hasGeometricLeg
                        ? GeoMath::getHeadingFromPoints(m_aircraft->location(), toPoint)
                        : m_aircraft->attitude().heading();
                    const int legDurationSeconds = hasGeometricLeg
                        ? procedureLegDurationSeconds(legType, leg, fromPoint, toPoint)
                        : procedureLegDurationSeconds(legType);
                    const string legStepId = stepId + "_" + to_string(legIndex);

                    // Capture leg-specific data by value to avoid loop variable issues
                    const shared_ptr<FlightPlan::Leg> currentLeg = leg;
                    const GeoPoint currentFromPoint = fromPoint;
                    const GeoPoint currentToPoint = toPoint;
                    const bool currentHasGeometricLeg = hasGeometricLeg;
                    const float currentTargetGroundSpeed = targetGroundSpeed;
                    const float currentTargetVerticalSpeed = targetVerticalSpeed;
                    const float currentTargetHeading = targetHeading;
                    const int currentLegDurationSeconds = legDurationSeconds;
                    const string currentLegStepId = legStepId;
                    
                    // Detailed logging for arrival procedure legs
                    if (legType == FlightPlan::LegType::Star || legType == FlightPlan::LegType::Approach)
                    {
                        host()->writeLog(
                            "AIPILO|ARR_PROC flight[%s] leg[%d] type[%s] from[%s|%.4f,%.4f] to[%s|%.4f,%.4f] hasGeo[%d]",
                            flight()->callSign().c_str(),
                            legIndex,
                            (legType == FlightPlan::LegType::Star ? "STAR" : "APPROACH"),
                            leg->fromNavaid().c_str(),
                            hasFromPoint ? fromPoint.latitude : 0.0,
                            hasFromPoint ? fromPoint.longitude : 0.0,
                            leg->toNavaid().c_str(),
                            hasToPoint ? toPoint.latitude : 0.0,
                            hasToPoint ? toPoint.longitude : 0.0,
                            (int)hasGeometricLeg);
                    }

                    // Parallel maneuver: track toward waypoint while waiting to reach it
                    vector<shared_ptr<Maneuver>> legSteps;

                    // 1. Initial setup - set speed and initial heading
                    legSteps.push_back(M.instantAction([this, cursor, currentLeg, legType, phase, currentTargetGroundSpeed, currentTargetVerticalSpeed, currentToPoint, currentHasGeometricLeg, currentLegStepId]() {
                        host()->writeLog(
                            "AIPILO|procedure leg step[%s] flight[%s] legIndex[%d] type[%d] from[%s] to[%s] targetAlt[%f] targetSpd[%f] commandedSpd[%f]",
                            currentLegStepId.c_str(),
                            flight()->callSign().c_str(),
                            cursor->activeLegIndex(),
                            static_cast<int>(legType),
                            currentLeg->fromNavaid().c_str(),
                            currentLeg->toNavaid().c_str(),
                            currentLeg->targetAltitude(),
                            currentLeg->targetSpeed(),
                            currentTargetGroundSpeed);

                        flight()->setPhase(phase);
                        if (currentTargetGroundSpeed > 0)
                        {
                            m_aircraft->setGroundSpeedKt(currentTargetGroundSpeed);
                        }
                        if (currentTargetVerticalSpeed != 0)
                        {
                            m_aircraft->setVerticalSpeedFpm(currentTargetVerticalSpeed);
                        }
                        if (currentHasGeometricLeg)
                        {
                            // Calculate heading at execution time, not setup time
                            // Aircraft position may have changed since maneuver was created
                            float initialHeading = GeoMath::getHeadingFromPoints(
                                m_aircraft->location(), currentToPoint);
                            m_aircraft->setAttitude(m_aircraft->attitude().withHeading(initialHeading));
                        }
                    }));

                    // 2. Continuously track toward the target waypoint with heading updates
                    if (currentHasGeometricLeg)
                    {
                        // Create a maneuver that continuously updates heading toward target
                        // and waits until the waypoint is reached
                        // Use a repeating instant action sequence instead of AnimationManeuver
                        // because AnimationManeuver has a fixed duration and stops updating
                        const GeoPoint legStartPoint = currentFromPoint;
                        const GeoPoint targetPoint = currentToPoint;
                        const string targetName = currentLeg->toNavaid();
                        
                        const float legTargetAltitude = currentLeg->targetAltitude();
                        const float legTargetSpeed = currentLeg->targetSpeed();
                        const float legCommandedSpeed = currentTargetGroundSpeed;

                        legSteps.push_back(M.await(Maneuver::Type::Unspecified, "track_waypoint",
                            [this, legStartPoint, targetPoint, targetName, legTargetAltitude, legTargetSpeed, legCommandedSpeed]() {
                                bool reached = hasReachedWaypoint(targetPoint, legStartPoint);
                                if (reached)
                                {
                                    host()->writeLog(
                                        "AIPILO|Flight[%s] reached waypoint[%s]",
                                        flight()->callSign().c_str(),
                                        targetName.c_str());
                                    return true;
                                }

                                // Calculate heading to target waypoint while enroute.
                                float headingToTarget = GeoMath::getHeadingFromPoints(
                                    m_aircraft->location(), targetPoint);
                                float currentHeading = m_aircraft->attitude().heading();
                                float turnDegrees = GeoMath::getTurnDegrees(
                                    currentHeading, headingToTarget);
                                float headingDiff = fabs(turnDegrees);

                                // Log significant heading changes for debugging.
                                if (headingDiff > 45.0f)
                                {
                                    host()->writeLog(
                                        "AIPILO|Flight[%s] large heading change to waypoint[%s]: "
                                        "current=%.1f target=%.1f turn=%.1f (possible overshoot)",
                                        flight()->callSign().c_str(),
                                        targetName.c_str(),
                                        currentHeading,
                                        headingToTarget,
                                        turnDegrees);
                                }

                                // Only update if off by more than 1.5 degrees (tighter dead-band reduces oscillation).
                                if (headingDiff > 1.5f)
                                {
                                    // Speed-dependent banked turn model.
                                    // Standard-rate turn: bank = atan(V / (g * R)), approx bank = V_kt / 10 + 7.
                                    // Limit per-tick heading delta by speed-appropriate turn rate.
                                    const float speedKt = static_cast<float>(std::max(80.0, m_aircraft->groundSpeedKt()));
                                    const float maxBank = isFighter()
                                        ? std::min(45.0f, speedKt / 8.0f + 10.0f)
                                        : (isHelicopter()
                                            ? std::min(20.0f, speedKt / 12.0f + 5.0f)
                                            : std::min(30.0f, speedKt / 10.0f + 7.0f));
                                    // Turn rate (deg/s) from bank: rate = g*tan(bank)/V, approx 1091*tan(bank)/V_kt.
                                    const float bankRad = maxBank * 3.14159265f / 180.0f;
                                    const float turnRateDegPerSec = std::max(1.5f, 1091.0f * std::tan(bankRad) / speedKt);
                                    // Proportional control: reduce commanded turn rate near rollout.
                                    const float proportionalRate = std::min(turnRateDegPerSec, headingDiff * 0.4f);
                                    const float limitedTurn = std::max(-proportionalRate, std::min(proportionalRate, turnDegrees));
                                    float newHeading = GeoMath::addTurnToHeading(currentHeading, limitedTurn);
                                    const float targetBank = std::max(5.0f, std::min(maxBank, std::abs(limitedTurn) / turnRateDegPerSec * maxBank));
                                    const float bankSign = (limitedTurn >= 0.0f ? 1.0f : -1.0f);
                                    const float roll = targetBank * bankSign;
                                    m_aircraft->setAttitude(
                                        m_aircraft->attitude().withHeading(newHeading).withRoll(roll));
                                }
                                else if (std::abs(m_aircraft->attitude().roll()) > 0.5)
                                {
                                    // Smoothly unwind residual bank with speed-scaled recovery rate.
                                    const float currentRoll = static_cast<float>(m_aircraft->attitude().roll());
                                    const float rollRate = std::max(2.0f, std::min(5.0f, static_cast<float>(m_aircraft->groundSpeedKt()) / 60.0f + 2.0f));
                                    const float unwindStep = (currentRoll > 0.0f ? -rollRate : rollRate);
                                    float nextRoll = currentRoll + unwindStep;
                                    if ((currentRoll > 0.0f && nextRoll < 0.0f) || (currentRoll < 0.0f && nextRoll > 0.0f))
                                    {
                                        nextRoll = 0.0f;
                                    }
                                    m_aircraft->setAttitude(m_aircraft->attitude().withRoll(nextRoll));
                                }

                                if (legCommandedSpeed > 0.0f)
                                {
                                    // Maintain commanded speed continuously.
                                    // This uses aircraft-specific procedural speed on unconstrained legs,
                                    // and still respects waypoint speed constraints when present.
                                    if (fabs(m_aircraft->groundSpeedKt() - legCommandedSpeed) > 0.5)
                                    {
                                        m_aircraft->setGroundSpeedKt(legCommandedSpeed);
                                    }
                                }

                                if (legTargetAltitude > 0.0f)
                                {
                                    const Altitude altitude = m_aircraft->altitude();
                                    double currentAltitudeMslFeet = altitude.feet();
                                    if (altitude.type() == Altitude::Type::AGL)
                                    {
                                        currentAltitudeMslFeet += host()->queryTerrainElevationAt(m_aircraft->location());
                                    }

                                    const double altitudeDeltaFeet = static_cast<double>(legTargetAltitude) - currentAltitudeMslFeet;
                                    const double distanceMeters = max(1.0, static_cast<double>(GeoMath::getDistanceMeters(m_aircraft->location(), targetPoint)));
                                    const double distanceNm = distanceMeters / 1852.0;
                                    const double groundSpeedKt = max(80.0, m_aircraft->groundSpeedKt());
                                    const double groundSpeedMps = groundSpeedKt * 1852.0 / 3600.0;
                                    const double timeToGoSeconds = max(20.0, distanceMeters / groundSpeedMps);

                                    // On approach/landing legs, clamp descent to a realistic
                                    // glidepath angle (max ~4.5 deg ≈ 475 ft/NM) to prevent
                                    // dive-bombing on short legs of complex RNP procedures.
                                    double requiredVsFpm = (altitudeDeltaFeet / timeToGoSeconds) * 60.0;
                                    if (altitudeDeltaFeet < 0.0 && distanceNm > 0.3)
                                    {
                                        const double maxDescentPerNm = 475.0; // ~4.5 deg
                                        const double maxAltLoss = maxDescentPerNm * distanceNm;
                                        if (fabs(altitudeDeltaFeet) > maxAltLoss)
                                        {
                                            requiredVsFpm = (-maxAltLoss / timeToGoSeconds) * 60.0;
                                        }
                                    }

                                    const auto performance = m_aircraft->performanceProfile();
                                    const double maxClimbFpm = max(900.0, static_cast<double>(performance.initialClimbRocFpm));
                                    const double maxDescentFpm = -max(700.0, static_cast<double>(performance.descentRateFpm) * 2.3);
                                    const double constrainedVsFpm = min(maxClimbFpm, max(maxDescentFpm, requiredVsFpm));

                                    if (fabs(m_aircraft->verticalSpeedFpm() - constrainedVsFpm) > 50.0)
                                    {
                                        m_aircraft->setVerticalSpeedFpm(constrainedVsFpm);
                                    }
                                }

                                return false;
                            }));
                    }
                    else
                    {
                        // No geometric info, fall back to time-based
                        legSteps.push_back(M.delay(chrono::seconds(currentLegDurationSeconds)));
                    }

                    steps.push_back(M.sequence(Maneuver::Type::Flight, currentLegStepId + "_track", legSteps));

                    // Peek at next leg: only continue if it also has the same type.
                    // This prevents the cursor from overshooting into the first leg of the
                    // next procedure (e.g., the EnRoute bridge leg after all Star legs),
                    // which would cause a subsequent activateNextLegOfType() call to skip
                    // that leg because it searches from activeLegIndex + 1.
                    {
                        const auto& allLegs = m_flightPlan->legs();
                        const int nextIdx = cursor->activeLegIndex() + 1;
                        const bool nextLegMatchesType =
                            (nextIdx < static_cast<int>(allLegs.size()) &&
                             allLegs[nextIdx] &&
                             allLegs[nextIdx]->type() == legType);
                        if (!nextLegMatchesType)
                        {
                            break;
                        }
                    }
                }
                while (cursor->activateNextLeg());

                if (steps.empty())
                {
                    return M.instantAction([]{});
                }

                return M.sequence(Maneuver::Type::Flight, stepId, steps);
            });
        }

        bool isHelicopter() const
        {
            if (!flight() || !flight()->aircraft())
            {
                return false;
            }

            if (flight()->aircraft()->category() == Aircraft::Category::Helicopter)
            {
                return true;
            }

            // Some feeds classify rotorcraft as generic GA/Prop while still using
            // helicopter ICAO type designators. Detect common rotorcraft prefixes
            // to route them through helicopter procedures instead of runway rolls.
            string model = flight()->aircraft()->modelIcao();
            transform(model.begin(), model.end(), model.begin(), [](unsigned char c) {
                return static_cast<char>(toupper(c));
            });

            static const vector<string> helicopterPrefixes = {
                "R22", "R44", "R66", "H12", "H13", "H47", "H60", "UH", "AH", "CH", "EC", "BK", "AS3", "AW1", "B06"
            };

            for (const auto& prefix : helicopterPrefixes)
            {
                if (model.rfind(prefix, 0) == 0)
                {
                    return true;
                }
            }

            return false;
        }

        bool hasRunwayFreeHelipadDeparture() const
        {
            return isHelicopter() && m_flightPlan &&
                !m_flightPlan->departureGate().empty() &&
                m_flightPlan->departureRunway().empty();
        }

        bool hasRunwayFreeHelipadArrival() const
        {
            return isHelicopter() && m_flightPlan &&
                !m_flightPlan->arrivalGate().empty() &&
                m_flightPlan->arrivalRunway().empty();
        }

        bool isFighter() const
        {
            return flight() && flight()->aircraft() &&
                   flight()->aircraft()->category() == Aircraft::Category::Fighter;
        }

        double currentAglFeet() const
        {
            if (!m_aircraft)
            {
                return 0.0;
            }

            switch (m_aircraft->altitude().type())
            {
            case Altitude::Type::Ground:
                return 0.0;
            case Altitude::Type::AGL:
                return m_aircraft->altitude().feet();
            case Altitude::Type::MSL:
            default:
                return m_aircraft->altitude().feet() - host()->getWorld()->queryTerrainElevationAt(m_aircraft->location());
            }
        }

        float helipadDepartureHeading(const shared_ptr<ParkingStand>& departureStand) const
        {
            const GeoPoint startPoint = departureStand
                ? departureStand->location().geo()
                : (m_departureAirport ? m_departureAirport->header().datum() : GeoPoint::empty);

            if (m_flightPlan)
            {
                auto arrivalAirport = host()->getWorld()->getAirport(m_flightPlan->arrivalAirportIcao());
                if (arrivalAirport && startPoint != GeoPoint::empty && arrivalAirport->header().datum() != GeoPoint::empty)
                {
                    const double distanceMeters = GeoMath::getDistanceMeters(startPoint, arrivalAirport->header().datum());
                    if (distanceMeters > 1.0)
                    {
                        return GeoMath::getHeadingFromPoints(startPoint, arrivalAirport->header().datum());
                    }
                }
            }

            return departureStand ? departureStand->heading() : 0.0f;
        }

        bool isSmallGaAircraft() const
        {
            if (!flight() || !flight()->aircraft())
            {
                return false;
            }
            switch (flight()->aircraft()->category())
            {
            case Aircraft::Category::LightProp:
            case Aircraft::Category::Prop:
            case Aircraft::Category::Turboprop:
                return true;
            default:
                return false;
            }
        }

        static string upperTextCopy(string value)
        {
            transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
                return static_cast<char>(toupper(c));
            });
            return value;
        }

        static bool textContainsAnyToken(const string& value, initializer_list<const char*> tokens)
        {
            const string upper = upperTextCopy(value);
            for (const char* token : tokens)
            {
                if (token && *token && upper.find(token) != string::npos)
                {
                    return true;
                }
            }
            return false;
        }

        float airportLongestRunwayMeters(const shared_ptr<Airport>& airport) const
        {
            if (!airport)
            {
                return 0.0f;
            }

            try
            {
                const auto runway = airport->findLongestRunway();
                return runway ? runway->lengthMeters() : 0.0f;
            }
            catch (const exception&)
            {
                return 0.0f;
            }
        }

        bool airportLooksLikeInternationalHub(const shared_ptr<Airport>& airport) const
        {
            if (!airport)
            {
                return false;
            }

            const size_t standCount = airport->parkingStands().size();
            const size_t runwayCount = airport->runways().size();
            const float longestRunwayMeters = airportLongestRunwayMeters(airport);
            const bool namedInternational = textContainsAnyToken(airport->header().name(), {
                "INTERNATIONAL",
                "INTL",
                "INTERCONTINENTAL"
            });

            if (airport->parallelRunwayGroupCount() > 1 || runwayCount >= 4 || standCount >= 60)
            {
                return true;
            }

            if (namedInternational && (longestRunwayMeters >= 2400.0f || standCount >= 18 || !airport->header().iata().empty()))
            {
                return true;
            }

            return longestRunwayMeters >= 3400.0f && (standCount >= 28 || !airport->header().iata().empty());
        }

        bool departureAirportSupportsPatternWork() const
        {
            if (!m_departureAirport)
            {
                return false;
            }

            if (airportLooksLikeInternationalHub(m_departureAirport))
            {
                return false;
            }

            const size_t runwayCount = m_departureAirport->runways().size();
            const size_t standCount = m_departureAirport->parkingStands().size();
            const float longestRunwayMeters = airportLongestRunwayMeters(m_departureAirport);

            if (runwayCount > 3)
            {
                return false;
            }

            if (standCount > 40 && longestRunwayMeters > 2600.0f)
            {
                return false;
            }

            return longestRunwayMeters <= 3200.0f || standCount <= 20 || m_departureAirport->header().iata().empty();
        }

        bool isLocalPatternMission() const
        {
            if (!m_departureAirport || !m_flightPlan)
            {
                return false;
            }

            if (m_flightPlan->arrivalAirportIcao() == m_flightPlan->departureAirportIcao())
            {
                return true;
            }

            const auto arrivalAirport = host()->getWorld()->getAirport(m_flightPlan->arrivalAirportIcao());
            if (!arrivalAirport)
            {
                return false;
            }

            const float distanceNm = static_cast<float>(
                GeoMath::getDistanceMeters(
                    m_departureAirport->header().datum(),
                    arrivalAirport->header().datum()) / METERS_IN_1_NAUTICAL_MILE);
            return distanceNm <= 120.0f;
        }

        bool shouldDoUnrestrictedClimbout() const
        {
            // Check if aircraft can perform unrestricted climbouts based on Eurocontrol performance data
            // Requires >= 20,000 ft/min initial climb ROC (F-16: 55000, F-18: 30000, A-10: 6000)
            if (!m_aircraft || !m_aircraft->performanceProfile().canPerformUnrestrictedClimbout())
                return false;

            if (m_aircraft->isFormationWingman())
            {
                return false;
            }

            // Random 50% chance for unrestricted climbout when capable.
            // Use the host RNG so the behavior stays deterministic under tests.
            auto pilotHost = host();
            if (!pilotHost)
            {
                return false;
            }

            int probabilityPercent = 50;
            switch (m_aircraft->missionProfile())
            {
            case AIAircraft::MissionProfile::Training:
                probabilityPercent = 60;
                break;
            case AIAircraft::MissionProfile::Patrol:
                probabilityPercent = 55;
                break;
            case AIAircraft::MissionProfile::LowLevel:
                probabilityPercent = 70;
                break;
            default:
                break;
            }

            return pilotHost->getNextRandom(100) < probabilityPercent;
        }

        bool shouldDoTouchAndGoPatternWork() const
        {
            if (!flight() || !m_aircraft || flight()->rules() != Flight::RulesType::VFR || isHelicopter() || isFighter())
            {
                return false;
            }

            if (!isSmallGaAircraft())
            {
                return false;
            }

            if (m_aircraft->performanceProfile().approachSpeedKt > 140.0f)
            {
                return false;
            }

            if (!departureAirportSupportsPatternWork() || !isLocalPatternMission())
            {
                return false;
            }

            auto pilotHost = host();
            if (!pilotHost)
            {
                return false;
            }

            // Training missions are more likely to do pattern work
            int probabilityPercent = 35;
            if (m_aircraft->missionProfile() == AIAircraft::MissionProfile::Training)
            {
                probabilityPercent = 80;
            }

            return pilotHost->getNextRandom(100) < probabilityPercent;
        }

        // Returns how many VFR pattern laps to fly based on mission profile
        int patternLapsForMission() const
        {
            if (!m_aircraft || !host())
            {
                return 1;
            }

            switch (m_aircraft->missionProfile())
            {
            case AIAircraft::MissionProfile::Training:
                return 3 + host()->getNextRandom(3); // 3-5 laps
            default:
                return 1 + host()->getNextRandom(2); // 1-2 laps
            }
        }

        bool shouldDoPatrolRoute() const
        {
            if (!m_aircraft || !m_flightPlan)
            {
                return false;
            }
            if (m_aircraft->missionProfile() != AIAircraft::MissionProfile::Patrol)
            {
                return false;
            }
            // Patrol route only makes sense for closed-circuit (same departure/arrival) flights
            return m_flightPlan->arrivalAirportIcao() == m_flightPlan->departureAirportIcao();
        }

        void resetFlightCycleState()
        {
            m_lastReceivedIntentId = 0;
            m_lastReceivedControl.reset();
            m_lastDeclineReason = DeclineReason::None;
            m_wasTakeoffClearanceReadBack = false;
            m_continueApproach = false;
            m_departureNumberInLine = 0;
            m_prepareForImmediateTakeoff = false;
            m_holdShortForDeparture = false;
            m_linedUpTimestamp = chrono::microseconds(0);
            m_finalReportedTimestamp = chrono::microseconds(0);
            m_arrivalTowerCheckInDone = false;
            m_departureRadarCheckInDone = false;
            m_arrivalRadarCheckInDone = false;
            m_stoppedBeforeTakeoff = false;
            m_doUnrestrictedClimbout = false;
            m_departureTowerKhz = 0;
            m_departureKhz = 0;
            m_arrivalGroundKhz = 0;

            if (auto cursor = flight()->planCursor())
            {
                cursor->reset();
            }

            for (int clearanceType = static_cast<int>(Clearance::Type::IfrClearance);
                clearanceType <= static_cast<int>(Clearance::Type::GoAroundRequest);
                ++clearanceType)
            {
                flight()->removeClearance(static_cast<Clearance::Type>(clearanceType));
            }
        }

        shared_ptr<Maneuver> maneuverPatrolRoute()
        {
            if (!m_departureAirport || !m_flightPlan)
            {
                return M.instantAction([]{});
            }

            const auto& performance = m_aircraft->performanceProfile();

            const float patrolAglFeet = isFighter() ? 6000.0f : 3000.0f;

            float patrolSpeedKt;
            if (isFighter())
                patrolSpeedKt = min(420.0f, max(280.0f, performance.approachSpeedKt * 2.0f));
            else
                patrolSpeedKt = min(170.0f, max(100.0f, performance.approachSpeedKt + 40.0f));

            const float climbFpm = max(800.0f, performance.descentRateFpm * 1.8f);

            // Patrol heading derived from departure runway; default north
            float patrolHeading = 0.0f;
            if (!m_flightPlan->departureRunway().empty())
            {
                try
                {
                    auto rwy = m_departureAirport->getRunwayOrThrow(m_flightPlan->departureRunway());
                    patrolHeading = rwy->getEndOrThrow(m_flightPlan->departureRunway()).heading();
                }
                catch (...) {}
            }
            const float returnHeading = GeoMath::flipHeading(patrolHeading);

            auto pilotHost = host();
            const int orbits = 3 + (pilotHost ? pilotHost->getNextRandom(3) : 1); // 3-5 orbits

            const float capturedPatrolHeading = patrolHeading;
            const float capturedClimbFpm = climbFpm;
            const float capturedPatrolSpeedKt = patrolSpeedKt;
            const float capturedPatrolAglFeet = patrolAglFeet;
            const int capturedOrbits = orbits;

            vector<shared_ptr<Maneuver>> steps;

            steps.push_back(M.instantAction([this, capturedPatrolHeading, capturedPatrolSpeedKt,
                                             capturedClimbFpm, capturedPatrolAglFeet, capturedOrbits]() {
                host()->writeLog(
                    "AIPILO|Flight[%s] patrol route: hdg=%.0f spd=%.0fkt agl=%.0fft orbits=%d",
                    flight()->callSign().c_str(),
                    capturedPatrolHeading, capturedPatrolSpeedKt,
                    capturedPatrolAglFeet, capturedOrbits);
                m_aircraft->setGroundSpeedKt(capturedPatrolSpeedKt);
                m_aircraft->setVerticalSpeedFpm(capturedClimbFpm);
            }));

            // Climb until patrol altitude is reached, then level off
            steps.push_back(M.await(Maneuver::Type::Unspecified, "await_patrol_altitude", [this, capturedPatrolAglFeet]() {
                return currentAglFeet() >= static_cast<double>(capturedPatrolAglFeet) - 100.0;
            }));
            steps.push_back(M.instantAction([this]() {
                m_aircraft->setVerticalSpeedFpm(0.0);
            }));

            // Fly racetrack orbits: outbound then 180° turn, inbound then 180° turn
            float curHeading = capturedPatrolHeading;
            for (int orbit = 0; orbit < orbits; ++orbit)
            {
                const float outHdg = (orbit % 2 == 0) ? capturedPatrolHeading : returnHeading;
                const float backHdg = (orbit % 2 == 0) ? returnHeading : capturedPatrolHeading;
                // Orbit leg duration from desired ~8 NM leg length at patrol speed.
                const int orbitLegSeconds = max(60, min(600, static_cast<int>(8.0f * 3600.0f / capturedPatrolSpeedKt)));
                steps.push_back(M.airborneTurn(flight(), curHeading, outHdg));
                steps.push_back(M.delay(chrono::seconds(orbitLegSeconds)));
                steps.push_back(M.airborneTurn(flight(), outHdg, backHdg));
                steps.push_back(M.delay(chrono::seconds(orbitLegSeconds)));
                curHeading = backHdg;
            }

            // Return to arrival heading (toward airport) before handing off to final
            steps.push_back(M.instantAction([this]() {
                host()->writeLog(
                    "AIPILO|Flight[%s] patrol complete, returning for landing",
                    flight()->callSign().c_str());
                m_aircraft->setVerticalSpeedFpm(-800.0); // Start descent to approach altitude
            }));

            return M.sequence(Maneuver::Type::Flight, "patrol_route", steps);
        }

        shared_ptr<Maneuver> maneuverVfrTouchAndGoPatternWork()
        {
            if (!m_departureAirport || !m_flightPlan)
            {
                return M.instantAction([]{});
            }

            auto runway = m_departureAirport->getRunwayOrThrow(m_flightPlan->departureRunway());
            const auto& runwayEnd = runway->getEndOrThrow(m_flightPlan->departureRunway());
            const float runwayHeading = runwayEnd.heading();
            const float crosswindHeading = GeoMath::addTurnToHeading(runwayHeading, 90.0f);
            const float downwindHeading = GeoMath::addTurnToHeading(runwayHeading, 180.0f);
            const float baseHeading = GeoMath::addTurnToHeading(runwayHeading, 270.0f);
            const auto performance = m_aircraft->performanceProfile();
            const float patternSpeedKt = max(90.0f, min(130.0f, performance.approachSpeedKt + 15.0f));
            const float recoverySpeedKt = max(patternSpeedKt + 20.0f, performance.approachSpeedKt + 25.0f);
            const float climbFpm = max(600.0f, performance.descentRateFpm * 0.35f);

            return M.sequence(Maneuver::Type::Flight, "vfr_touch_and_go_pattern", {
                M.instantAction([this, patternSpeedKt]() {
                    host()->writeLog(
                        "AIPILO|Flight[%s] VFR touch-and-go pattern work runway[%s] targetSpeed[%.0f]kt",
                        flight()->callSign().c_str(),
                        m_flightPlan->departureRunway().c_str(),
                        patternSpeedKt);
                }),
                M.parallel(Maneuver::Type::Flight, "vfr_pattern_setup", {
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "pattern_speed",
                        m_aircraft->groundSpeedKt(),
                        patternSpeedKt,
                        chrono::seconds(10),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [=](const double& value, double) {
                            m_aircraft->setGroundSpeedKt(value);
                        }
                    )),
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "pattern_climb",
                        m_aircraft->verticalSpeedFpm(),
                        climbFpm,
                        chrono::seconds(10),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [=](const double& value, double) {
                            m_aircraft->setVerticalSpeedFpm(value);
                        }
                    ))
                }),
                M.airborneTurn(flight(), runwayHeading, crosswindHeading),
                // Crosswind leg: ~0.5 NM at pattern speed (distance-based, not fixed time).
                M.delay(chrono::seconds(max(15, min(60, static_cast<int>(0.5f * 3600.0f / patternSpeedKt))))),
                M.airborneTurn(flight(), crosswindHeading, downwindHeading),
                // Downwind leg: ~1.5 NM at pattern speed (distance-based, not fixed time).
                M.delay(chrono::seconds(max(30, min(120, static_cast<int>(1.5f * 3600.0f / patternSpeedKt))))),
                M.parallel(Maneuver::Type::Flight, "vfr_pattern_final_setup", {
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "pattern_final_speed",
                        patternSpeedKt,
                        recoverySpeedKt,
                        chrono::seconds(10),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [=](const double& value, double) {
                            m_aircraft->setGroundSpeedKt(value);
                        }
                    )),
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "pattern_final_vs",
                        m_aircraft->verticalSpeedFpm(),
                        -max(500.0f, performance.descentRateFpm * 0.7f), // Descend on base/final
                        chrono::seconds(10),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [=](const double& value, double) {
                            m_aircraft->setVerticalSpeedFpm(value);
                        }
                    ))
                }),
                M.airborneTurn(flight(), downwindHeading, baseHeading),
                // Base leg: ~0.5 NM at recovery speed (distance-based, not fixed time).
                M.delay(chrono::seconds(max(10, min(45, static_cast<int>(0.5f * 3600.0f / recoverySpeedKt))))),
                M.airborneTurn(flight(), baseHeading, runwayHeading),
                M.instantAction([this, recoverySpeedKt, climbFpm]() {
                    m_aircraft->setGroundSpeedKt(recoverySpeedKt);
                    m_aircraft->setVerticalSpeedFpm(climbFpm);
                })
            });
        }

        shared_ptr<Maneuver> maneuverFlightCycle()
        {
            time_t startTime = flight()->plan()->departureTime() - 180;
            time_t secondsBeforeStart = startTime - host()->getWorld()->currentTime();
            const bool runwayFreeHelipadDeparture = hasRunwayFreeHelipadDeparture();
            const bool runwayFreeHelipadArrival = hasRunwayFreeHelipadArrival();

            // Build flight cycle steps based on aircraft type
            vector<shared_ptr<Maneuver>> steps;
            const bool isWingman = m_aircraft && m_aircraft->isFormationWingman();
            const bool doPatrolRoute = shouldDoPatrolRoute();
            m_doUnrestrictedClimbout = shouldDoUnrestrictedClimbout();

            steps.push_back(M.instantAction([this]() {
                resetFlightCycleState();
            }, "reset_cycle_state"));
            steps.push_back(M.delay(chrono::seconds(secondsBeforeStart)));
            // Formation wingmen do not request their own IFR clearance — the leader handles it
            if (flight()->rules() == Flight::RulesType::IFR && !isWingman)
            {
                steps.push_back(maneuverDepartureAwaitIfrClearance());
            }

            // Helicopters don't need pushback - they start engines on stand and hover-taxi
            if (!isHelicopter())
            {
                steps.push_back(maneuverDepartureAwaitPushback());
                steps.push_back(maneuverDeparturePushbackAndStart());
            }
            else
            {
                // Helicopter: start engines on stand, no pushback needed
                steps.push_back(M.sequence(Maneuver::Type::DeparturePushbackAndStart, "heli_startup", {
                    M.instantAction([=]{ flight()->setPhase(Flight::Phase::Departure); }),
                    M.switchLights(flight(), Aircraft::LightBits::Beacon),
                    M.delay(chrono::seconds(15)),
                    M.switchLights(flight(), Aircraft::LightBits::BeaconNav)
                }));
            }

            if (runwayFreeHelipadDeparture)
            {
                steps.push_back(maneuverHelipadDeparture());
            }
            else
            {
                steps.push_back(maneuverDepartureAwaitTaxi());
                steps.push_back(M.parallel(Maneuver::Type::Unspecified, "", {
                    isHelicopter() ? maneuverDepartureHovertaxi() : maneuverDepartureTaxi(),
                    maneuverAwaitTakeOffClearance(),
                }));

                if (isHelicopter())
                {
                    steps.push_back(maneuverHelicopterRunwayDeparture());
                }
                else
                {
                    steps.push_back(maneuverLineup());

                    // Formation wingman waits for the leader to be airborne before starting takeoff roll
                    if (isWingman)
                    {
                        const int wingmanGapSeconds = isFighter()
                            ? (m_aircraft->missionProfile() == AIAircraft::MissionProfile::Patrol ? 2 : 4)
                            : 10;
                        steps.push_back(M.await(Maneuver::Type::Unspecified, "wait_for_formation_leader_airborne",
                            [this]() {
                                auto leader = m_aircraft->formationLeaderAircraft();
                                // Proceed if leader is now airborne (not ground-based) or leader is gone
                                return !leader || !leader->altitude().isGroundBased();
                            }));
                        steps.push_back(M.delay(chrono::seconds(wingmanGapSeconds))); // Brief gap behind leader
                    }

                    steps.push_back(maneuverTakeoff());
                }

                if (doPatrolRoute)
                {
                    steps.push_back(maneuverPatrolRoute());
                }
                else if (shouldDoTouchAndGoPatternWork())
                {
                    const int laps = patternLapsForMission();
                    steps.push_back(M.instantAction([this, laps]() {
                        host()->writeLog(
                            "AIPILO|Flight[%s] performing %d-lap VFR pattern at airport[%s]",
                            flight()->callSign().c_str(),
                            laps,
                            m_departureAirport ? m_departureAirport->header().icao().c_str() : "");
                    }));
                    for (int lap = 0; lap < laps; ++lap)
                    {
                        steps.push_back(maneuverVfrTouchAndGoPatternWork());
                    }
                }
            }
            // Fighters may do unrestricted climbout with aggressive pitch during SID
            // Patrol missions with closed-circuit flight plans skip the SID/enroute to stay local
            if (!doPatrolRoute)
            {
            if (isFighter() && m_doUnrestrictedClimbout)
            {
                steps.push_back(M.instantAction([this]() {
                    host()->writeLog("AIPILO|Flight[%s] fighter commencing unrestricted climbout", flight()->callSign().c_str());
                }));
                steps.push_back(M.sequence(Maneuver::Type::DepartureClimbBySid, "unrestricted_climbout", {
                    maneuverDepartureRadarCheckIn(),
                    maneuverUnrestrictedClimbout()
                }));
                steps.push_back(maneuverProcedureLeg(FlightPlan::LegType::EnRoute, Flight::Phase::EnRoute, "enroute_leg"));
            }
            else
            {
                steps.push_back(M.sequence(Maneuver::Type::Flight, "sid_leg", {
                    maneuverDepartureRadarCheckIn(),
                    maneuverProcedureLeg(FlightPlan::LegType::Sid, Flight::Phase::Departure, "sid_leg_procedure")
                }));
                steps.push_back(maneuverProcedureLeg(FlightPlan::LegType::EnRoute, Flight::Phase::EnRoute, "enroute_leg"));
            }
            } // end !doPatrolRoute
            steps.push_back(M.instantAction([this, runwayFreeHelipadArrival]() {
                if (runwayFreeHelipadArrival)
                {
                    auto arrivalAirport = m_helper.getArrivalAirport(flight());
                    auto landingStand = arrivalAirport->getParkingStandOrThrow(m_flightPlan->arrivalGate());
                    host()->writeLog(
                        "AIPILO|flight[%s] entering helicopter final to stand[%s]",
                        flight()->callSign().c_str(),
                        landingStand->name().c_str());
                    m_aircraft->setOnHelipadFinal(landingStand);
                    m_aircraft->setManeuver(getHelipadFinalToGate(landingStand));
                    return;
                }

                auto landingRunwayEnd = m_helper.getLandingRunwayEnd(flight());
                host()->writeLog(
                    "AIPILO|flight[%s] entering final approach runway[%s]",
                    flight()->callSign().c_str(),
                    landingRunwayEnd.name().c_str());
                m_aircraft->setOnFinal(landingRunwayEnd);
            }, runwayFreeHelipadArrival ? "enter_heli_final" : "enter_final"));

            auto result = M.sequence(Maneuver::Type::Flight, "flight_cycle", steps);
            return result;
        }

        shared_ptr<Maneuver> maneuverFinalToGate(const Runway::End& landingRunway)
        {
            flight()->setArrivalRunway(landingRunway.name());
            m_arrivalTowerCheckInDone = false;
            m_arrivalRadarCheckInDone = false;
            m_finalReportedTimestamp = chrono::microseconds(0);

            // Reset flight plan cursor to ensure we start from the beginning of the flight plan
            // This is critical because the cursor may have been advanced during departure/enroute phases
            // and we need to find STAR/Approach legs from the beginning
            if (auto cursor = flight()->planCursor())
            {
                cursor->reset();
            }

            vector<shared_ptr<Maneuver>> steps;

            // Spawned arrivals must check in with approach/radar before flying STAR/approach legs.
            // Without this, aircraft appear on approach without ever contacting ATC.
            steps.push_back(maneuverArrivalRadarCheckIn());

            const auto& legs = flight()->plan()->legs();
            const bool hasBridgeLegAfterStar = [&legs]() {
                bool seenStarLeg = false;

                for (const auto& leg : legs)
                {
                    if (!leg)
                    {
                        continue;
                    }

                    if (leg->type() == FlightPlan::LegType::Star)
                    {
                        seenStarLeg = true;
                        continue;
                    }

                    if (!seenStarLeg)
                    {
                        continue;
                    }

                    return leg->type() == FlightPlan::LegType::EnRoute;
                }

                return false;
            }();

            if (hasProcedureLeg(FlightPlan::LegType::Star))
            {
                steps.push_back(maneuverProcedureLeg(FlightPlan::LegType::Star, Flight::Phase::Arrival, "star_leg"));
                if (hasBridgeLegAfterStar)
                {
                    steps.push_back(maneuverProcedureLeg(FlightPlan::LegType::EnRoute, Flight::Phase::Arrival, "arrival_bridge_leg"));
                }
            }
            if (hasProcedureLeg(FlightPlan::LegType::Approach))
            {
                steps.push_back(maneuverProcedureLeg(FlightPlan::LegType::Approach, Flight::Phase::Arrival, "approach_leg"));
            }

            // Fly the runway-alignment segment before final communications so aircraft
            // keep navigating and descending instead of drifting while waiting for tower.
            if (hasProcedureLeg(FlightPlan::LegType::Landing))
            {
                steps.push_back(maneuverProcedureLeg(FlightPlan::LegType::Landing, Flight::Phase::Arrival, "landing_align_leg"));
            }

            steps.push_back(maneuverFinal());
            steps.push_back(M.deferred([this, landingRunway]() {
                if (flight()->tryFindClearance<GoAroundRequest>(Clearance::Type::GoAroundRequest))
                {
                    return maneuverGoAround(landingRunway);
                }

                return M.sequence(Maneuver::Type::ArrivalApproach, "land_to_gate", {
                    maneuverLanding(),
                    M.deferred([this]() {
                        return maneuverArrivalTaxiToGate();
                    })
                });
            }));

            return M.sequence(Maneuver::Type::ArrivalApproach, "final_to_gate", steps);
        }

        shared_ptr<Maneuver> maneuverApproachRetryToGate(const Runway::End& landingRunway)
        {
            flight()->setArrivalRunway(landingRunway.name());
            m_arrivalTowerCheckInDone = false;
            m_finalReportedTimestamp = chrono::microseconds(0);

            // Reset flight plan cursor to ensure we can find approach legs from the beginning
            // This is important after a go-around when the cursor may be at the end of missed approach
            if (auto cursor = flight()->planCursor())
            {
                cursor->reset();
            }

            vector<shared_ptr<Maneuver>> steps;
            if (hasProcedureLeg(FlightPlan::LegType::Approach))
            {
                steps.push_back(maneuverProcedureLeg(FlightPlan::LegType::Approach, Flight::Phase::Arrival, "approach_retry_leg"));
            }

            if (hasProcedureLeg(FlightPlan::LegType::Landing))
            {
                steps.push_back(maneuverProcedureLeg(FlightPlan::LegType::Landing, Flight::Phase::Arrival, "landing_retry_align_leg"));
            }

            steps.push_back(maneuverFinal());
            steps.push_back(M.deferred([this, landingRunway]() {
                if (flight()->tryFindClearance<GoAroundRequest>(Clearance::Type::GoAroundRequest))
                {
                    return maneuverGoAround(landingRunway);
                }

                return M.sequence(Maneuver::Type::ArrivalApproach, "land_after_retry", {
                    maneuverLanding(),
                    M.deferred([this]() {
                        return maneuverArrivalTaxiToGate();
                    })
                });
            }));

            return M.sequence(Maneuver::Type::ArrivalApproach, "approach_retry_to_gate", steps);
        }

        shared_ptr<Maneuver> maneuverFinal()
        {
            auto flaps15GearDown = M.sequence(Maneuver::Type::Unspecified, "flaps_15_gear_down", {
                shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "flaps",
                    0,
                    0.15,
                    chrono::seconds(7),
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress; 
                    },
                    [=](const double& value, double progress) {
                        m_aircraft->setFlapState(value);
                    }
                )),
                shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "gear",
                    0,
                    1.0,
                    chrono::seconds(10),
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress; 
                    },
                    [=](const double& value, double progress) {
                        m_aircraft->setGearState(value);
                    }
                )),
                shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "pitch",
                    -2.0,
                    0.0,
                    chrono::seconds(3),
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress;
                    },
                    [=](const double& value, double progress) {
                        m_aircraft->setAttitude(m_aircraft->attitude().withPitch(value));
                    }
                ))
            });
            auto flaps40 = M.parallel(Maneuver::Type::Unspecified, "flaps_40", {
                shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "flaps",
                    0.15,
                    0.4,
                    chrono::seconds(10),
                    [](const double &from, const double &to, double progress, double &value) {
                        value = from + (to - from) * progress;
                    },
                    [=](const double &value, double progress) {
                        m_aircraft->setFlapState(value);
                    }
                )),
                shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "pitch",
                    0.0,
                    1.5,
                    chrono::seconds(5),
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress;
                    },
                    [=](const double& value, double progress) {
                        m_aircraft->setAttitude(m_aircraft->attitude().withPitch(value));
                    }
                ))
            });

            string landingRunwayName = m_helper.getLandingRunwayEnd(flight()).name();

            auto aviate = M.sequence(Maneuver::Type::Unspecified, "aviate", {
                M.delay(chrono::seconds(3)),
                flaps15GearDown,
                M.delay(chrono::seconds(12)),
                flaps40,
            });

            m_finalReportedTimestamp = chrono::microseconds(0);
            
            auto finalCore = M.parallel(Maneuver::Type::ArrivalApproach, "final_core", {
                aviate,
                M.sequence(Maneuver::Type::Unspecified, "communicate", {
                    M.deferred([this]() {
                        if (m_arrivalTowerCheckInDone)
                        {
                            return M.instantAction([] {});
                        }

                        auto reportFinal = I.pilotReportFinal(flight());
                        if (!reportFinal)
                        {
                            // If no controller is available here, do not deadlock on final.
                            // Anchor timeout so late-clearance auto go-around protection can trigger.
                            m_arrivalTowerCheckInDone = true;
                            m_finalReportedTimestamp = host()->getWorld()->timestamp();
                            host()->writeLog(
                                "AIPILO|WARNING: no arrival controller available for final report flight[%s]; starting final timeout watchdog",
                                flight()->callSign().c_str());
                            return M.instantAction([] {});
                        }

                        m_arrivalTowerCheckInDone = true;
                        m_finalReportedTimestamp = host()->getWorld()->timestamp();

                        return M.sequence(Maneuver::Type::Unspecified, "arrival_tower_checkin_final", {
                            M.tuneComRadio(flight(), reportFinal->subjectControl()->frequency()),
                            M.transmitIntent(flight(), reportFinal, "twr_report_final")
                        });
                    }),
                    M.await(Maneuver::Type::Unspecified, "await_twr_reply", [this]{
                        if (shouldAutoGoAroundForRunwayOvershoot())
                        {
                            ensureAutoGoAroundRequested("runway_overshoot_before_reply");
                        }
                        else if (shouldAutoGoAroundForLateLandingClearance())
                        {
                            ensureAutoGoAroundRequested("landing_clearance_timeout_before_reply");
                        }

                        return (
                            m_continueApproach ||
                            flight()->tryFindClearance<LandingClearance>(Clearance::Type::LandingClearance) ||
                            flight()->tryFindClearance<GoAroundRequest>(Clearance::Type::GoAroundRequest));
                    }),
                    M.deferred([this,landingRunwayName]{
                        if (m_continueApproach)
                        {
                            m_continueApproach = false;
                            auto arrivalControl = m_lastReceivedControl
                                ? m_lastReceivedControl
                                : tryResolveArrivalControlPosition();
                            if (!arrivalControl)
                            {
                                host()->writeLog(
                                    "AIPILO|WARNING: unable to resolve arrival control for continue-approach readback flight[%s]",
                                    flight()->callSign().c_str());
                                return M.instantAction([]{});
                            }

                            return M.transmitIntent(
                                flight(),
                                I.pilotContinueApproachReadback(flight(), arrivalControl, landingRunwayName, m_lastReceivedIntentId));
                        }
                        return M.instantAction([]{});
                    }),
                    M.await(Maneuver::Type::Unspecified, "await_landing_or_go_around", [this]{
                        if (shouldAutoGoAroundForRunwayOvershoot())
                        {
                            ensureAutoGoAroundRequested("runway_overshoot_no_landing_clearance");
                        }
                        else if (shouldAutoGoAroundForLateLandingClearance())
                        {
                            ensureAutoGoAroundRequested("landing_clearance_timeout_on_final");
                        }

                        return (
                            flight()->tryFindClearance<LandingClearance>(Clearance::Type::LandingClearance) ||
                            flight()->tryFindClearance<GoAroundRequest>(Clearance::Type::GoAroundRequest));
                    }),
                    M.deferred([=](){
                        auto goAround = flight()->tryFindClearance<GoAroundRequest>(Clearance::Type::GoAroundRequest);
                        if (goAround)
                        {
                            return M.transmitIntent(
                                flight(),
                                I.pilotGoAroundReadback(goAround, m_lastReceivedIntentId),
                                "readback_go_around");
                        }

                        auto clearance = flight()->findClearanceOrThrow<LandingClearance>(Clearance::Type::LandingClearance);
                        auto readback = I.pilotLandingClearanceReadback(flight(), clearance, m_lastReceivedIntentId);
                        return M.transmitIntent(flight(), readback, "readback_landing_clrnc");
                    })
                })
            });

            return M.sequence(Maneuver::Type::ArrivalApproach, "final", {
                maneuverArrivalRadarCheckIn(),
                finalCore
            });
        }

        shared_ptr<Maneuver> maneuverGoAround(const Runway::End& landingRunway)
        {
            const auto performance = m_aircraft->performanceProfile();

            vector<shared_ptr<Maneuver>> steps = {
                M.instantAction([this]() {
                    flight()->removeClearance(Clearance::Type::LandingClearance);
                    m_continueApproach = false;
                }),
                M.parallel(Maneuver::Type::ArrivalGoAround, "go_around_reconfigure", {
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "go_around_pitch",
                        m_aircraft->attitude().pitch(),
                        10.0,
                        chrono::seconds(4),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [=](const double& value, double) {
                            m_aircraft->setAttitude(m_aircraft->attitude().withPitch(value));
                        }
                    )),
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "go_around_speed",
                        max(120.0, m_aircraft->groundSpeedKt()),
                        max(160.0f, performance.approachSpeedKt + 20.0f),
                        chrono::seconds(8),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [=](const double& value, double) {
                            m_aircraft->setGroundSpeedKt(value);
                        }
                    )),
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "go_around_climb",
                        max(0.0, m_aircraft->verticalSpeedFpm()),
                        max(1200.0f, performance.initialClimbRocFpm * 0.6f),
                        chrono::seconds(6),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [=](const double& value, double) {
                            m_aircraft->setVerticalSpeedFpm(value);
                        }
                    )),
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "go_around_gear",
                        m_aircraft->gearState(),
                        0.0,
                        chrono::seconds(6),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [=](const double& value, double) {
                            m_aircraft->setGearState(value);
                        }
                    )),
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "go_around_flaps",
                        m_aircraft->flapState(),
                        0.15,
                        chrono::seconds(8),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [=](const double& value, double) {
                            m_aircraft->setFlapState(value);
                        }
                    ))
                })
            };

            // Reset cursor before following missed approach legs to ensure they are found from the beginning
            steps.push_back(M.instantAction([this]() {
                if (auto cursor = flight()->planCursor())
                {
                    cursor->reset();
                }
            }));

            if (hasProcedureLeg(FlightPlan::LegType::GoAround))
            {
                steps.push_back(maneuverProcedureLeg(FlightPlan::LegType::GoAround, Flight::Phase::Arrival, "go_around_leg"));
            }
            else
            {
                steps.push_back(M.sequence(Maneuver::Type::ArrivalGoAround, "go_around_fallback", {
                    M.instantAction([this, landingRunway]() {
                        m_aircraft->setAttitude(m_aircraft->attitude().withHeading(landingRunway.heading()));
                    }),
                    M.delay(chrono::seconds(20))
                }));
            }

            steps.push_back(M.instantAction([this, landingRunway]() {
                flight()->removeClearance(Clearance::Type::GoAroundRequest);
                m_aircraft->prepareForApproachRetry(landingRunway);
            }));
            steps.push_back(M.deferred([this, landingRunway]() {
                return maneuverApproachRetryToGate(landingRunway);
            }));

            return M.sequence(Maneuver::Type::ArrivalGoAround, "go_around", steps);
        }

        shared_ptr<Maneuver> maneuverLanding()
        {
            auto landingProcedure = maneuverProcedureLeg(FlightPlan::LegType::Landing, Flight::Phase::Arrival, "landing_leg");
            const auto performance = m_aircraft->performanceProfile();
            const float touchdownSpeedKt = max(15.0f, min(performance.approachSpeedKt, performance.landingTouchdownSpeedKt));
            const float exitSpeedKt = max(0.0f, min(touchdownSpeedKt, performance.landingExitSpeedKt));

            // Calculate rollout duration using Eurocontrol landing distance data if available
            // Eurocontrol landingDistanceMeters is total distance from threshold to full stop
            // Rollout distance = landingDistanceMeters - landingTouchdownDistanceMeters
            double rolloutDurationSeconds;
            if (performance.landingDistanceMeters > 0.0f && performance.landingTouchdownDistanceMeters > 0.0f)
            {
                // Calculate rollout distance from Eurocontrol data
                const float rolloutDistanceMeters = performance.landingDistanceMeters - performance.landingTouchdownDistanceMeters;
                if (rolloutDistanceMeters > 50.0f)
                {
                    // Calculate time to decelerate from touchdown speed to exit speed over rollout distance
                    // Average speed during rollout in m/s (knots * 1852 / 3600)
                    const float avgSpeedMps = ((touchdownSpeedKt + exitSpeedKt) / 2.0f) * 1852.0f / 3600.0f;
                    rolloutDurationSeconds = max(6.0, static_cast<double>(rolloutDistanceMeters) / max(0.1f, avgSpeedMps));
                }
                else
                {
                    // Fallback to deceleration-based calculation
                    rolloutDurationSeconds = static_cast<double>(touchdownSpeedKt - exitSpeedKt) /
                        max(0.5f, performance.landingRolloutDecelerationKtPerSecond);
                }
            }
            else
            {
                // Fallback to deceleration-based calculation
                rolloutDurationSeconds = max(
                    6.0,
                    static_cast<double>(touchdownSpeedKt - exitSpeedKt) /
                        max(0.5f, performance.landingRolloutDecelerationKtPerSecond));
            }
            const auto rolloutDuration = chrono::milliseconds(static_cast<int64_t>(rolloutDurationSeconds * 1000.0));

            auto preFlare = M.deferred([=]() {
                const double startVs = m_aircraft->verticalSpeedFpm();
                const double endVs = -performance.descentRateFpm * 0.5;
                const double currentPitch = m_aircraft->attitude().pitch();
                return M.parallel(Maneuver::Type::ArrivalLanding, "pre_flare", {
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "", 
                        currentPitch,
                        max(2.0, currentPitch + 1.5),
                        chrono::milliseconds(3500),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress; 
                        },
                        [=](const double& value, double progress) {
                            m_aircraft->setAttitude(m_aircraft->attitude().withPitch(value));
                        }
                    )),
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "", 
                        startVs,
                        endVs,
                        chrono::milliseconds(3500),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress; 
                        },
                        [=](const double& value, double progress) {
                            m_aircraft->setVerticalSpeedFpm(value);
                        }
                    )),
                });
            });
            const float preFlarePitchEnd = static_cast<float>(max(2.0, m_aircraft->attitude().pitch() + 1.5));
            auto flare = M.parallel(Maneuver::Type::ArrivalLanding, "flare", {
                shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "pitch",
                    preFlarePitchEnd,
                    preFlarePitchEnd + 2.5,
                    chrono::seconds(3),
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress; 
                    },
                    [=](const double& value, double progress) {
                        m_aircraft->setAttitude(m_aircraft->attitude().withPitch(value));
                    }
                )),
                shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "gndspd",
                    performance.approachSpeedKt,
                    touchdownSpeedKt,
                    chrono::seconds(3),
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress;
                    },
                    [=](const double& value, double progress) {
                        m_aircraft->setGroundSpeedKt(value);
                    }
                )),
                M.sequence(Maneuver::Type::Unspecified, "", {
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "verspd_1",
                        -performance.descentRateFpm * 0.5,
                        -performance.descentRateFpm * 0.05,
                        chrono::seconds(2),
                        [](const double &from, const double &to, double progress, double &value) {
                            value = from + (to - from) * progress;
                        },
                        [=](const double &value, double progress) {
                            m_aircraft->setVerticalSpeedFpm(value);
                        }
                    )),
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "verspd_2",
                        isHelicopter() ? -30.0 : -80.0,
                        isHelicopter() ? -60.0 : (m_aircraft->category() == Aircraft::Category::Heavy ? -250.0 : (m_aircraft->category() == Aircraft::Category::LightProp ? -120.0 : -180.0)),
                        chrono::seconds(1),
                        [](const double &from, const double &to, double progress, double &value) {
                            value = from + (to - from) * progress;
                        },
                        [=](const double &value, double progress) {
                            m_aircraft->setVerticalSpeedFpm(value);
                        }
                    ))
                })
            });
            auto logTouchDown = M.instantAction([this](){
                const auto& perf = m_aircraft->performanceProfile();
                if (perf.landingDistanceMeters > 500.0f)
                {
                    host()->writeLog(
                        "AIPILO|TOUCHDOWN flight[%s] at [%f,%f] vertical-speed[%f]fpm ground-speed[%f]kt pitch[%f]deg lda[%0.fm]",
                        flight()->callSign().c_str(),
                        m_aircraft->location().latitude,
                        m_aircraft->location().longitude,
                        m_aircraft->verticalSpeedFpm(),
                        m_aircraft->groundSpeedKt(),
                        m_aircraft->attitude().pitch(),
                        perf.landingDistanceMeters);
                }
                else
                {
                    host()->writeLog(
                        "AIPILO|TOUCHDOWN flight[%s] at [%f,%f] vertical-speed[%f]fpm ground-speed[%f]kt pitch[%f]deg",
                        flight()->callSign().c_str(),
                        m_aircraft->location().latitude,
                        m_aircraft->location().longitude,
                        m_aircraft->verticalSpeedFpm(),
                        m_aircraft->groundSpeedKt(),
                        m_aircraft->attitude().pitch());
                }
            });
            auto touchDownAndDeccelerate = M.parallel(Maneuver::Type::ArrivalLandingRoll, "touch_and_decel", {
                shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "spdbrk",
                    0,
                    1.0,
                    chrono::seconds(1),
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress; 
                    },
                    [=](const double& value, double progress) {
                        m_aircraft->setSpoilerState(value);
                    }
                )),
                shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "pitch",
                    5.5,
                    0.0,
                    chrono::seconds(6),
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress; 
                    },
                    [=](const double& value, double progress) {
                        m_aircraft->setAttitude(m_aircraft->attitude().withPitch(value));
                    }
                )),
                shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "gndspd",
                    touchdownSpeedKt,
                    exitSpeedKt,
                    rolloutDuration,
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress; 
                    },
                    [=](const double& value, double progress) {
                        m_aircraft->setGroundSpeedKt(value);
                    }
                )),
            });

            return M.sequence(Maneuver::Type::ArrivalLanding, "landing", {
                landingProcedure,
                M.await(Maneuver::Type::Unspecified, "await_55_agl", [=]() {
                    return (m_aircraft->altitude().type() == Altitude::Type::AGL && m_aircraft->altitude().feet() <= 55);
                }),
                preFlare,
                M.await(Maneuver::Type::Unspecified, "await_20_agl", [=]() {
                    return (m_aircraft->altitude().type() == Altitude::Type::AGL && m_aircraft->altitude().feet() <= 20);
                }),
                flare,
                M.await(Maneuver::Type::Unspecified, "await_touch_down", [=]() {
                    return (m_aircraft->altitude().type() == Altitude::Type::Ground);
                }),
                logTouchDown,
                touchDownAndDeccelerate
            });
        }

        shared_ptr<Maneuver> maneuverHelipadArrivalToGate(shared_ptr<ParkingStand> landingStand)
        {
            if (!landingStand)
            {
                return M.instantAction([]{});
            }

            const GeoPoint touchdownPoint = landingStand->location().geo();
            const float approachSpeedKt = max(35.0f, min(80.0f, m_aircraft->performanceProfile().approachSpeedKt));
            const float shortFinalSpeedKt = max(18.0f, min(45.0f, approachSpeedKt * 0.55f));
            const float hoverSpeedKt = max(4.0f, min(12.0f, shortFinalSpeedKt * 0.35f));
            const float approachDescentFpm = -max(180.0f, m_aircraft->performanceProfile().descentRateFpm * 0.55f);
            const float shortFinalDescentFpm = -max(120.0f, m_aircraft->performanceProfile().descentRateFpm * 0.35f);
            const float hoverDescentFpm = -max(90.0f, m_aircraft->performanceProfile().descentRateFpm * 0.22f);
            const auto distanceToStandMeters = [this, touchdownPoint]() {
                return GeoMath::getDistanceMeters(m_aircraft->location(), touchdownPoint);
            };

            return M.sequence(Maneuver::Type::ArrivalApproach, "heli_arrival", {
                M.instantAction([this, landingStand, touchdownPoint, approachSpeedKt, approachDescentFpm]() {
                    flight()->setPhase(Flight::Phase::Arrival);
                    if (auto arrivalTower = m_helper.tryGetArrivalTower(flight(), touchdownPoint))
                    {
                        aircraft()->setFrequency(arrivalTower->frequency());
                    }

                    host()->writeLog(
                        "AIPILO|Flight[%s] helicopter arrival direct to stand[%s]",
                        flight()->callSign().c_str(),
                        landingStand->name().c_str());

                    const float trackToStand = GeoMath::getHeadingFromPoints(m_aircraft->location(), touchdownPoint);
                    m_aircraft->setLights(Aircraft::LightBits::BeaconLandingNavStrobe);
                    m_aircraft->setAttitude(m_aircraft->attitude().withHeading(trackToStand));
                    m_aircraft->setTrack(trackToStand);
                    m_aircraft->setGroundSpeedKt(approachSpeedKt);
                    m_aircraft->setVerticalSpeedFpm(approachDescentFpm);
                }, "heli_arrival_setup"),
                M.await(Maneuver::Type::ArrivalApproach, "await_heli_short_final", [this, distanceToStandMeters]() {
                    return distanceToStandMeters() <= 180.0 || currentAglFeet() <= 120.0;
                }),
                M.instantAction([this, touchdownPoint, shortFinalSpeedKt, shortFinalDescentFpm]() {
                    const float trackToStand = GeoMath::getHeadingFromPoints(m_aircraft->location(), touchdownPoint);
                    m_aircraft->setAttitude(m_aircraft->attitude().withHeading(trackToStand).withPitch(-0.5f));
                    m_aircraft->setTrack(trackToStand);
                    m_aircraft->setGroundSpeedKt(shortFinalSpeedKt);
                    m_aircraft->setVerticalSpeedFpm(shortFinalDescentFpm);
                }, "heli_short_final"),
                M.await(Maneuver::Type::ArrivalApproach, "await_heli_hover", [this, distanceToStandMeters]() {
                    return distanceToStandMeters() <= 35.0 || currentAglFeet() <= 20.0;
                }),
                M.instantAction([this, touchdownPoint, hoverSpeedKt, hoverDescentFpm]() {
                    const float trackToStand = GeoMath::getHeadingFromPoints(m_aircraft->location(), touchdownPoint);
                    m_aircraft->setAttitude(m_aircraft->attitude().withHeading(trackToStand).withPitch(1.0f));
                    m_aircraft->setTrack(trackToStand);
                    m_aircraft->setGroundSpeedKt(hoverSpeedKt);
                    m_aircraft->setVerticalSpeedFpm(hoverDescentFpm);
                }, "heli_hover_descent"),
                M.await(Maneuver::Type::ArrivalApproach, "await_heli_touchdown", [this]() {
                    return m_aircraft->altitude().type() == Altitude::Type::Ground;
                }),
                M.instantAction([this, landingStand]() {
                    host()->writeLog(
                        "AIPILO|Flight[%s] helicopter parked at stand[%s]",
                        flight()->callSign().c_str(),
                        landingStand->name().c_str());
                    m_aircraft->park(landingStand);
                }, "heli_park")
            });
        }

        shared_ptr<Maneuver> maneuverArrivalTaxiToGate()
        {
            WorldHelper helper(host());
            auto aircraft = flight()->aircraft();
            auto airport = helper.getArrivalAirport(flight());
            auto runway = airport->getRunwayOrThrow(m_flightPlan->arrivalRunway());
            const auto& runwayEnd = runway->getEndOrThrow(m_flightPlan->arrivalRunway());
            auto gate = airport->getParkingStandOrThrow(m_flightPlan->arrivalGate());
            shared_ptr<TaxiEdge> exitFirstEdge;
            shared_ptr<TaxiEdge> exitLastEdge;
            string exitName;
            shared_ptr<Intent> arrivalCheckInIntent;
            bool needsGroundTaxiClearance = false;

            const auto safeCreateExitManeuver = [
                this, &exitFirstEdge, &exitLastEdge, &exitName, &arrivalCheckInIntent, &needsGroundTaxiClearance, airport, runway, gate, aircraft, runwayEnd
            ]{
                shared_ptr<Maneuver> result;
                host()->writeLog(
                    "AIPILO|Flight[%s] landed rwy[%s] will look for exit path",
                    flight()->callSign().c_str(), runwayEnd.name().c_str());

                auto taxiPath = airport->taxiNet()->tryFindExitPathFromRunway(
                    host(),
                    runway,
                    runwayEnd,
                    gate,
                    aircraft->location(),
                    static_cast<float>(aircraft->groundSpeedKt()));

                if (taxiPath)
                {
                    exitFirstEdge = taxiPath->edges[0];
                    exitLastEdge = taxiPath->edges[taxiPath->edges.size() - 1];
                    exitName = taxiPath->toHumanFriendlyString();

                    host()->writeLog(
                        "AIPILO|Flight[%s] arrival gate[%s] will exit runway[%s] via[%s]",
                        flight()->callSign().c_str(),
                        gate->name().c_str(),
                        runwayEnd.name().c_str(),
                        exitName.c_str());

                    arrivalCheckInIntent = I.pilotArrivalCheckInWithGround(
                        flight(),
                        runwayEnd.name(),
                        exitName,
                        exitLastEdge);
                    needsGroundTaxiClearance = !!arrivalCheckInIntent;
                    result = M.taxiByPath(flight(), taxiPath, ManeuverFactory::TaxiType::HighSpeed);
                }
                else
                {
                    host()->writeLog(
                        "AIPILO|Flight[%s] arrival exit path from runway NOT FOUND! will teleport to gate[%s]",
                        flight()->callSign().c_str(),
                        gate->name().c_str());
                    result = M.instantAction([=]{
                        aircraft->park(gate);
                    });
                }

                return result;
            };

            // Flap retraction time scales with aircraft category: heavies have larger,
            // slower-acting flaps; light props retract quickly.
            const int flapRetractSec = (m_aircraft->category() == Aircraft::Category::Heavy ? 45
                : (m_aircraft->category() == Aircraft::Category::LightProp ? 12
                : (m_aircraft->category() == Aircraft::Category::Turboprop ? 18 : 30)));
            auto flapsZero = shared_ptr<Maneuver>(new AnimationManeuver<double>(
                "flaps_0",
                0.4,
                0,
                chrono::seconds(flapRetractSec),
                [](const double& from, const double& to, double progress, double& value) {
                    value = from + (to - from) * progress;
                },
                [=](const double& value, double progress) {
                    m_aircraft->setFlapState(value);
                }
            ));
            auto speedBrakeDown = shared_ptr<Maneuver>(new AnimationManeuver<double>(
                "speedbrk_down",
                1.0,
                0,
                chrono::seconds(1),
                [](const double& from, const double& to, double progress, double& value) {
                    value = from + (to - from) * progress;
                },
                [=](const double& value, double progress) {
                    m_aircraft->setSpoilerState(value);
                }
            ));
            auto exitRunway = safeCreateExitManeuver();
            auto logVacatedActive = M.instantAction([this](){
                host()->writeLog(
                    "AIPILO|VACATEDACTIVE flight[%s] at [%f,%f] ground-speed[%f]kt",
                    flight()->callSign().c_str(),
                    m_aircraft->location().latitude,
                    m_aircraft->location().longitude,
                    m_aircraft->groundSpeedKt());
            });
            auto taxiLights = M.instantAction([=] {
                m_aircraft->setLights(Aircraft::LightBits::BeaconTaxiNav);
            });
            auto lightsOff = M.instantAction([=] {
                m_aircraft->setLights(Aircraft::LightBits::None);
                m_aircraft->setGearState(1.0f);
                m_aircraft->setFlapState(0.0f);
                m_aircraft->setSpoilerState(0.0f);
                m_aircraft->setGroundSpeedKt(0);
                m_aircraft->setVerticalSpeedFpm(0);
                flight()->setPhase(Flight::Phase::TurnAround);
            });

            const auto onHoldingShort = [=](shared_ptr<TaxiEdge> holdShortEdge) {
                if (holdShortEdge->activeZones().arrival.runwaysMask() == runway->maskBit())
                {
                    //TODO: this is a hack. Instead check if the runway is ahead or behind
                    return M.instantAction([]{}); // don't hold short of runway we've just landed on (and which is supposed to be behind us).
                }
                return maneuverAwaitCrossRunway(airport, holdShortEdge);
            };

            return M.sequence(Maneuver::Type::ArrivalTaxi, "arrival_taxi", {
                M.instantAction([this] {
                    m_aircraft->setGroundSpeedKt(0);
                }),
                M.parallel(Maneuver::Type::Unspecified, "", {
                    flapsZero,
                    speedBrakeDown,
                    M.deferred([=]() {
                        if (!needsGroundTaxiClearance)
                        {
                            return M.instantAction([]{});
                        }

                        return M.sequence(Maneuver::Type::Unspecified, "arrival_ground_contact", {
                            M.await(Maneuver::Type::Unspecified, "", [this,exitFirstEdge]{
                                return (!exitFirstEdge) || hasReachedWaypoint(exitFirstEdge->node2()->location().geo(), 300.0f);
                            }),
                            M.delay(chrono::seconds(3)),
                            M.tuneComRadio(flight(), arrivalCheckInIntent->subjectControl()->frequency()),
                            M.transmitIntent(flight(), arrivalCheckInIntent),
                            M.awaitClearance(flight(), Clearance::Type::ArrivalTaxiClearance, "await_taxi_clrnc"),
                            M.deferred([this]{
                                auto clearance = flight()->findClearanceOrThrow<ArrivalTaxiClearance>(Clearance::Type::ArrivalTaxiClearance);
                                return M.transmitIntent(flight(), I.pilotArrivalTaxiReadback(flight(), m_lastReceivedIntentId));
                            }),
                        });
                    }),
                    M.sequence(Maneuver::Type::Unspecified, "", {
                       exitRunway,
                       logVacatedActive,
                       taxiLights,
                       M.deferred([=]() {
                           if (!needsGroundTaxiClearance)
                           {
                               auto directTaxiPath = airport->taxiNet()->tryFindTaxiPathToGate(gate, aircraft->location());
                               if (!directTaxiPath)
                               {
                                   host()->writeLog(
                                       "AIPILO|Flight[%s] arrival taxi path to gate[%s] not found, parking directly",
                                       flight()->callSign().c_str(),
                                       gate->name().c_str());
                                   return M.instantAction([=]{
                                       aircraft->park(gate);
                                   });
                               }

                               return M.taxiByPath(
                                   flight(),
                                   directTaxiPath,
                                   ManeuverFactory::TaxiType::Normal,
                                   onHoldingShort);
                           }

                           return M.sequence(Maneuver::Type::Unspecified, "arrival_taxi_to_gate", {
                               M.awaitClearance(flight(), Clearance::Type::ArrivalTaxiClearance),
                               M.parallel(Maneuver::Type::Unspecified, "", {
                                   M.deferred([=]{
                                       auto clearance = flight()->findClearanceOrThrow<ArrivalTaxiClearance>(Clearance::Type::ArrivalTaxiClearance);
                                       if (!clearance->taxiPath())
                                       {
                                           host()->writeLog(
                                               "AIPILO|Flight[%s] arrival taxi path to gate[%s] not found, parking directly",
                                               flight()->callSign().c_str(),
                                               gate->name().c_str());
                                           return M.instantAction([=]{
                                               aircraft->park(gate);
                                           });
                                       }

                                       return M.taxiByPath(
                                           flight(),
                                           clearance->taxiPath(),
                                           ManeuverFactory::TaxiType::Normal,
                                           onHoldingShort);
                                   }),
                               }),
                           });
                       }),
                   }),
                }),
                M.delay(chrono::seconds(5)),
                lightsOff
            });
        }

        shared_ptr<Maneuver> maneuverDepartureAwaitIfrClearance()
        {
            auto intentFactory = host()->services().get<IntentFactory>();
            auto ifrClearanceRequest = intentFactory->pilotIfrClearanceRequest(flight());
            auto airport = host()->getWorld()->getAirport(flight()->plan()->departureAirportIcao());
            auto clearanceDelivery = airport->clearanceDeliveryAt(flight()->aircraft()->location());
            auto ground = airport->groundAt(flight()->aircraft()->location());

            return M.sequence(Maneuver::Type::DepartureAwaitIfrClearance, "await_ifr_clr", {
                M.tuneComRadio(flight(), clearanceDelivery->frequency()),
                M.transmitIntent(flight(), ifrClearanceRequest),
                M.awaitClearance(flight(), Clearance::Type::IfrClearance),
                M.deferred([=]() {
                    host()->writeLog("ifrClearanceReadback deferred factory");
                    auto ifrClearanceReadback = intentFactory->pilotIfrClearanceReadback(flight(), m_lastReceivedIntentId);
                    return M.transmitIntent(flight(), ifrClearanceReadback);
                }),
                M.await(Maneuver::Type::Unspecified, "", [=](){
                    return flight()->findClearanceOrThrow<IfrClearance>(Clearance::Type::IfrClearance)->readbackCorrect();
                }),
                M.deferred([=]() {
                    host()->writeLog("deliveryToGroundHandoffReadback deferred factory");
                    auto handoffReadback = intentFactory->pilotHandoffReadback(
                        flight(), clearanceDelivery, ground->frequency()->khz(), m_lastReceivedIntentId);
                    return M.transmitIntent(flight(), handoffReadback);
                }),
                M.deferred([=]() {
                    return M.delay(chrono::seconds(5));
                })
            });
        }

        shared_ptr<Maneuver> maneuverDepartureAwaitPushback()
        {
            auto intentFactory = host()->services().get<IntentFactory>();
            auto pushAndStartRequest = intentFactory->pilotPushAndStartRequest(flight());
            auto airport = host()->getWorld()->getAirport(flight()->plan()->departureAirportIcao());
            auto ground = airport->groundAt(flight()->aircraft()->location());

            return M.sequence(Maneuver::Type::DepartureAwaitPushback, "await_pushback", {
                M.tuneComRadio(flight(), ground->frequency()),
                M.transmitIntent(flight(), pushAndStartRequest),
                M.awaitClearance(flight(), Clearance::Type::PushAndStartApproval),
                M.deferred([=]() {
                    host()->writeLog("pushAndStartReadback deferred factory");
                    auto pushAndStartReadback =
                        intentFactory->pilotPushAndStartReadback(flight(), ground, m_lastReceivedIntentId);
                    return M.transmitIntent(flight(), pushAndStartReadback);
                }),
                M.deferred([=]() {
                    return M.delay(chrono::seconds(5));
                })
            });
        }
        
        shared_ptr<Maneuver> maneuverDeparturePushbackAndStart()
        {
            const auto createPushbackTaxiPath = [this](const vector<GeoPoint>& pushbackPath) {
                if (pushbackPath.size() < 2)
                {
                    host()->writeLog(
                        "AIPILO|Flight[%s] pushback path invalid: expected >=2 points, got %zu",
                        flight()->callSign().c_str(),
                        pushbackPath.size());
                    return shared_ptr<TaxiPath>();
                }

                vector<shared_ptr<TaxiEdge>> pushbackEdges;
                for (int i = 0 ; i < pushbackPath.size() - 1 ; i++)
                {
                    host()->writeLog(
                        "PUSHBACK-PATH (%.9f,%.9f)->(%.9f,%.9f)", 
                        pushbackPath[i].latitude,
                        pushbackPath[i].longitude,
                        pushbackPath[i+1].latitude,
                        pushbackPath[i+1].longitude);
                    pushbackEdges.push_back(shared_ptr<TaxiEdge>(new TaxiEdge(
                        UniPoint::fromGeo(host(), pushbackPath[i]),
                        UniPoint::fromGeo(host(), pushbackPath[i+1])
                    )));
                }
                auto taxiPath = shared_ptr<TaxiPath>(new TaxiPath(
                    pushbackEdges[0]->node1(),
                    pushbackEdges[pushbackEdges.size()-1]->node2(),
                    pushbackEdges
                ));
                return taxiPath;
            };

            return DeferredManeuver::create(Maneuver::Type::DeparturePushbackAndStart, "push_and_start", [=]() {
                auto flightPlan = flight()->plan();
                auto approval = flight()->findClearanceOrThrow<PushAndStartApproval>(Clearance::Type::PushAndStartApproval);
                auto taxiPath = createPushbackTaxiPath(approval->pushbackPath());

                if (!taxiPath)
                {
                    return M.sequence(Maneuver::Type::DeparturePushbackAndStart, "push_and_start_no_path", {
                        M.instantAction([=]{
                            flight()->setPhase(Flight::Phase::Departure);
                            host()->writeLog(
                                "AIPILO|Flight[%s] pushback path unavailable, skipping pushback motion",
                                flight()->callSign().c_str());
                        }),
                        M.switchLights(flight(), Aircraft::LightBits::Beacon),
                        M.delay(chrono::seconds(10)),
                        M.switchLights(flight(), Aircraft::LightBits::BeaconNav)
                    });
                }

                vector<shared_ptr<Maneuver>> maneuverSteps = {
                    M.instantAction([=]{
                        flight()->setPhase(Flight::Phase::Departure);
                    }),
                    M.switchLights(flight(), Aircraft::LightBits::Beacon),
                    M.delay(chrono::seconds(10)),
                    M.switchLights(flight(), Aircraft::LightBits::BeaconNav),
                    M.delay(chrono::seconds(5)),
                    M.taxiByPath(flight(), taxiPath, ManeuverFactory::TaxiType::Pushback)
                };

                return shared_ptr<Maneuver>(new SequentialManeuver(
                    Maneuver::Type::DeparturePushbackAndStart, 
                    "",
                    maneuverSteps
                )); 
            });
        }
        
        shared_ptr<Maneuver> maneuverDepartureAwaitTaxi()
        {
            auto flapsToTakeoffPosition = shared_ptr<Maneuver>(new AnimationManeuver<double>(
                "flaps_t_o",
                0.0,
                0.15,
                chrono::seconds(3),
                [](const double& from, const double& to, double progress, double& value) {
                    value = from + (to - from) * progress; 
                },
                [this](const double& value, double progress) {
                    m_aircraft->setFlapState(value);
                }
            ));

            auto intentFactory = host()->services().get<IntentFactory>();
            auto taxiRequest = intentFactory->pilotDepartureTaxiRequest(flight());

            return M.sequence(Maneuver::Type::DepartureAwaitTaxi, "await_departure_taxi", {
                M.taxiStop(flight()),
                M.delay(chrono::seconds(5)),
                flapsToTakeoffPosition,
                M.delay(chrono::seconds(5)),
                M.transmitIntent(flight(), taxiRequest),
                M.awaitClearance(flight(), Clearance::Type::DepartureTaxiClearance),
                M.deferred([=]() {
                    host()->writeLog("taxiReadback deferred factory");
                    auto taxiReadback = intentFactory->pilotDepartureTaxiReadback(flight(), m_lastReceivedIntentId);
                    return M.transmitIntent(flight(), taxiReadback);
                }),
                M.deferred([=]() {
                    return M.delay(chrono::seconds(10));
                }),
            });
        }

        shared_ptr<Maneuver> maneuverDepartureTaxi()
        {
            const auto addLineupEdges = [=](shared_ptr<DepartureTaxiClearance> clearance) {
                auto taxiPath = clearance->taxiPath();
                if (!taxiPath)
                {
                    host()->writeLog("AIPILO|Flight[%s] addLineupEdges: null taxiPath, cannot add lineup edges", flight()->callSign().c_str());
                    return;
                }

                auto runway = m_departureAirport->getRunwayOrThrow(clearance->departureRunway());
                const auto& runwayEnd = runway->getEndOrThrow(clearance->departureRunway());

                // Use actual runway centerline point; taxiPath terminal node can still be hold-short.
                auto centerlinePoint = runwayEnd.centerlinePoint().geo();
                auto lineupPoint1 = GeoMath::getPointAtDistance(
                    centerlinePoint,
                    runwayEnd.heading(),
                    30);
                auto lineupPoint2 = GeoMath::getPointAtDistance(
                    centerlinePoint,
                    runwayEnd.heading(),
                    60);

                host()->writeLog(
                    "AIPILO|Flight[%s] addLineupEdges: path had %zu edges, adding lineup points from [%f,%f]",
                    flight()->callSign().c_str(),
                    taxiPath->edges.size(),
                    centerlinePoint.latitude,
                    centerlinePoint.longitude);

                taxiPath->appendEdgeTo(UniPoint::fromGeo(host(), lineupPoint1));
                taxiPath->appendEdgeTo(UniPoint::fromGeo(host(), lineupPoint2));
            };

            return DeferredManeuver::create(Maneuver::Type::DepartureTaxi, "departure_taxi", [=]() {
                auto clearance = flight()->findClearanceOrThrow<DepartureTaxiClearance>(Clearance::Type::DepartureTaxiClearance);
                // Note: addLineupEdges removed - lineup now happens AFTER clearance is received

                auto departureRunway = m_departureAirport->getRunwayOrThrow(m_flightPlan->departureRunway());

                const auto edgeMatchesDepartureRunway = [departureRunway](shared_ptr<TaxiEdge> edge) {
                    return edge && (
                        edge->activeZones().departue.has(departureRunway) ||
                        edge->activeZones().arrival.has(departureRunway) ||
                        edge->activeZones().ils.has(departureRunway));
                };

                const auto findFinalDepartureHoldShortEdge = [this, edgeMatchesDepartureRunway, departureRunway](shared_ptr<TaxiPath> taxiPath) {
                    shared_ptr<TaxiEdge> result;

                    if (!taxiPath || !departureRunway)
                    {
                        host()->writeLog("AIPILO|Flight[%s] findFinalDepartureHoldShortEdge: null path or runway", flight()->callSign().c_str());
                        return result;
                    }

                    host()->writeLog(
                        "AIPILO|Flight[%s] findFinalDepartureHoldShortEdge: checking %zu edges for runway[%s]",
                        flight()->callSign().c_str(),
                        taxiPath->edges.size(),
                        departureRunway->end1().name().c_str());

                    for (int i = 0; i < (int)taxiPath->edges.size(); i++)
                    {
                        auto edge = taxiPath->edges[i];
                        auto prevEdge = i > 0 ? taxiPath->edges[i - 1] : nullptr;
                        auto nextEdge = (i < (int)taxiPath->edges.size() - 1) ? taxiPath->edges[i + 1] : nullptr;

                        bool entersActiveZone = edge->activeZones().hasAny() && (!prevEdge || !prevEdge->activeZones().hasAny());
                        bool matchesDepartureZone = edgeMatchesDepartureRunway(edge);

                        // The TRUE final hold short is the edge that:
                        // 1. Enters the departure runway's active zone, AND
                        // 2. The NEXT edge is the actual runway edge (Type::Runway)
                        // This prevents taxiway crossings (which have active zones but lead to more taxiway)
                        // from being incorrectly detected as the final hold short
                        bool leadsToRunwayEdge = nextEdge && nextEdge->type() == TaxiEdge::Type::Runway;

                        if (entersActiveZone && matchesDepartureZone)
                        {
                            host()->writeLog(
                                "AIPILO|Flight[%s] edge[%d|%s|type=%d] enters departure active zone, leadsToRunway=%d",
                                flight()->callSign().c_str(),
                                edge->id(),
                                edge->name().c_str(),
                                static_cast<int>(edge->type()),
                                leadsToRunwayEdge);

                            if (!result && leadsToRunwayEdge)
                            {
                                result = edge;
                            }
                        }
                    }

                    // Fallback: if no edge leads directly to runway, find the last edge
                    // that enters the departure runway's active zone
                    if (!result)
                    {
                        for (int i = (int)taxiPath->edges.size() - 1; i >= 0; i--)
                        {
                            auto edge = taxiPath->edges[i];
                            auto prevEdge = i > 0 ? taxiPath->edges[i - 1] : nullptr;

                            bool entersActiveZone = edge->activeZones().hasAny() && (!prevEdge || !prevEdge->activeZones().hasAny());
                            bool matchesDepartureZone = edgeMatchesDepartureRunway(edge);

                            if (entersActiveZone && matchesDepartureZone)
                            {
                                result = edge;
                                host()->writeLog(
                                    "AIPILO|Flight[%s] findFinalDepartureHoldShortEdge: fallback selected edge[%d|%s]",
                                    flight()->callSign().c_str(),
                                    edge->id(),
                                    edge->name().c_str());
                                break;
                            }
                        }
                    }

                    // Fallback for airports without active zones (like LECO):
                    // Look for HoldShort type edges that lead to runway edges
                    if (!result)
                    {
                        for (int i = 0; i < (int)taxiPath->edges.size(); i++)
                        {
                            auto edge = taxiPath->edges[i];
                            auto nextEdge = (i < (int)taxiPath->edges.size() - 1) ? taxiPath->edges[i + 1] : nullptr;

                            if (edge && edge->type() == TaxiEdge::Type::HoldShort &&
                                nextEdge && nextEdge->type() == TaxiEdge::Type::Runway)
                            {
                                result = edge;
                                host()->writeLog(
                                    "AIPILO|Flight[%s] findFinalDepartureHoldShortEdge: HoldShort edge fallback selected edge[%d|%s]",
                                    flight()->callSign().c_str(),
                                    edge->id(),
                                    edge->name().c_str());
                                break;
                            }
                        }
                    }

                    // Final fallback for synthetic lineup extensions (edge id < 0):
                    // use the last real edge before synthetic extension as hold-short edge.
                    if (!result)
                    {
                        for (int i = 1; i < static_cast<int>(taxiPath->edges.size()); ++i)
                        {
                            auto edge = taxiPath->edges[i];
                            auto prevEdge = taxiPath->edges[i - 1];
                            if (edge && prevEdge && edge->id() < 0 && prevEdge->id() >= 0)
                            {
                                result = prevEdge;
                                host()->writeLog(
                                    "AIPILO|Flight[%s] findFinalDepartureHoldShortEdge: synthetic fallback selected edge[%d|%s] before synthetic edge[%d|%s]",
                                    flight()->callSign().c_str(),
                                    prevEdge->id(),
                                    prevEdge->name().c_str(),
                                    edge->id(),
                                    edge->name().c_str());
                                break;
                            }
                        }
                    }

                    // Last resort: use the last edge before the runway edge, regardless of type
                    if (!result)
                    {
                        for (int i = 0; i < (int)taxiPath->edges.size() - 1; i++)
                        {
                            auto edge = taxiPath->edges[i];
                            auto nextEdge = taxiPath->edges[i + 1];

                            if (edge && nextEdge && nextEdge->type() == TaxiEdge::Type::Runway)
                            {
                                result = edge;
                                host()->writeLog(
                                    "AIPILO|Flight[%s] findFinalDepartureHoldShortEdge: last resort fallback selected edge[%d|%s]",
                                    flight()->callSign().c_str(),
                                    edge->id(),
                                    edge->name().c_str());
                                break;
                            }
                        }
                    }

                    if (result)
                    {
                        host()->writeLog(
                            "AIPILO|Flight[%s] findFinalDepartureHoldShortEdge: selected edge[%d|%s]",
                            flight()->callSign().c_str(),
                            result->id(),
                            result->name().c_str());
                    }
                    else
                    {
                        host()->writeLog(
                            "AIPILO|Flight[%s] findFinalDepartureHoldShortEdge: NO hold short edge found!",
                            flight()->callSign().c_str());
                    }

                    return result;
                };

                auto finalDepartureHoldShortEdge = findFinalDepartureHoldShortEdge(clearance->taxiPath());

                const auto onHoldingShort = [=](shared_ptr<TaxiEdge> holdShortEdge) {
                    bool isFinalFallbackEdge = finalDepartureHoldShortEdge && holdShortEdge == finalDepartureHoldShortEdge;
                    bool isHoldingShortDepartureRunway =
                        edgeMatchesDepartureRunway(holdShortEdge) ||
                        isFinalFallbackEdge;

                    host()->writeLog(
                        "AIPILO|Flight[%s] onHoldingShort edge[%d|%s] matchesDeparture[%d] finalHoldShortEdge[%p]",
                        flight()->callSign().c_str(),
                        holdShortEdge->id(),
                        holdShortEdge->name().c_str(),
                        isHoldingShortDepartureRunway,
                        finalDepartureHoldShortEdge.get());

                    if (isHoldingShortDepartureRunway)
                    {
                        bool isFinalDepartureHoldShort = (!finalDepartureHoldShortEdge || holdShortEdge == finalDepartureHoldShortEdge);
                        if (isFinalDepartureHoldShort)
                        {
                            host()->writeLog(
                                "AIPILO|Flight[%s] calling maneuverDepartureAwaitLineup for edge[%d]",
                                flight()->callSign().c_str(),
                                holdShortEdge->id());
                            return maneuverDepartureAwaitLineup(m_flightPlan->departureRunway(), holdShortEdge);
                        }

                        host()->writeLog(
                            "AIPILO|Flight[%s] skipping early departure hold-short edge[%d], proceeding to final hold-short",
                            flight()->callSign().c_str(),
                            holdShortEdge->id());
                        return M.instantAction([]{});
                    }

                    host()->writeLog(
                        "AIPILO|Flight[%s] treating edge[%d] as runway cross (not departure runway)",
                        flight()->callSign().c_str(),
                        holdShortEdge->id());
                    return maneuverAwaitCrossRunway(m_departureAirport, holdShortEdge);
                };

                vector<shared_ptr<Maneuver>> steps;
                steps.push_back(M.delay(chrono::seconds(10)));
                steps.push_back(M.switchLights(flight(), Aircraft::LightBits::BeaconTaxi));
                steps.push_back(M.delay(chrono::seconds(5)));
                steps.push_back(M.taxiByPath(
                    flight(),
                    clearance->taxiPath(),
                    ManeuverFactory::TaxiType::Normal,
                    onHoldingShort));

                return shared_ptr<Maneuver>(new SequentialManeuver(
                    Maneuver::Type::DepartureTaxi,
                    "",
                    steps
                ));
            });
        }

        shared_ptr<Maneuver> maneuverHelipadDeparture()
        {
            if (!m_departureAirport || !m_flightPlan || m_flightPlan->departureGate().empty())
            {
                return M.instantAction([]{});
            }

            auto departureStand = m_departureAirport->getParkingStandOrThrow(m_flightPlan->departureGate());
            const float departureHeading = helipadDepartureHeading(departureStand);
            const float liftoffRocFpm = max(250.0f, m_aircraft->performanceProfile().initialClimbRocFpm * 0.35f);
            const float climboutRocFpm = max(liftoffRocFpm + 150.0f, m_aircraft->performanceProfile().initialClimbRocFpm * 0.55f);
            const float transitionSpeedKt = max(40.0f, min(80.0f, m_aircraft->performanceProfile().takeoffInitialClimbSpeedKt));
            const float climboutSpeedKt = max(55.0f, max(transitionSpeedKt, m_aircraft->performanceProfile().approachSpeedKt));

            return M.sequence(Maneuver::Type::DepartureClimbInitialHeading, "heli_departure", {
                M.instantAction([this, departureStand, departureHeading, liftoffRocFpm]() {
                    flight()->setPhase(Flight::Phase::Departure);
                    host()->writeLog(
                        "AIPILO|Flight[%s] helicopter vertical departure from stand[%s]",
                        flight()->callSign().c_str(),
                        departureStand->name().c_str());
                    m_aircraft->setLights(Aircraft::LightBits::BeaconLandingNavStrobe);
                    m_aircraft->setGroundSpeedKt(0.0f);
                    m_aircraft->setTrack(departureHeading);
                    m_aircraft->setAttitude(AircraftAttitude(departureHeading, 4.0f, 0.0f));
                    m_aircraft->setVerticalSpeedFpm(liftoffRocFpm);
                }, "heli_liftoff"),
                M.await(Maneuver::Type::DepartureClimbInitialHeading, "await_heli_liftoff", [this]() {
                    return m_aircraft->altitude().type() != Altitude::Type::Ground;
                }),
                M.delay(chrono::seconds(2)),
                M.instantAction([this, departureHeading, transitionSpeedKt, climboutRocFpm]() {
                    m_aircraft->setTrack(departureHeading);
                    m_aircraft->setAttitude(AircraftAttitude(departureHeading, 6.0f, 0.0f));
                    m_aircraft->setGroundSpeedKt(transitionSpeedKt);
                    m_aircraft->setVerticalSpeedFpm(climboutRocFpm);
                }, "heli_transition"),
                M.await(Maneuver::Type::DepartureClimbInitialHeading, "await_heli_safe_altitude", [this]() {
                    return currentAglFeet() >= 250.0;
                }),
                M.instantAction([this, departureHeading, climboutSpeedKt, climboutRocFpm]() {
                    m_aircraft->setTrack(departureHeading);
                    m_aircraft->setAttitude(AircraftAttitude(departureHeading, 5.0f, 0.0f));
                    m_aircraft->setGroundSpeedKt(climboutSpeedKt);
                    m_aircraft->setVerticalSpeedFpm(climboutRocFpm);
                }, "heli_climbout")
            });
        }

        shared_ptr<Maneuver> maneuverDepartureHovertaxi()
        {
            // Helicopter hovertaxi: lift off stand, fly directly to hold-short or lineup point
            return DeferredManeuver::create(Maneuver::Type::DepartureTaxi, "hovertaxi", [=]() {
                auto clearance = flight()->findClearanceOrThrow<DepartureTaxiClearance>(Clearance::Type::DepartureTaxiClearance);
                auto departureRunway = m_departureAirport->getRunwayOrThrow(m_flightPlan->departureRunway());
                const auto& runwayEnd = departureRunway->getEndOrThrow(m_flightPlan->departureRunway());
                const GeoPoint runwayCenterline = runwayEnd.centerlinePoint().geo();
                float runwayHeading = runwayEnd.heading();

                // Calculate hold-short point (150m from runway centerline)
                auto holdShortPoint = GeoMath::getPointAtDistance(runwayCenterline, GeoMath::flipHeading(runwayHeading), 150);
                const double liftoffHeightFeet = 30.0;
                const double hoverTaxiSpeedKt = 20.0;
                const double distanceMeters = GeoMath::getDistanceMeters(m_aircraft->location(), holdShortPoint);
                const double hoverTaxiDurationSec = max(8.0, min(45.0, distanceMeters / (hoverTaxiSpeedKt * METERS_IN_1_NAUTICAL_MILE / 3600.0)));

                host()->writeLog(
                    "AIPILO|Flight[%s] helicopter hovertaxi from stand to hold-short [%.6f,%.6f]",
                    flight()->callSign().c_str(),
                    holdShortPoint.latitude,
                    holdShortPoint.longitude);

                vector<shared_ptr<Maneuver>> steps;

                // Lift off from stand to 30 feet AGL
                steps.push_back(M.instantAction([=]() {
                    m_aircraft->setAttitude(m_aircraft->attitude().withPitch(0.0f));
                }));
                steps.push_back(shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "lift_off",
                    m_aircraft->location().altitude,
                    m_aircraft->location().altitude + liftoffHeightFeet,
                    chrono::seconds(5),
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress;
                    },
                    [=](const double& value, double progress) {
                        auto location = m_aircraft->location();
                        location.altitude = value;
                        m_aircraft->setLocation(location);
                    }
                )));

                // Hover-taxi to hold-short point at 30ft, 20 knots
                GeoPoint hoverTaxiStart = m_aircraft->location();
                GeoPoint hoverTaxiEnd = holdShortPoint;
                hoverTaxiEnd.altitude = hoverTaxiStart.altitude + liftoffHeightFeet;
                steps.push_back(shared_ptr<Maneuver>(new AnimationManeuver<GeoPoint>(
                    "hover_taxi",
                    hoverTaxiStart,
                    hoverTaxiEnd,
                    chrono::milliseconds(static_cast<int>(hoverTaxiDurationSec * 1000.0)),
                    [](const GeoPoint& from, const GeoPoint& to, double progress, GeoPoint& value) {
                        value.latitude = from.latitude + (to.latitude - from.latitude) * progress;
                        value.longitude = from.longitude + (to.longitude - from.longitude) * progress;
                        value.altitude = from.altitude + (to.altitude - from.altitude) * progress;
                    },
                    [=](const GeoPoint& value, double progress) {
                        m_aircraft->setLocation(value);
                        m_aircraft->setGroundSpeedKt(20.0);
                        // Point toward runway
                        float headingToRunway = GeoMath::getHeadingFromPoints(m_aircraft->location(), runwayCenterline);
                        m_aircraft->setAttitude(m_aircraft->attitude().withHeading(headingToRunway));
                    }
                )));

                // Align with runway heading and hold short
                steps.push_back(M.instantAction([=]() {
                    m_aircraft->setAttitude(m_aircraft->attitude().withHeading(runwayHeading).withRoll(0.0f));
                    m_aircraft->setGroundSpeedKt(0);
                    host()->writeLog(
                        "AIPILO|Flight[%s] helicopter holding short runway %s",
                        flight()->callSign().c_str(),
                        m_flightPlan->departureRunway().c_str());
                }));

                return M.sequence(Maneuver::Type::DepartureTaxi, "hovertaxi_seq", steps);
            });
        }

        shared_ptr<Maneuver> maneuverHelicopterRunwayDeparture()
        {
            return DeferredManeuver::create(Maneuver::Type::DepartureClimbInitialHeading, "heli_runway_departure", [=]() {
                auto clearance = flight()->findClearanceOrThrow<TakeoffClearance>(Clearance::Type::TakeoffClearance);
                auto runway = m_departureAirport->getRunwayOrThrow(clearance->departureRunway());
                const auto& runwayEnd = runway->getEndOrThrow(clearance->departureRunway());
                const float runwayHeading = runwayEnd.heading();
                const float initialHeading = clearance->initialHeading();
                const auto performance = m_aircraft->performanceProfile();

                const float initialClimbRocFpm = max(450.0f, performance.initialClimbRocFpm * 0.35f);
                const float departureClimbRocFpm = max(650.0f, performance.initialClimbRocFpm * 0.5f);
                const float hoverTransitionSpeedKt = max(25.0f, min(55.0f, performance.approachSpeedKt * 0.45f));
                const float departureSpeedKt = max(55.0f, min(110.0f, performance.takeoffInitialClimbSpeedKt));

                return M.sequence(Maneuver::Type::DepartureClimbInitialHeading, "heli_runway_departure_seq", {
                    M.instantAction([this, runwayHeading, initialClimbRocFpm]() {
                        flight()->setPhase(Flight::Phase::Departure);
                        m_aircraft->setLights(Aircraft::LightBits::BeaconLandingNavStrobe);
                        m_aircraft->setGroundSpeedKt(0.0f);
                        m_aircraft->setTrack(runwayHeading);
                        m_aircraft->setAttitude(m_aircraft->attitude().withHeading(runwayHeading).withPitch(5.0f).withRoll(0.0f));
                        m_aircraft->setVerticalSpeedFpm(initialClimbRocFpm);
                    }),
                    M.await(Maneuver::Type::DepartureClimbInitialHeading, "await_heli_runway_liftoff", [this]() {
                        return currentAglFeet() >= 35.0;
                    }),
                    M.parallel(Maneuver::Type::DepartureClimbInitialHeading, "heli_turnout", {
                        shared_ptr<Maneuver>(new AnimationManeuver<double>(
                            "heli_turnout_heading",
                            runwayHeading,
                            initialHeading,
                            chrono::seconds(5),
                            [](const double& from, const double& to, double progress, double& value) {
                                value = GeoMath::addTurnToHeading(from, GeoMath::getTurnDegrees(from, to) * progress);
                            },
                            [this](const double& value, double) {
                                m_aircraft->setTrack(value);
                                m_aircraft->setAttitude(m_aircraft->attitude().withHeading(value).withRoll(0.0f));
                            }
                        )),
                        shared_ptr<Maneuver>(new AnimationManeuver<double>(
                            "heli_turnout_speed",
                            0.0,
                            hoverTransitionSpeedKt,
                            chrono::seconds(5),
                            [](const double& from, const double& to, double progress, double& value) {
                                value = from + (to - from) * progress;
                            },
                            [this](const double& value, double) {
                                m_aircraft->setGroundSpeedKt(value);
                            }
                        ))
                    }),
                    M.await(Maneuver::Type::DepartureClimbInitialHeading, "await_heli_departure_altitude", [this]() {
                        return currentAglFeet() >= 180.0;
                    }),
                    M.instantAction([this, initialHeading, departureSpeedKt, departureClimbRocFpm]() {
                        m_aircraft->setTrack(initialHeading);
                        m_aircraft->setAttitude(m_aircraft->attitude().withHeading(initialHeading).withPitch(4.0f).withRoll(0.0f));
                        m_aircraft->setGroundSpeedKt(departureSpeedKt);
                        m_aircraft->setVerticalSpeedFpm(departureClimbRocFpm);
                    })
                });
            });
        }

        shared_ptr<Maneuver> maneuverDepartureAwaitLineup(const string& runwayName, shared_ptr<TaxiEdge> holdShortEdge)
        {
            return M.sequence(Maneuver::Type::Unspecified, "await_lineup", {
                M.instantAction([this]() {
                    m_departureNumberInLine = 1;
                }),
                M.taxiStop(flight()),
                M.deferred([=]() {
                    if (m_departureTowerKhz == 0)
                    {
                        auto reportHoldingShort = I.pilotReportHoldingShort(
                            flight(),
                            m_helper.getDepartureAirport(flight()),
                            runwayName,
                            holdShortEdge->name());

                        if (!reportHoldingShort)
                        {
                            return M.instantAction([]{});
                        }

                        return M.transmitIntent(flight(), reportHoldingShort, "", 1000, [this]{
                            if (m_departureTowerKhz > 0)
                            {
                                host()->writeLog("AIPILO|Flight[%s] CANCEL_TRANSMIT_HOLDING_SHORT", flight()->callSign().c_str());
                                return true;
                            }
                            return false;
                        });
                    }
                    return M.instantAction([]{});
                }),
                M.await(Maneuver::Type::Unspecified, "await_tower_khz", [this](){
                    return (m_departureTowerKhz > 0);
                }),
//                M.deferred([=]() {
//                    auto ground = m_departureAirport->groundAt(flight()->aircraft()->location());
//                    return M.transmitIntent(flight(), I.pilotHandoffReadback(flight(), ground, m_departureTowerKhz, m_lastReceivedIntentId));
//                }),
                M.instantAction([=]() {
                    aircraft()->setFrequencyKhz(m_departureTowerKhz);
                    m_lastDeclineReason = DeclineReason::None;
                    m_holdShortForDeparture = false;
                }),
                M.transmitIntent(flight(), I.pilotCheckInWithTower(flight(), runwayName, holdShortEdge->name(), false)),
                M.await(Maneuver::Type::AwaitClearance, "await_any_luaw_clrnc_holdshrt", [this]{
                    auto clearance = flight()->tryFindClearance<TakeoffClearance>(Clearance::Type::TakeoffClearance);
                    auto luaw = flight()->tryFindClearance<LineUpAndWaitApproval>(Clearance::Type::LineUpAndWait);
                    return (clearance || luaw || m_holdShortForDeparture);
                }),
                M.deferred([this, holdShortEdge] {
                    if (!m_holdShortForDeparture || !holdShortEdge)
                    {
                        return M.instantAction([]{});
                    }

                    const int numberInLine = max(1, m_departureNumberInLine);
                    if (numberInLine <= 1)
                    {
                        return M.instantAction([this, holdShortEdge]() {
                            m_aircraft->hardStop(holdShortEdge->heading());
                        });
                    }

                    // Queue spacing scaled by aircraft category (wingspan proxy).
                    const float baseSpacingMeters = isHelicopter() ? 22.0f
                        : (isFighter() ? 32.0f
                        : (m_aircraft->category() == Aircraft::Category::Heavy ? 65.0f
                        : (m_aircraft->category() == Aircraft::Category::LightProp ? 25.0f
                        : (m_aircraft->category() == Aircraft::Category::Turboprop ? 35.0f
                        : 45.0f))));
                    const float queueOffsetMeters = baseSpacingMeters * static_cast<float>(numberInLine - 1);
                    const GeoPoint queuePoint = GeoMath::getPointAtDistance(
                        holdShortEdge->node1()->location().geo(),
                        GeoMath::flipHeading(holdShortEdge->heading()),
                        queueOffsetMeters);
                    const float distanceToQueueMeters = GeoMath::getDistanceMeters(m_aircraft->location(), queuePoint);

                    host()->writeLog(
                        "AIPILO|Flight[%s] hold-short queue slot[%d] offset[%.0fm] current-distance[%.0fm]",
                        flight()->callSign().c_str(),
                        numberInLine,
                        queueOffsetMeters,
                        distanceToQueueMeters);

                    if (distanceToQueueMeters < 3.0f)
                    {
                        return M.instantAction([this, holdShortEdge]() {
                            m_aircraft->hardStop(holdShortEdge->heading());
                        });
                    }

                    return M.sequence(Maneuver::Type::TaxiHoldShort, "move_to_queue_slot", {
                        M.taxiStraight(flight(), m_aircraft->location(), queuePoint, ManeuverFactory::TaxiType::Normal),
                        M.instantAction([this, holdShortEdge]() {
                            m_aircraft->hardStop(holdShortEdge->heading());
                        })
                    });
                }),
                M.deferred([this, runwayName]{
                    if (m_holdShortForDeparture)
                    {
                        return M.transmitIntent(flight(), I.pilotRunwayHoldShortReadback(
                            flight(),
                            m_helper.getDepartureTower(flight()),
                            runwayName,
                            m_lastDeclineReason,
                            m_lastReceivedIntentId));
                    }
                    return M.instantAction([]{});
                }),
                M.await(Maneuver::Type::AwaitClearance, "await_luaw_or_takeoff_clrnc", [this]{
                    auto clearance = flight()->tryFindClearance<TakeoffClearance>(Clearance::Type::TakeoffClearance);
                    auto luaw = flight()->tryFindClearance<LineUpAndWaitApproval>(Clearance::Type::LineUpAndWait);
                    return (clearance || luaw);
                }),
//                M.deferred([=]() {
//                    auto clearance = flight()->tryFindClearance<TakeoffClearance>(Clearance::Type::TakeoffClearance);
//                    auto luaw = flight()->tryFindClearance<LineUpAndWaitApproval>(Clearance::Type::LineUpAndWait);
//                    auto readback = clearance
//                        ? I.pilotTakeoffClearanceReadback(flight(), clearance, m_departureKhz, m_lastReceivedIntentId)
//                        : I.pilotLineUpAndWaitReadback(luaw, m_lastReceivedIntentId);
//                    m_wasTakeoffClearanceReadBack = (readback->code() == PilotTakeoffClearanceReadbackIntent::IntentCode);
//                    return M.transmitIntent(flight(), readback);
//                }),
                //M.delay(chrono::seconds(3))
            });
        }

        shared_ptr<Maneuver> maneuverAwaitCrossRunway(shared_ptr<Airport> airport, shared_ptr<TaxiEdge> holdShortEdge)
        {
            auto runway = getActiveZoneRunway(airport, holdShortEdge);
            string runwayName = runway ? runway->end1().name() : "";
            auto reportHoldingShort = I.pilotReportHoldingShort(flight(), runwayName, holdShortEdge->name());

            if (!reportHoldingShort)
            {
                return M.sequence(Maneuver::Type::TaxiHoldShort, "await_cross_rwy", {
                    M.taxiStop(flight()),
                    M.instantAction([this]{
                        m_lastDeclineReason = DeclineReason::None;
                        flight()->removeClearance(Clearance::Type::RunwayCrossClearance);
                    }),
                });
            }

            return M.sequence(Maneuver::Type::TaxiHoldShort, "await_cross_rwy", {
                M.taxiStop(flight()),
                M.instantAction([this]{
                    m_lastDeclineReason = DeclineReason::None;
                    flight()->removeClearance(Clearance::Type::RunwayCrossClearance);
                }),
                M.deferred([=]() {
                    return M.transmitIntent(flight(), reportHoldingShort);
                }),
                M.await(Maneuver::Type::Unspecified, "", [this]{
                    auto clearance = flight()->tryFindClearance<RunwayCrossClearance>(Clearance::Type::RunwayCrossClearance);
                    return (clearance || m_lastDeclineReason != DeclineReason::None);
                }),
                M.deferred([this, runwayName, holdShortEdge, reportHoldingShort]{
                    if (m_lastDeclineReason != DeclineReason::None)
                    {
                        return M.transmitIntent(flight(), I.pilotRunwayHoldShortReadback(
                            flight(),
                            reportHoldingShort->subjectControl(), runwayName,
                            m_lastDeclineReason,
                            m_lastReceivedIntentId));
                    }
                    return M.instantAction([]{});
                }),
                M.awaitClearance(flight(), Clearance::Type::RunwayCrossClearance),
                M.deferred([=]() {
                    auto clearance = flight()->findClearanceOrThrow<RunwayCrossClearance>(Clearance::Type::RunwayCrossClearance);
                    return M.transmitIntent(flight(), I.pilotRunwayCrossReadback(clearance, m_lastReceivedIntentId));
                }),
                M.instantAction([this]{
                    flight()->removeClearance(Clearance::Type::RunwayCrossClearance);
                })
            });
        }

        shared_ptr<Maneuver> maneuverLineup()
        {
            return M.sequence(Maneuver::Type::DepartureTaxi, "lineup", {
                M.instantAction([this]() {
                    m_linedUpTimestamp = host()->getWorld()->timestamp();
                    host()->writeLog(
                        "AIPILO|Flight[%s] lining up on runway",
                        flight()->callSign().c_str());
                }),
                M.deferred([this]() {
                    auto clearance = flight()->findClearanceOrThrow<TakeoffClearance>(Clearance::Type::TakeoffClearance);
                    auto runway = m_departureAirport->getRunwayOrThrow(clearance->departureRunway());
                    const auto& runwayEnd = runway->getEndOrThrow(clearance->departureRunway());
                    const GeoPoint runwayStart = runwayEnd.centerlinePoint().geo();
                    float runwayHeading = runwayEnd.heading();

                    // Move to runway centerline if not already there
                    const float distanceToRunwayStart = GeoMath::getDistanceMeters(m_aircraft->location(), runwayStart);
                    if (distanceToRunwayStart > 5.0f)
                    {
                        host()->writeLog(
                            "AIPILO|Flight[%s] moving to runway centerline, distance=%.0fm",
                            flight()->callSign().c_str(),
                            distanceToRunwayStart);
                        return M.taxiStraight(flight(), m_aircraft->location(), runwayStart, ManeuverFactory::TaxiType::Normal);
                    }
                    return M.instantAction([]{});
                }),
                M.instantAction([this]() {
                    auto clearance = flight()->findClearanceOrThrow<TakeoffClearance>(Clearance::Type::TakeoffClearance);
                    auto runway = m_departureAirport->getRunwayOrThrow(clearance->departureRunway());
                    const auto& runwayEnd = runway->getEndOrThrow(clearance->departureRunway());
                    float runwayHeading = runwayEnd.heading();
                    // Hard-stop and sync track immediately so the aircraft does not drift
                    // off the runway centerline during the before-takeoff checklist wait.
                    m_aircraft->hardStop(runwayHeading);
                })
            });
        }

        shared_ptr<Maneuver> maneuverAwaitTakeOffClearance()
        {
            return M.sequence(Maneuver::Type::DepartureAwaitTakeOff, "await_takeoff_clrnc", {
                M.await(Maneuver::Type::AwaitClearance, "await_hold_short_clear", [this]{
                    return !m_holdShortForDeparture;
                }),
                M.await(Maneuver::Type::AwaitClearance, "await_luaw_or_takeoff_clrnc", [this]{
                    auto clearance = flight()->tryFindClearance<TakeoffClearance>(Clearance::Type::TakeoffClearance);
                    auto luaw = flight()->tryFindClearance<LineUpAndWaitApproval>(Clearance::Type::LineUpAndWait);
                    return (clearance || luaw);
                }),
                M.instantAction([this]() {
                    auto takeoffClearance = flight()->tryFindClearance<TakeoffClearance>(Clearance::Type::TakeoffClearance);
                    auto luaw = flight()->tryFindClearance<LineUpAndWaitApproval>(Clearance::Type::LineUpAndWait);

                    // Do not switch strobe/landing while holding short.
                    // Strobes come on after lineup/takeoff clearance;
                    // landing lights come on only after takeoff clearance.
                    if (takeoffClearance)
                    {
                        m_aircraft->setLights(Aircraft::LightBits::BeaconLandingNavStrobe);
                    }
                    else if (luaw)
                    {
                        m_aircraft->setLights(Aircraft::LightBits::BeaconNavStrobe);
                    }
                }),
                M.deferred([=]() {
                    auto clearance = flight()->tryFindClearance<TakeoffClearance>(Clearance::Type::TakeoffClearance);
                    auto luaw = flight()->tryFindClearance<LineUpAndWaitApproval>(Clearance::Type::LineUpAndWait);
                    auto readback = clearance
                        ? I.pilotTakeoffClearanceReadback(flight(), clearance, m_departureKhz, m_lastReceivedIntentId)
                        : I.pilotLineUpAndWaitReadback(luaw, m_lastReceivedIntentId);
                    m_wasTakeoffClearanceReadBack = (readback->code() == PilotTakeoffClearanceReadbackIntent::IntentCode);
                    return M.transmitIntent(flight(), readback);
                }),
                M.awaitClearance(flight(), Clearance::Type::TakeoffClearance),
                M.instantAction([this]() {
                    // Ensure landing lights come on once takeoff clearance is active,
                    // even if we previously only had line-up-and-wait.
                    m_aircraft->setLights(Aircraft::LightBits::BeaconLandingNavStrobe);
                }),
                M.deferred([=]() {
                    auto clearance = flight()->findClearanceOrThrow<TakeoffClearance>(Clearance::Type::TakeoffClearance);
                    if (!m_wasTakeoffClearanceReadBack)
                    {
                        auto readback = I.pilotTakeoffClearanceReadback(flight(), clearance, m_departureKhz, m_lastReceivedIntentId);
                        m_wasTakeoffClearanceReadBack = true;
                        return M.transmitIntent(flight(), readback);
                    }
                    return M.instantAction([]{});
                }),
            });
        }

        shared_ptr<Maneuver> maneuverTakeoff()
        {
            return DeferredManeuver::create(Maneuver::Type::DepartureTakeOffRoll, "takeoff", [=]() {
                auto clearance = flight()->findClearanceOrThrow<TakeoffClearance>(Clearance::Type::TakeoffClearance);
                auto luaw = flight()->tryFindClearance<LineUpAndWaitApproval>(Clearance::Type::LineUpAndWait);
                auto runway = m_departureAirport->getRunwayOrThrow(clearance->departureRunway());
                const auto& runwayEnd = runway->getEndOrThrow(clearance->departureRunway());
                float runwayHeading = runwayEnd.heading();
                const auto performance = m_aircraft->performanceProfile();
                const float rotateTargetPitch = max(6.0f, performance.takeoffRotatePitchDegrees);
                const float rotatePhase1Pitch = max(4.0f, rotateTargetPitch * 0.6f);
                const float currentGroundSpeedKt = static_cast<float>(m_aircraft->groundSpeedKt());
                // If stopped (speed <= 1 kt) or explicitly waited for checklist, start from 0.
                // Otherwise use current rolling speed (capped at 35% of liftoff speed)
                // for aircraft that enter the runway while still rolling.
                const float runwayEntrySpeedKt = (m_stoppedBeforeTakeoff || currentGroundSpeedKt <= 1.0f)
                    ? 0.0f
                    : max(20.0f, min(currentGroundSpeedKt, performance.takeoffLiftOffSpeedKt * 0.35f));
                const float rotateSpeedKt = max(runwayEntrySpeedKt + 10.0f, performance.takeoffRotateSpeedKt);
                const float liftOffSpeedKt = max(rotateSpeedKt + 5.0f, performance.takeoffLiftOffSpeedKt);
                const float initialClimbSpeedKt = max(liftOffSpeedKt + 10.0f, performance.takeoffInitialClimbSpeedKt);
                const float takeoffAccelerationKtPerSecond = max(2.5f, performance.takeoffAccelerationKtPerSecond);
                const auto secondsForSpeedDelta = [](float fromSpeedKt, float toSpeedKt, float accelerationKtPerSecond, float minimumSeconds) {
                    const float deltaSpeedKt = max(0.0f, toSpeedKt - fromSpeedKt);
                    const float durationSeconds = max(minimumSeconds, deltaSpeedKt / max(0.1f, accelerationKtPerSecond));
                    return chrono::milliseconds(static_cast<int>(durationSeconds * 1000.0f));
                };
                const auto rotationDelay = secondsForSpeedDelta(runwayEntrySpeedKt, rotateSpeedKt, takeoffAccelerationKtPerSecond, 4.0f);

                // Calculate ground roll duration using Eurocontrol takeoff distance if available
                // Eurocontrol takeoffDistanceMeters is total distance from brake release to 35ft
                // Ground roll is approximately 80% of total takeoff distance
                auto groundRollDuration = secondsForSpeedDelta(runwayEntrySpeedKt, liftOffSpeedKt, takeoffAccelerationKtPerSecond, 6.0f);
                if (performance.takeoffDistanceMeters > 500.0f)
                {
                    // Estimate ground roll as ~80% of total takeoff distance (to 35ft)
                    const float groundRollDistanceMeters = performance.takeoffDistanceMeters * 0.8f;
                    // Calculate ground roll time based on average speed during acceleration
                    // Average speed = (entry + liftoff) / 2 in m/s (knots * 1852 / 3600)
                    const float avgGroundSpeedMps = ((runwayEntrySpeedKt + liftOffSpeedKt) / 2.0f) * 1852.0f / 3600.0f;
                    const float calculatedGroundRollSeconds = groundRollDistanceMeters / max(0.1f, avgGroundSpeedMps);
                    const int groundRollMillis = static_cast<int>(max(5.0f, calculatedGroundRollSeconds) * 1000.0f);
                    groundRollDuration = chrono::milliseconds(min(groundRollMillis, 30000)); // Cap at 30 seconds
                }
                const auto airborneAccelerationDuration = secondsForSpeedDelta(
                    liftOffSpeedKt,
                    initialClimbSpeedKt,
                    max(1.5f, takeoffAccelerationKtPerSecond * 0.45f),
                    8.0f);
                const auto rotation1Duration = chrono::milliseconds(max(1500, min(3500, static_cast<int>(rotationDelay.count() / 3))));
                const auto rotation2Duration = chrono::milliseconds(max(2500, min(6500, static_cast<int>(groundRollDuration.count() / 2))));
                const auto gearUpDelay = groundRollDuration + chrono::seconds(2);
                const auto turnDelay = groundRollDuration + chrono::seconds(8);
                const double initialClimbRocFpm = max(
                    900.0,
                    min(8000.0, static_cast<double>(performance.initialClimbRocFpm) * (m_aircraft->category() == Aircraft::Category::Fighter ? 0.45 : 0.75)));
                const auto liftUpDuration = chrono::milliseconds(max(7000, min(14000, static_cast<int>(initialClimbSpeedKt * 90.0f))));

                auto beforeTakeoffChecklist = M.await(Maneuver::Type::Unspecified, "bfr_tkoff_chklst", [=]() {
                    auto now = host()->getWorld()->timestamp();
                    auto elapsed = now - m_linedUpTimestamp;
                    if (elapsed > chrono::milliseconds(100) && !m_stoppedBeforeTakeoff)
                    {
                        m_stoppedBeforeTakeoff = true;
                        host()->writeLog(
                            "AIPILO|BEFORE-TAKEOFF Flight[%s] stopped for before-takeoff checklist lined-up[%lld] now[%lld] elapsed[%lld]",
                            flight()->callSign().c_str(),
                            m_linedUpTimestamp,
                            now,
                            elapsed);
                    }
                    return (clearance->immediate() || elapsed >= chrono::seconds(3));
                });
                auto logTakeoffRoll = M.instantAction([this, rotateSpeedKt, liftOffSpeedKt, initialClimbSpeedKt, takeoffAccelerationKtPerSecond](){
                    const auto& perf = m_aircraft->performanceProfile();
                    if (perf.takeoffDistanceMeters > 500.0f)
                    {
                        host()->writeLog(
                            "AIPILO|TAKEOFFROLL flight[%s] at [%f,%f] stopped[%d] vr[%.0f] vlof[%.0f] climb[%.0f] accel[%.1f] tod[%0.fm]",
                            flight()->callSign().c_str(),
                            m_aircraft->location().latitude,
                            m_aircraft->location().longitude,
                            m_stoppedBeforeTakeoff ? 1 : 0,
                            rotateSpeedKt,
                            liftOffSpeedKt,
                            initialClimbSpeedKt,
                            takeoffAccelerationKtPerSecond,
                            perf.takeoffDistanceMeters);
                    }
                    else
                    {
                        host()->writeLog(
                            "AIPILO|TAKEOFFROLL flight[%s] at [%f,%f] stopped[%d] vr[%.0f] vlof[%.0f] climb[%.0f] accel[%.1f]",
                            flight()->callSign().c_str(),
                            m_aircraft->location().latitude,
                            m_aircraft->location().longitude,
                            m_stoppedBeforeTakeoff ? 1 : 0,
                            rotateSpeedKt,
                            liftOffSpeedKt,
                            initialClimbSpeedKt,
                            takeoffAccelerationKtPerSecond);
                    }
                });
                auto rollOnRunway = M.deferred([this, runwayEntrySpeedKt, liftOffSpeedKt, groundRollDuration]() {
                    return shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "roll",
                        runwayEntrySpeedKt,
                        liftOffSpeedKt,
                        groundRollDuration,
                        [](const double &from, const double &to, double progress, double &value) {
                            value = from + (to - from) * progress;
                        },
                        [this](const double &value, double progress) {
                            m_aircraft->setGroundSpeedKt(value);
                        }
                    ));
                });
                auto rotate1 = shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "rotate_1",
                    0,
                    rotatePhase1Pitch,
                    rotation1Duration,
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress; 
                    },
                    [=](const double& value, double progress) {
                        m_aircraft->setAttitude(m_aircraft->attitude().withPitch(value));
                    }
                ));
                auto rotate2 = shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "rotate_2",
                    rotatePhase1Pitch,
                    rotateTargetPitch,
                    rotation2Duration,
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress; 
                    },
                    [this](const double& value, double progress) {
                        m_aircraft->setAttitude(m_aircraft->attitude().withPitch(value));
                    }
                ));
                auto logLiftUp = M.instantAction([this](){
                    host()->writeLog(
                        "AIPILO|LIFTUP flight[%s] at [%f,%f] ground-speed[%f]kt pitch[%f]deg",
                        flight()->callSign().c_str(),
                        m_aircraft->location().latitude,
                        m_aircraft->location().longitude,
                        m_aircraft->groundSpeedKt(),
                        m_aircraft->attitude().pitch());
                });
                auto liftUp = shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "lift_up",
                    0,
                    initialClimbRocFpm,
                    liftUpDuration,
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress; 
                    },
                    [this](const double& value, double progress) {
                        m_aircraft->setVerticalSpeedFpm(value);
                        //aircraft->setAltitude(value);
                    }
                ));
                auto gearUp = shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "gear_up",
                    1.0,
                    0.0,
                    chrono::seconds(8),
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress; 
                    },
                    [this](const double& value, double progress) {
                        m_aircraft->setGearState(value);
                    }
                ));
                auto accelerateAirborne = shared_ptr<Maneuver>(new AnimationManeuver<double>(
                    "accel_airb",
                    liftOffSpeedKt,
                    initialClimbSpeedKt,
                    airborneAccelerationDuration,
                    [](const double& from, const double& to, double progress, double& value) {
                        value = from + (to - from) * progress; 
                    },
                    [this](const double& value, double progress) {
                        m_aircraft->setGroundSpeedKt(value);
                    }
                ));
                return M.sequence(Maneuver::Type::Unspecified, "", {
                    M.instantAction([this, runway]() {
                        flight()->setPhase(Flight::Phase::Departure);
                        const auto& runwayEnd = runway->getEndOrThrow(m_flightPlan->departureRunway());
                        m_aircraft->setAttitude(m_aircraft->attitude().withHeading(runwayEnd.heading()));
                    }),
                    beforeTakeoffChecklist,
                    logTakeoffRoll,
                    M.parallel(Maneuver::Type::Unspecified, "", {
                        M.sequence(Maneuver::Type::Unspecified, "", {
                            rollOnRunway,
                            accelerateAirborne,
                        }),
                        M.sequence(Maneuver::Type::Unspecified, "", {
                            M.delay(rotationDelay),
                            rotate1,
                            rotate2,
                        }),
                        M.sequence(Maneuver::Type::Unspecified, "", {
                            M.delay(groundRollDuration),
                            logLiftUp,
                            liftUp
                        }),
                        M.sequence(Maneuver::Type::Unspecified, "", {
                            M.delay(gearUpDelay),
                            gearUp,
                        }),
                        M.sequence(Maneuver::Type::Unspecified, "", {
                            M.delay(turnDelay),
                            M.airborneTurn(flight(), runwayHeading, clearance->initialHeading()),
                        }),
                    })
                });
            });
        }

        shared_ptr<Maneuver> maneuverUnrestrictedClimbout()
        {
            return DeferredManeuver::create(Maneuver::Type::DepartureClimbBySid, "unrestricted_climbout_core", [this]() {
                float targetAltitudeFeet = 10000.0f;
                if (m_flightPlan)
                {
                    for (const auto& leg : m_flightPlan->legs())
                    {
                        if (leg && leg->type() == FlightPlan::LegType::Sid && leg->targetAltitude() > 0)
                        {
                            targetAltitudeFeet = leg->targetAltitude();
                            break;
                        }
                    }

                    if (m_flightPlan->cruiseAltitudeFeet() > 0.0f)
                    {
                        targetAltitudeFeet = min(targetAltitudeFeet, m_flightPlan->cruiseAltitudeFeet());
                    }
                }

                const auto performance = m_aircraft->performanceProfile();
                const auto missionProfile = m_aircraft->missionProfile();
                const bool lowLevelMission = missionProfile == AIAircraft::MissionProfile::LowLevel;
                const float departureElevationFeet = m_departureAirport ? m_departureAirport->header().elevation() : 0.0f;

                if (lowLevelMission)
                {
                    targetAltitudeFeet = min(targetAltitudeFeet, max(2500.0f, departureElevationFeet + 3500.0f));
                }
                else if (missionProfile == AIAircraft::MissionProfile::Training)
                {
                    targetAltitudeFeet = min(targetAltitudeFeet, max(7000.0f, departureElevationFeet + 8000.0f));
                }

                targetAltitudeFeet = max(targetAltitudeFeet, departureElevationFeet + 2000.0f);

                const float ratedClimbRateFpm = max(6000.0f, performance.initialClimbRocFpm);
                const float initialClimbRateFpm = lowLevelMission
                    ? min(9000.0f, max(3800.0f, ratedClimbRateFpm * 0.24f))
                    : min(18000.0f, max(5000.0f, ratedClimbRateFpm * 0.38f));
                const float sustainedClimbRateFpm = lowLevelMission
                    ? min(6500.0f, max(2800.0f, ratedClimbRateFpm * 0.16f))
                    : min(12000.0f, max(3500.0f, ratedClimbRateFpm * 0.24f));
                const float finalClimbRateFpm = max(1800.0f, min(3500.0f, sustainedClimbRateFpm * 0.55f));

                const float cleanupSpeedKt = max(210.0f, min(lowLevelMission ? 300.0f : 320.0f, performance.approachSpeedKt + (lowLevelMission ? 110.0f : 135.0f)));
                const float climbSpeedKt = min(lowLevelMission ? 360.0f : 420.0f, max(cleanupSpeedKt + 35.0f, 250.0f + ratedClimbRateFpm / 250.0f));
                const float exitSpeedKt = min(430.0f, max(climbSpeedKt + 20.0f, lowLevelMission ? 320.0f : 340.0f));

                const float initialPitchDegrees = min(lowLevelMission ? 28.0f : 34.0f, max(22.0f, 20.0f + ratedClimbRateFpm / 3000.0f));
                const float transitionPitchDegrees = max(lowLevelMission ? 8.0f : 12.0f, initialPitchDegrees - (lowLevelMission ? 12.0f : 10.0f));
                const float exitPitchDegrees = lowLevelMission ? 3.0f : 5.0f;

                const float cleanupAltitudeAglFeet = lowLevelMission ? 1200.0f : 1800.0f;
                const float cleanupAltitudeFeet = departureElevationFeet + cleanupAltitudeAglFeet;
                const float transitionAltitudeFeet = min(
                    targetAltitudeFeet - 300.0f,
                    max(cleanupAltitudeFeet + 800.0f, departureElevationFeet + (lowLevelMission ? 3500.0f : 7000.0f)));

                const auto currentAltitudeMslFeet = [this]() {
                    const Altitude altitude = m_aircraft->altitude();
                    switch (altitude.type())
                    {
                    case Altitude::Type::Ground:
                        return host()->queryTerrainElevationAt(m_aircraft->location());
                    case Altitude::Type::AGL:
                        return altitude.feet() + host()->queryTerrainElevationAt(m_aircraft->location());
                    case Altitude::Type::MSL:
                    default:
                        return altitude.feet();
                    }
                };

                // Capture currentAltitudeMslFeet by VALUE, not by reference.
                // It is a local lambda; capturing by reference would leave a dangling
                // reference once the factory returns, causing 0xc0000005 on the first tick.
                const auto currentAltitudeAglFeet = [this, currentAltitudeMslFeet]() {
                    return currentAltitudeMslFeet() - host()->queryTerrainElevationAt(m_aircraft->location());
                };

                host()->writeLog(
                    "AIPILO|Flight[%s] unrestricted climbout mission[%d] targetAlt[%.0f] ratedROC[%.0f fpm] initialROC[%.0f fpm] sustainedROC[%.0f fpm]",
                    flight()->callSign().c_str(),
                    static_cast<int>(missionProfile),
                    targetAltitudeFeet,
                    ratedClimbRateFpm,
                    initialClimbRateFpm,
                    sustainedClimbRateFpm);

                vector<shared_ptr<Maneuver>> climboutSteps;
                climboutSteps.push_back(M.instantAction([this]() {
                    flight()->setPhase(Flight::Phase::Departure);
                    m_aircraft->setLights(Aircraft::LightBits::BeaconNavStrobe);
                }));
                climboutSteps.push_back(M.parallel(Maneuver::Type::DepartureClimbBySid, "fighter_climb_initial", {
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "fighter_pitch_initial",
                        m_aircraft->attitude().pitch(),
                        initialPitchDegrees,
                        chrono::seconds(3),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [this](const double& value, double) {
                            m_aircraft->setAttitude(m_aircraft->attitude().withPitch(value));
                        }
                    )),
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "fighter_vs_initial",
                        max(0.0, m_aircraft->verticalSpeedFpm()),
                        initialClimbRateFpm,
                        chrono::seconds(5),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [this](const double& value, double) {
                            m_aircraft->setVerticalSpeedFpm(value);
                        }
                    )),
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "fighter_speed_initial",
                        max(180.0, m_aircraft->groundSpeedKt()),
                        cleanupSpeedKt,
                        chrono::seconds(8),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [this](const double& value, double) {
                            m_aircraft->setGroundSpeedKt(value);
                        }
                    ))
                }));
                climboutSteps.push_back(M.await(Maneuver::Type::Unspecified, "fighter_cleanup_altitude", [=]() {
                    return currentAltitudeAglFeet() >= cleanupAltitudeAglFeet || currentAltitudeMslFeet() >= cleanupAltitudeFeet;
                }));
                climboutSteps.push_back(M.parallel(Maneuver::Type::DepartureClimbBySid, "fighter_climb_transition", {
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "fighter_pitch_transition",
                        initialPitchDegrees,
                        transitionPitchDegrees,
                        chrono::seconds(6),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [this](const double& value, double) {
                            m_aircraft->setAttitude(m_aircraft->attitude().withPitch(value));
                        }
                    )),
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "fighter_vs_transition",
                        initialClimbRateFpm,
                        sustainedClimbRateFpm,
                        chrono::seconds(8),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [this](const double& value, double) {
                            m_aircraft->setVerticalSpeedFpm(value);
                        }
                    )),
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "fighter_speed_transition",
                        cleanupSpeedKt,
                        climbSpeedKt,
                        chrono::seconds(12),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [this](const double& value, double) {
                            m_aircraft->setGroundSpeedKt(value);
                        }
                    ))
                }));
                climboutSteps.push_back(M.await(Maneuver::Type::Unspecified, "fighter_transition_altitude", [=]() {
                    return currentAltitudeMslFeet() >= transitionAltitudeFeet;
                }));
                climboutSteps.push_back(M.parallel(Maneuver::Type::DepartureClimbBySid, "fighter_climb_exit", {
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "fighter_pitch_exit",
                        transitionPitchDegrees,
                        exitPitchDegrees,
                        chrono::seconds(7),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [this](const double& value, double) {
                            m_aircraft->setAttitude(m_aircraft->attitude().withPitch(value));
                        }
                    )),
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "fighter_vs_exit",
                        sustainedClimbRateFpm,
                        finalClimbRateFpm,
                        chrono::seconds(8),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [this](const double& value, double) {
                            m_aircraft->setVerticalSpeedFpm(value);
                        }
                    )),
                    shared_ptr<Maneuver>(new AnimationManeuver<double>(
                        "fighter_speed_exit",
                        climbSpeedKt,
                        exitSpeedKt,
                        chrono::seconds(12),
                        [](const double& from, const double& to, double progress, double& value) {
                            value = from + (to - from) * progress;
                        },
                        [this](const double& value, double) {
                            m_aircraft->setGroundSpeedKt(value);
                        }
                    ))
                }));
                climboutSteps.push_back(M.await(Maneuver::Type::Unspecified, "fighter_target_altitude", [=]() {
                    return currentAltitudeMslFeet() >= max(1500.0, static_cast<double>(targetAltitudeFeet) - 250.0);
                }));
                climboutSteps.push_back(M.instantAction([=]() {
                    host()->writeLog(
                        "AIPILO|Flight[%s] unrestricted climbout complete at FL%.0f speed[%.0f]kt vs[%.0f]fpm",
                        flight()->callSign().c_str(),
                        currentAltitudeMslFeet() / 100.0,
                        m_aircraft->groundSpeedKt(),
                        m_aircraft->verticalSpeedFpm());
                    m_aircraft->setLights(Aircraft::LightBits::BeaconNav);
                }));

                return M.sequence(Maneuver::Type::DepartureClimbBySid, "unrestricted_climbout_exec", climboutSteps);
            });
        }

        shared_ptr<Runway> getActiveZoneRunway(shared_ptr<Airport> airport, shared_ptr<TaxiEdge> activeZoneEdge)
        {
            host()->writeLog(
                "AIPILO|getActiveZoneRunway: edge [%d|%s] departure-mask [%d] arrival-mask [%d] ils-mask [%d]",
                activeZoneEdge->id(),
                activeZoneEdge->name().c_str(),
                activeZoneEdge->activeZones().departue.runwaysMask(),
                activeZoneEdge->activeZones().arrival.runwaysMask(),
                activeZoneEdge->activeZones().ils.runwaysMask());

            for (const auto& runway : airport->runways())
            {
                host()->writeLog(
                    "AIPILO|getActiveZoneRunway: checking runway [%s/%s], maskbit [%d]",
                    runway->end1().name().c_str(), runway->end2().name().c_str(), runway->maskBit());

                if (activeZoneEdge->activeZones().departue.has(runway) ||
                    activeZoneEdge->activeZones().arrival.has(runway) ||
                    activeZoneEdge->activeZones().ils.has(runway))
                {
                    host()->writeLog("AIPILO|getActiveZoneRunway: FOUND");
                    return runway;//runwayName = runway->end1().name();
                }
            }

            host()->writeLog("AIPILO|getActiveZoneRunway: RUNWAY NOT FOUND");
            return nullptr;
        }

        float waypointReachThresholdMeters(float distanceThresholdMeters) const
        {
            const double groundSpeedKt = max(0.0, aircraft()->groundSpeedKt());
            const float groundSpeedMetersPerSecond = static_cast<float>(groundSpeedKt * METERS_IN_1_NAUTICAL_MILE / 3600.0);
            const float lookAheadMeters = groundSpeedMetersPerSecond * 6.0f;
            return max(distanceThresholdMeters, min(900.0f, lookAheadMeters));
        }

        bool hasReachedWaypoint(const GeoPoint& point, const GeoPoint& legStartPoint, float distanceThresholdMeters = 250.0f)
        {
            const float effectiveThresholdMeters = waypointReachThresholdMeters(distanceThresholdMeters);

            // Check both distance and along-track geometry to determine if waypoint has been reached.
            float distanceToPoint = GeoMath::getDistanceMeters(aircraft()->location(), point);
            
            // Close enough to waypoint - count as reached.
            if (distanceToPoint <= effectiveThresholdMeters)
            {
                return true;
            }

            // If we have a valid leg start point, use triangle geometry to detect
            // if we already passed abeam/beyond the waypoint on this segment.
            if (legStartPoint != GeoPoint::empty)
            {
                const float legLengthMeters = GeoMath::getDistanceMeters(legStartPoint, point);
                const float distanceFromLegStartMeters = GeoMath::getDistanceMeters(legStartPoint, aircraft()->location());
                if (legLengthMeters > 1.0f && distanceFromLegStartMeters > 1.0f)
                {
                    const double fromToAircraftSquared = static_cast<double>(distanceFromLegStartMeters) * distanceFromLegStartMeters;
                    const double fromToWaypointSquared = static_cast<double>(legLengthMeters) * legLengthMeters;
                    const double aircraftToWaypointSquared = static_cast<double>(distanceToPoint) * distanceToPoint;

                    // By cosine law, if FA^2 >= FT^2 + TA^2 then angle at target waypoint is >= 90°,
                    // which means aircraft is already past the waypoint relative to the leg direction.
                    if (fromToAircraftSquared >= fromToWaypointSquared + aircraftToWaypointSquared)
                    {
                        host()->writeLog(
                            "AIPILO|Flight[%s] waypoint passed by along-track geometry, distance %.0f m",
                            flight()->callSign().c_str(),
                            distanceToPoint);
                        return true;
                    }
                }
            }
            
            // Fallback: waypoint significantly behind current heading.
            float headingToPoint = GeoMath::getHeadingFromPoints(aircraft()->location(), point);
            float turnToPointDegrees = GeoMath::getTurnDegrees(aircraft()->attitude().heading(), headingToPoint);
            
            if (abs(turnToPointDegrees) >= 120.0f && distanceToPoint > effectiveThresholdMeters * 1.5f)
            {
                host()->writeLog(
                    "AIPILO|Flight[%s] waypoint appears to be missed - behind by %.1f degrees, distance %.0f m",
                    flight()->callSign().c_str(),
                    turnToPointDegrees,
                    distanceToPoint);
                return true;
            }
            
            return false;
        }

        bool hasReachedWaypoint(const GeoPoint& point, float distanceThresholdMeters = 250.0f)
        {
            return hasReachedWaypoint(point, GeoPoint::empty, distanceThresholdMeters);
        }

        double currentAltitudeAglFeet() const
        {
            const auto altitude = m_aircraft->altitude();
            switch (altitude.type())
            {
            case Altitude::Type::Ground:
                return 0.0;
            case Altitude::Type::AGL:
                return altitude.feet();
            case Altitude::Type::MSL:
            default:
                return altitude.feet() - host()->queryTerrainElevationAt(m_aircraft->location());
            }
        }

        shared_ptr<Maneuver> maneuverDepartureRadarCheckIn()
        {
            return M.deferred([this]() {
                if (m_departureRadarCheckInDone)
                {
                    return M.instantAction([]{});
                }

                auto targetControl = m_helper.tryGetDepartureOrArea(flight());
                if (!targetControl ||
                    targetControl->type() == ControllerPosition::Type::Local ||
                    targetControl->type() == ControllerPosition::Type::Ground)
                {
                    return M.instantAction([]{});
                }

                m_departureRadarCheckInDone = true;
                return M.sequence(Maneuver::Type::DepartureClimbInitialHeading, "departure_radar_checkin", {
                    M.await(Maneuver::Type::Unspecified, "await_departure_radar_altitude", [this]() {
                        return !m_aircraft->altitude().isGroundBased() && currentAltitudeAglFeet() >= 800.0;
                    }),
                    M.delay(chrono::seconds(4)),
                    M.tuneComRadio(flight(), targetControl->frequency()),
                    M.transmitIntent(flight(), I.pilotCheckInWithRadar(flight(), targetControl), "departure_radar_checkin_tx")
                });
            });
        }

        shared_ptr<Maneuver> maneuverArrivalRadarCheckIn()
        {
            return M.deferred([this]() {
                if (m_arrivalRadarCheckInDone)
                {
                    return M.instantAction([]{});
                }

                auto targetControl = m_helper.tryGetArrivalApproach(flight(), m_aircraft->location());
                if (!targetControl ||
                    targetControl->type() == ControllerPosition::Type::Local ||
                    targetControl->type() == ControllerPosition::Type::Ground)
                {
                    return M.instantAction([]{});
                }

                m_arrivalRadarCheckInDone = true;
                return M.sequence(Maneuver::Type::ArrivalDescentFromTop, "arrival_radar_checkin", {
                    M.tuneComRadio(flight(), targetControl->frequency()),
                    M.transmitIntent(flight(), I.pilotCheckInWithRadar(flight(), targetControl), "arrival_radar_checkin_tx")
                });
            });
        }

        shared_ptr<ControllerPosition> tryResolveArrivalControlPosition()
        {
            try
            {
                auto airport = host()->getWorld()->getAirport(flight()->plan()->arrivalAirportIcao());
                if (!airport)
                {
                    return nullptr;
                }

                const GeoPoint location = flight()->aircraft()->location();
                const auto altitude = flight()->aircraft()->altitude();
                float altitudeMslFeet = -1.0f;
                if (altitude.type() == Altitude::Type::MSL)
                {
                    altitudeMslFeet = altitude.feet();
                }
                else if (altitude.type() == Altitude::Type::AGL)
                {
                    altitudeMslFeet = altitude.feet() + host()->queryTerrainElevationAt(location);
                }
                else if (altitude.type() == Altitude::Type::Ground)
                {
                    altitudeMslFeet = host()->queryTerrainElevationAt(location);
                }
                try
                {
                    return airport->localAt(location, altitudeMslFeet);
                }
                catch (const exception&)
                {
                }

                auto tower = airport->tower();
                if (!tower)
                {
                    return nullptr;
                }

                const vector<ControllerPosition::Type> fallbackTypes = {
                    ControllerPosition::Type::Local,
                    ControllerPosition::Type::Ground,
                    ControllerPosition::Type::Approach,
                    ControllerPosition::Type::Departure,
                    ControllerPosition::Type::ClearanceDelivery
                };

                for (const auto fallbackType : fallbackTypes)
                {
                    try
                    {
                        auto candidate = tower->tryFindPosition(fallbackType, location, altitudeMslFeet);
                        if (candidate)
                        {
                            return candidate;
                        }
                    }
                    catch (const exception&)
                    {
                    }
                }

                if (!tower->positions().empty())
                {
                    return tower->positions().front();
                }
            }
            catch (const exception&)
            {
            }

            return nullptr;
        }

        void ensureAutoGoAroundRequested(const string& reasonTag)
        {
            if (flight()->tryFindClearance<GoAroundRequest>(Clearance::Type::GoAroundRequest))
            {
                return;
            }

            static atomic<long long> nextSyntheticId(-1);

            Clearance::Header header;
            header.id = nextSyntheticId.fetch_sub(1);
            header.type = Clearance::Type::GoAroundRequest;
            header.issuedBy = tryResolveArrivalControlPosition();
            header.issuedTo = flight();
            header.issuedTimestamp = host()->getWorld()->timestamp();

            const auto& landingRunway = m_helper.getLandingRunwayEnd(flight());
            auto request = shared_ptr<GoAroundRequest>(new GoAroundRequest(
                header,
                landingRunway.name(),
                DeclineReason::RunwayNotVacated));

            flight()->addClearance(request);

            host()->writeLog(
                "AIPILO|AUTO_GO_AROUND flight[%s] runway[%s] reason[%s]",
                flight()->callSign().c_str(),
                landingRunway.name().c_str(),
                reasonTag.c_str());
        }

        bool shouldAutoGoAroundForLateLandingClearance()
        {
            if (m_finalReportedTimestamp.count() <= 0)
            {
                return false;
            }

            if (flight()->tryFindClearance<LandingClearance>(Clearance::Type::LandingClearance) ||
                flight()->tryFindClearance<GoAroundRequest>(Clearance::Type::GoAroundRequest))
            {
                return false;
            }

            const auto now = host()->getWorld()->timestamp();
            if (now <= m_finalReportedTimestamp)
            {
                return false;
            }

            const auto elapsed = now - m_finalReportedTimestamp;
            if (elapsed < chrono::seconds(75))
            {
                return false;
            }

            const auto& runwayEnd = m_helper.getLandingRunwayEnd(flight());
            const GeoPoint threshold = runwayEnd.centerlinePoint().geo();
            const float distanceMeters = GeoMath::getDistanceMeters(m_aircraft->location(), threshold);

            // Only trigger once close enough to be committed on final.
            return distanceMeters <= 12000.0f;
        }

        bool shouldAutoGoAroundForRunwayOvershoot()
        {
            if (flight()->tryFindClearance<LandingClearance>(Clearance::Type::LandingClearance) ||
                flight()->tryFindClearance<GoAroundRequest>(Clearance::Type::GoAroundRequest))
            {
                return false;
            }

            if (m_aircraft->altitude().type() == Altitude::Type::Ground)
            {
                return false;
            }

            const auto& runwayEnd = m_helper.getLandingRunwayEnd(flight());
            const GeoPoint threshold = runwayEnd.centerlinePoint().geo();
            const float distanceMeters = GeoMath::getDistanceMeters(m_aircraft->location(), threshold);
            // Scale detection range by approach speed so faster aircraft trigger earlier.
            const float speedKt = static_cast<float>(max(80.0, m_aircraft->groundSpeedKt()));
            const float detectionRangeMeters = max(4000.0f, min(12000.0f, speedKt * 40.0f));
            if (distanceMeters > detectionRangeMeters)
            {
                return false;
            }

            const float headingToThreshold = GeoMath::getHeadingFromPoints(m_aircraft->location(), threshold);
            const float turnToThreshold = fabs(GeoMath::getTurnDegrees(m_aircraft->attitude().heading(), headingToThreshold));

            // Runway is significantly behind the aircraft while still near the threshold area.
            // Tighter angle at high speed avoids late detection.
            const float overshootAngle = speedKt > 200.0f ? 110.0f : 130.0f;
            return turnToThreshold > overshootAngle;
        }
    };
}
