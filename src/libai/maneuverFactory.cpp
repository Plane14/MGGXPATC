// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include "libworld.h"
#include "basicManeuverTypes.hpp"
#include "maneuverFactory.hpp"
#include "clearanceFactory.hpp"
#include "intentFactory.hpp"
#include "aiAircraft.hpp"

using namespace std;
using namespace world;

namespace ai
{
    static shared_ptr<AIAircraft> getAIAircraft(shared_ptr<Flight> flight)
    {
        auto aircraft = dynamic_pointer_cast<AIAircraft>(flight->aircraft());
        if (aircraft)
        {
            return aircraft;
        }
        throw runtime_error("Flight [" + flight->callSign() + "]: not an AI aircraft");
    }
    
    // shared_ptr<Maneuver> ManeuverFactory::departureLineUpAndWait(shared_ptr<Flight> flight)
    // {
    //     auto airport = m_host->getWorld()->getAirport(flight->plan()->departureAirportIcao());
    //     auto gate = airport->getParkingStandOrThrow(flight->plan()->departureGate());
    //     auto runway = airport->getRunwayOrThrow(flight->plan()->departureRunway());
    //     auto runwayEnd = runway->getEndOrThrow(flight->plan()->departureRunway());

    // }

    shared_ptr<Maneuver> ManeuverFactory::taxiByPath(
        shared_ptr<Flight> flight,
        shared_ptr<TaxiPath> path,
        TaxiType typeOfTaxi,
        HoldingShortCallback onHoldingShort)
    {
        if (!path)
        {
            m_host->writeLog(
                "MANEUV|WARNING: taxiByPath called with null path for flight[%s], using noop",
                flight->callSign().c_str());
            return instantAction([]{});
        }

        const auto getTurnRadius = [typeOfTaxi](float fromHeading, float toHeading) {
            float angle = abs(GeoMath::getTurnDegrees(fromHeading, toHeading));
            if (angle < 1)
            {
                return -1.0;
            }
            if (typeOfTaxi == TaxiType::Pushback)
            {
                return 0.00015;
            }
            if (angle < 15)
            {
                return 0.0004;
            }
            if (angle < 30)
            {
                return 0.0003;
            }
            return 0.0002;
            //return (angle > 1 ? 0.0002 /*  * (1 + angle/180)  */ : -1);
        };

        const auto calcRoundTurn = [=](
            shared_ptr<TaxiEdge> from, 
            shared_ptr<TaxiEdge> to,
            GeoMath::TurnData& turnData,
            GeoMath::TurnArc& turnArc
        ) {
            float turnRadius;
            if (!to || (turnRadius = getTurnRadius(from->heading(), to->heading())) <= 0)
            {
                return false;
            }
            turnData = {  
                from->node1()->location().geo(),
                from->node2()->location().geo(),
                GeoMath::headingToAngleRadians(from->heading()),
                to->node1()->location().geo(),
                to->node2()->location().geo(),
                GeoMath::headingToAngleRadians(to->heading()),
                turnRadius
            };
            m_host->writeLog("before calculateTurn");
            GeoMath::calculateTurn(turnData, turnArc, m_host);
            m_host->writeLog("after calculateTurn");
            return true;
        };

        const auto patchArcEndPoint = [](GeoMath::TurnArc& arc){
            GeoPoint patched(
                arc.arcCenter.latitude + arc.arcRadius * sin(arc.arcEndAngle),
                arc.arcCenter.longitude + arc.arcRadius * cos(arc.arcEndAngle),
                7777);
            return patched;
        };

        const auto addTaxiStep = [=](
            vector<shared_ptr<Maneuver>>& steps,
            GeoMath::TurnData& turnData,
            GeoMath::TurnArc& turnArc,
            shared_ptr<TaxiEdge> prevEdge,
            shared_ptr<TaxiEdge> edge,
            shared_ptr<TaxiEdge> nextEdge,
            bool exitRoundTurn,
            bool& enterRoundTurn
        ){
            stringstream logstr;
            logstr << setprecision(11) << endl;

            logstr << "---addTaxiStep(" << edge->node1()->id() << "->" << edge->node2()->id() << ")" << endl;
            logstr << edge->node1()->location().geo().latitude << ","
                 << edge->node1()->location().geo().longitude << "->"
                 << edge->node2()->location().geo().latitude << ","
                 << edge->node2()->location().geo().longitude
                 << (edge->activeZones().hasAny() ? " [AZ!]" : "") << endl;
            m_host->writeLog(logstr.str().c_str());

            const bool entersActiveZone = edge->activeZones().hasAny() && (!prevEdge || !prevEdge->activeZones().hasAny());
            // Fallback for synthetic lineup edges appended at the end of some taxi paths:
            // trigger hold-short callback at the last real edge before synthetic extension.
            const bool entersSyntheticLineupExtension =
                !edge->activeZones().hasAny() &&
                edge->id() >= 0 &&
                nextEdge &&
                nextEdge->id() < 0;

            if (entersActiveZone || entersSyntheticLineupExtension)
            {
                auto holdShortManeuver = onHoldingShort(edge);
                if (holdShortManeuver)
                {
                    steps.push_back(holdShortManeuver);
                }
            }

            const GeoPoint straightFromPoint = exitRoundTurn
                ? patchArcEndPoint(turnArc)
                : edge->node1()->location().geo();

            enterRoundTurn = calcRoundTurn(edge, nextEdge, turnData, turnArc);
            const GeoPoint straightToPoint = enterRoundTurn
                ? turnArc.p0
                : edge->node2()->location().geo();
            
            steps.push_back(taxiStraight(
                flight, 
                straightFromPoint,
                straightToPoint,
                typeOfTaxi
            ));

            if (enterRoundTurn)
            {
                float speedFactor;
                if (typeOfTaxi == TaxiType::Pushback) {
                    speedFactor = 2.5f;
                } else if (typeOfTaxi == TaxiType::HighSpeed) {
                    speedFactor = 16.0f;
                } else {
                    speedFactor = 5.0f;
                }
                auto turnDuration = chrono::milliseconds((int)(1000 * turnArc.arcLengthMeters / speedFactor));
                steps.push_back(taxiTurn(flight, turnArc, turnDuration, typeOfTaxi));
            }
        };

        const auto& edges = path->edges;
        vector<shared_ptr<Maneuver>> steps;

        if (edges.empty())
        {
            m_host->writeLog(
                "MANEUV|WARNING: taxiByPath called with empty path for flight[%s], using noop",
                flight->callSign().c_str());
            return instantAction([]{});
        }

        if (typeOfTaxi != TaxiType::Pushback && flight->aircraft()->location() != edges[0]->node1()->location().geo())
        {
            steps.push_back(taxiStraight(
                flight, 
                flight->aircraft()->location(),
                edges[0]->node1()->location().geo(),
                typeOfTaxi
            ));
        }

        GeoMath::TurnData turnData;
        GeoMath::TurnArc turnArc;
        bool exitingRoundTurn = false;
        bool enteredRoundTurn = false;

        for (int i = 0 ; i < edges.size() ; i++ )
        {
            exitingRoundTurn = enteredRoundTurn;
            enteredRoundTurn = false;
            addTaxiStep(
                steps, 
                turnData,
                turnArc,
                i > 0 ? edges[i-1] : nullptr,
                edges[i],
                i < edges.size() - 1 ? edges[i+1] : nullptr,
                exitingRoundTurn,
                enteredRoundTurn);
        }

        return shared_ptr<Maneuver>(new SequentialManeuver(
            Maneuver::Type::TaxiByPath,
            "",
            steps
        ));
    }

    shared_ptr<Maneuver> ManeuverFactory::taxiStraight(
        shared_ptr<Flight> flight,
        const GeoPoint& from,
        const GeoPoint& to,
        TaxiType typeOfTaxi)
    {
        return deferred([=]() {
            stringstream logstr;
            logstr << setprecision(11) << endl;

            logstr << "---taxiStraight---" << endl;
            logstr << "from=" << from.latitude << "," << from.longitude << endl;
            logstr << "to=" << to.latitude << "," << to.longitude << endl;

            float heading = (typeOfTaxi == TaxiType::Pushback
                ? GeoMath::getHeadingFromPoints(to, from)
                : GeoMath::getHeadingFromPoints(from, to));

            float distanceMeters = GeoMath::getDistanceMeters(from, to);

            logstr << "heading=" << heading << endl;
            m_host->writeLog(logstr.str().c_str());

            auto world = m_host->getWorld();
            auto aircraft = getAIAircraft(flight);
            const float startSpeedKnots = static_cast<float>(aircraft->groundSpeedKt());
            // Category-aware taxi speed (m/s): heavies taxi slower, light props faster relative to size.
            float speedFactor;
            if (typeOfTaxi == TaxiType::Pushback) {
                speedFactor = 2.5f;
            } else if (typeOfTaxi == TaxiType::HighSpeed) {
                speedFactor = (aircraft->category() == Aircraft::Category::Heavy ? 20.0f
                    : (aircraft->category() == Aircraft::Category::LightProp ? 14.0f : 18.0f));
            } else {
                speedFactor = (aircraft->category() == Aircraft::Category::Heavy ? 5.0f
                    : (aircraft->category() == Aircraft::Category::LightProp ? 7.0f
                    : (aircraft->category() == Aircraft::Category::Turboprop ? 6.5f : 6.0f)));
            }
            const float targetSpeedKnots = speedFactor * 1.94384f;

            // Shared state for smooth speed transitions
            struct SpeedState {
                float currentSpeed;
                bool isBraking = false;
            };
            auto speedState = make_shared<SpeedState>();
            speedState->currentSpeed = startSpeedKnots;

            // Acceleration/deceleration rates (knots per second) scaled by performance profile.
            const float brakeRate = max(4.0f, aircraft->performanceProfile().landingRolloutDecelerationKtPerSecond * 0.5f);
            const float accelRate = max(2.0f, aircraft->performanceProfile().takeoffAccelerationKtPerSecond * 0.35f);
            const float dt = 0.1f;             // Assume 10Hz update rate

            return shared_ptr<Maneuver>(new AnimationManeuver<GeoPoint>(
                "",
                from,
                to,
                chrono::milliseconds((int)(1000 * distanceMeters / speedFactor)),
                [=](const GeoPoint& from, const GeoPoint& to, double progress, GeoPoint& value) {
                    value.latitude = from.latitude + progress * (to.latitude - from.latitude);
                    value.longitude = from.longitude + progress * (to.longitude - from.longitude);
                    value.altitude = 0;
                },
                [flight, heading, targetSpeedKnots, speedState, brakeRate, accelRate, dt](const GeoPoint& value, double progress) {
                    auto aircraft = getAIAircraft(flight);
                    aircraft->setLocation(value);
                    aircraft->setAttitude(aircraft->attitude().withHeading(heading));

                    // Smooth speed transition
                    float target = speedState->isBraking ? 0.0f : targetSpeedKnots;
                    float& current = speedState->currentSpeed;

                    if (current < target) {
                        current = min(target, current + accelRate * dt);
                    } else if (current > target) {
                        current = max(target, current - brakeRate * dt);
                    }

                    aircraft->setGroundSpeedKt(current);
                },
                [=](Maneuver::SemaphoreState previousState, chrono::microseconds totalWaitDuration) {
                    auto state = obstacleScanSemaphore(
                        world, flight, typeOfTaxi == TaxiType::Pushback, previousState, totalWaitDuration);

                    auto aircraft = getAIAircraft(flight);
                    // Update braking state based on semaphore
                    bool wasBraking = speedState->isBraking;
                    speedState->isBraking = (state == Maneuver::SemaphoreState::Closed);
                    if (speedState->isBraking)
                    {
                        speedState->currentSpeed = 0.0f;
                        aircraft->hardStop(heading);
                    }

                    // Log transition
                    if (wasBraking && !speedState->isBraking) {
                        m_host->writeLog("MANEUV|Flight[%s] obstacle cleared, resuming taxi", flight->callSign().c_str());
                    } else if (!wasBraking && speedState->isBraking) {
                        m_host->writeLog("MANEUV|Flight[%s] obstacle detected, braking", flight->callSign().c_str());
                    }

                    return state;
                },
                [=]() {
                    auto aircraft = getAIAircraft(flight);
                    speedState->isBraking = true;
                    speedState->currentSpeed = 0.0f;
                    aircraft->hardStop(heading);
                }
            ));
        });
    }

    shared_ptr<Maneuver> ManeuverFactory::taxiTurn(
        shared_ptr<Flight> flight,
        const GeoMath::TurnArc& arc,
        chrono::microseconds duration,
        TaxiType typeOfTaxi)
    {
        return deferred([=]() {
            stringstream logstr;
            logstr << setprecision(11) << endl;
            logstr << "---taxiTurn---" << endl;
            logstr << "from=" << arc.p0.latitude << "," << arc.p0.longitude << endl;
            logstr << "to=" << arc.p1.latitude << "," << arc.p1.longitude << endl;
            logstr << "arcCenter=" << arc.arcCenter.latitude << "," << arc.arcCenter.longitude << endl;
            logstr << "heading0=" << arc.heading0 << endl;
            logstr << "heading1=" << arc.heading1 << endl;
            logstr << "deltaAngle=" << arc.arcDeltaAngle << endl;

            double deltaAngle = arc.arcDeltaAngle;

            float deltaHeading = arc.heading1 - arc.heading0;
            // Normalize to shortest-path signed angle in (-180, +180]
            if (deltaHeading > 180.0f)
            {
                deltaHeading -= 360.0f;
            }
            if (deltaHeading < -180.0f)
            {
                deltaHeading += 360.0f;
            }
            logstr << "deltaHeading=" << deltaHeading << endl;
            m_host->writeLog(logstr.str().c_str());

            auto world = m_host->getWorld();
            auto aircraft = getAIAircraft(flight);
            const float startSpeedKnots = static_cast<float>(aircraft->groundSpeedKt());

            // Calculate target speed for this turn
            const double durationSeconds = chrono::duration_cast<chrono::microseconds>(duration).count() / 1000000.0;
            const double targetSpeedMetersPerSecond = (durationSeconds > 0) ? (arc.arcLengthMeters / durationSeconds) : 0;
            const float targetSpeedKnots = static_cast<float>(targetSpeedMetersPerSecond * 1.94384);

            // Shared state for smooth speed transitions
            struct SpeedState {
                float currentSpeed = 0.0f;
                bool isBraking = false;
            };
            auto speedState = make_shared<SpeedState>();
            speedState->currentSpeed = startSpeedKnots;

            // Acceleration/deceleration rates scaled by performance — reduced for turns.
            const float brakeRate = max(3.0f, aircraft->performanceProfile().landingRolloutDecelerationKtPerSecond * 0.4f);
            const float accelRate = max(1.5f, aircraft->performanceProfile().takeoffAccelerationKtPerSecond * 0.25f);
            const float dt = 0.1f;             // Assume 10Hz update rate

            return shared_ptr<Maneuver>(new AnimationManeuver<double>(
                "",
                arc.arcStartAngle,
                arc.arcEndAngle,
                duration,
                [deltaAngle](const double& from, const double& to, double progress, double& value) {
                    value = from + progress * deltaAngle;
                },
                [flight, arc, deltaHeading, typeOfTaxi, targetSpeedKnots, speedState, brakeRate, accelRate, dt](const double& value, double progress) {
                    auto aircraft = getAIAircraft(flight);
                    GeoPoint newLocation(
                        arc.arcCenter.latitude + arc.arcRadius * sin(value),
                        arc.arcCenter.longitude + arc.arcRadius * cos(value));
                    double newHeading = GeoMath::addTurnToHeading(
                        static_cast<float>(arc.heading0),
                        static_cast<float>(progress * deltaHeading));
                    if (typeOfTaxi == TaxiType::Pushback)
                    {
                        newHeading = GeoMath::flipHeading(newHeading);
                    }
                    aircraft->setLocation(newLocation);
                    aircraft->setAttitude(aircraft->attitude().withHeading(newHeading));

                    // Smooth speed transition
                    float target = speedState->isBraking ? 0.0f : targetSpeedKnots;
                    float& current = speedState->currentSpeed;

                    if (current < target) {
                        current = min(target, current + accelRate * dt);
                    } else if (current > target) {
                        current = max(target, current - brakeRate * dt);
                    }

                    aircraft->setGroundSpeedKt(current);
                },
                [=](Maneuver::SemaphoreState previousState, chrono::microseconds totalWaitDuration) {
                    auto state = obstacleScanSemaphore(
                        world, flight, typeOfTaxi == TaxiType::Pushback, previousState, totalWaitDuration);

                    auto aircraft = getAIAircraft(flight);
                    // Update braking state based on semaphore
                    speedState->isBraking = (state == Maneuver::SemaphoreState::Closed);
                    if (speedState->isBraking)
                    {
                        speedState->currentSpeed = 0.0f;
                        aircraft->hardStop(static_cast<float>(aircraft->attitude().heading()));
                    }

                    return state;
                },
                [=]() {
                    auto aircraft = getAIAircraft(flight);
                    speedState->isBraking = true;
                    speedState->currentSpeed = 0.0f;
                    aircraft->hardStop(static_cast<float>(aircraft->attitude().heading()));
                }
            ));
        });
    }

    shared_ptr<Maneuver> ManeuverFactory::taxiStop(shared_ptr<Flight> flight)
    {
        return deferred([=]() {
            auto aircraft = getAIAircraft(flight);
            const float startSpeedKnots = static_cast<float>(aircraft->groundSpeedKt());
            if (startSpeedKnots <= 0.01f)
            {
                return instantAction([]{});
            }

            const float brakeRate = max(4.0f, aircraft->performanceProfile().landingRolloutDecelerationKtPerSecond * 0.6f);
            const auto duration = chrono::milliseconds(max(500, static_cast<int>(1000.0f * startSpeedKnots / brakeRate)));

            return shared_ptr<Maneuver>(new AnimationManeuver<double>(
                "taxi_stop",
                startSpeedKnots,
                0.0,
                duration,
                [](const double& from, const double& to, double progress, double& value) {
                    value = from + progress * (to - from);
                },
                [=](const double& value, double progress) {
                    auto aircraft = getAIAircraft(flight);
                    aircraft->setGroundSpeedKt(value);
                }
            ));
        });
    }

    shared_ptr<Maneuver> ManeuverFactory::instantAction(function<void()> action, const string& id)
    {
        return shared_ptr<Maneuver>(new InstantActionManeuver(
            Maneuver::Type::Unspecified,
            id,
            action
        ));
    }

    shared_ptr<Maneuver> ManeuverFactory::delay(chrono::microseconds duration)
    {
        return deferred([=](){
            chrono::microseconds targetTimestamp = 
                m_host->getWorld()->timestamp() + 
                chrono::microseconds(duration.count());
        
            return await(Maneuver::Type::Unspecified, "delay", [=]() {
                return m_host->getWorld()->timestamp() >= targetTimestamp;
            });
        });
    }

    shared_ptr<Maneuver> ManeuverFactory::deferred(
        Maneuver::Type type, 
        const string& id,
        DeferredManeuver::Factory factory)
    {
        return DeferredManeuver::create(type, id, factory);
    }

    shared_ptr<Maneuver> ManeuverFactory::deferred(DeferredManeuver::Factory factory)
    {
        return DeferredManeuver::create(Maneuver::Type::Unspecified, "", factory);
    }

    shared_ptr<Maneuver> ManeuverFactory::await(Maneuver::Type type, const string& id, function<bool()> isReady)
    {
        return shared_ptr<AwaitManeuver>(new AwaitManeuver(m_host, type, id, isReady));
    }

    shared_ptr<Maneuver> ManeuverFactory::awaitClearance(shared_ptr<Flight> flight, Clearance::Type clearanceType, const string& id)
    {
        return await(Maneuver::Type::AwaitClearance, id, [=]() {
            return !!flight->tryFindClearance<Clearance>(clearanceType);
        });
    }

    shared_ptr<Maneuver> ManeuverFactory::sequence(Maneuver::Type type, const string& id, const vector<shared_ptr<Maneuver>>& steps)
    {
        return shared_ptr<SequentialManeuver>(new SequentialManeuver(
            type,
            id,
            steps
        ));
    }

    shared_ptr<Maneuver> ManeuverFactory::sequence(Maneuver::Type type, const string& id, initializer_list<shared_ptr<Maneuver>> steps)
    {
        return sequence(type, id, vector<shared_ptr<Maneuver>>(steps));
    }
    
    shared_ptr<Maneuver> ManeuverFactory::parallel(Maneuver::Type type, const string& id, const vector<shared_ptr<Maneuver>>& steps)
    {
        return shared_ptr<ParallelManeuver>(new ParallelManeuver(
            type,
            id,
            steps
        ));
    }

    shared_ptr<Maneuver> ManeuverFactory::parallel(Maneuver::Type type, const string& id, initializer_list<shared_ptr<Maneuver>> steps)
    {
        return parallel(type, id, vector<shared_ptr<Maneuver>>(steps));
    }

    shared_ptr<Maneuver> ManeuverFactory::switchLights(shared_ptr<Flight> flight, Aircraft::LightBits lights)
    {
        return instantAction([=](){
            getAIAircraft(flight)->setLights(lights);
        });
    }

    shared_ptr<Maneuver> ManeuverFactory::tuneComRadio(shared_ptr<Flight> flight, int frequencyKhz)
    {
        return instantAction([=](){ 
            flight->aircraft()->setFrequencyKhz(frequencyKhz);
        });
    }

    shared_ptr<Maneuver> ManeuverFactory::tuneComRadio(shared_ptr<Flight> flight, shared_ptr<Frequency> frequency)
    {
        return instantAction([=](){ 
            flight->aircraft()->setFrequency(frequency);
        });
    }

    struct BoxedTransmission
    {
        shared_ptr<Transmission> ptr;
        bool enqueuedPushToTalk = false;
    };

    static chrono::milliseconds getSilenceDurationBeforePushToTalk(
        const shared_ptr<Flight>& flight,
        const shared_ptr<Intent>& intent,
        int millisecondsOverride)
    {
        if (millisecondsOverride >= 0)
        {
            return chrono::milliseconds(millisecondsOverride);
        }

        if (intent->isReply())
        {
            return chrono::milliseconds(150);
        }

        if (intent->isCritical())
        {
            return chrono::milliseconds(1500);
        }

        return chrono::milliseconds(flight->phase() == Flight::Phase::Arrival ? 3000 : 4000);
    }

    shared_ptr<Maneuver> ManeuverFactory::transmitIntent(
        shared_ptr<Flight> flight,
        shared_ptr<Intent> intent,
        const string& id,
        int millisecondsSilence,
        Frequency::CancellationQueryCallback onQueryCancel)
    {
        auto boxedTransmission = shared_ptr<BoxedTransmission>(new BoxedTransmission());
        chrono::milliseconds silenceDuration = getSilenceDurationBeforePushToTalk(flight, intent, millisecondsSilence);
        string silenceAwaitId =
            flight->callSign() + "/" + to_string(silenceDuration.count()) + "-silence/" +
            to_string(flight->aircraft()->frequencyKhz());
        auto waitStartedAt = m_host->getWorld()->timestamp();

        return sequence(Maneuver::Type::Unspecified, id, {
            await(Maneuver::Type::AwaitSilenceOnFrequency, silenceAwaitId, [=](){
                auto frequency = flight->aircraft()->frequency();
                if (!frequency)
                {
                    return true;
                }
                if (onQueryCancel())
                {
                    m_host->writeLog(
                        "AIPILO|TRANSMISSION CANCELLED [%s]->[%s] intent code[%d]",
                        intent->subjectFlight()->callSign().c_str(),
                        intent->subjectControl()->callSign().c_str(),
                        intent->code());
                    return true;
                }
                if (!boxedTransmission->enqueuedPushToTalk)
                {
                    frequency->enqueuePushToTalk(silenceDuration, intent, [=](shared_ptr<Transmission> transmission) {
                        boxedTransmission->ptr = transmission;
                    }, onQueryCancel);
                    boxedTransmission->enqueuedPushToTalk = true;
                }
                bool result = !!boxedTransmission->ptr;
                return result;

//                auto elapsed = m_host->getWorld()->timestamp() - waitStartedAt;
//                int effectiveWaitMilliseconds = (elapsed.count() > 30000000 ? awaitSilenceMilliseconds / 2 : awaitSilenceMilliseconds);
//                auto frequency = flight->aircraft()->frequency();
//                bool result = !frequency || frequency->wasSilentFor(chrono::milliseconds(effectiveWaitMilliseconds));
//                return result;
            }),
            instantAction([=](){
                if (!flight->aircraft()->frequency())
                {
                    m_host->writeLog(
                        "AIPILO|TRANSMISSION ERROR from [%s] on [%d]: no ATC on this frequency",
                        flight->callSign().c_str(), 
                        flight->aircraft()->frequencyKhz());
                }
            }),
            await(Maneuver::Type::Unspecified, "await-transmit-end", [=](){
                return (!boxedTransmission->ptr ||
                    boxedTransmission->ptr->state() == Transmission::State::Completed ||
                    boxedTransmission->ptr->state() == Transmission::State::Cancelled);
            }),
            instantAction([=]() {
                if (!boxedTransmission->ptr)
                {
                    return;
                }
                bool completed = boxedTransmission->ptr->state() == Transmission::State::Completed;
                m_host->writeLog(
                    "AIPILO|TRANSMISSION %s [%s]->[%s] intent code[%d]",
                    completed ? "COMPLETED" : "CANCELLED",
                    intent->subjectFlight()->callSign().c_str(),
                    intent->subjectControl()->callSign().c_str(),
                    intent->code());
            })
        });
    }

    shared_ptr<Maneuver> ManeuverFactory::airborneTurn(shared_ptr<Flight> flight, float fromHeading, float toHeading)
    {
        if (abs(GeoMath::getTurnDegrees(fromHeading, toHeading)) < 0.1f)
        {
            return instantAction([](){});
        }

        auto aircraft = getAIAircraft(flight);
        float turnDegrees = GeoMath::getTurnDegrees(fromHeading, toHeading);
        // Speed-dependent bank angle: standard-rate turn bank ≈ V_kt / 10 + 7.
        const float speedKt = static_cast<float>(max(80.0, aircraft->groundSpeedKt()));
        const float maxBank = min(30.0f, speedKt / 10.0f + 7.0f);
        const float absBank = (abs(turnDegrees) > 10) ? maxBank : min(15.0f, maxBank * 0.6f);
        int bankAngle = static_cast<int>(absBank) * (turnDegrees < 0 ? -1 : 1);
        // Turn rate (deg/s) from bank: rate = g·tan(bank)/V ≈ 1091·tan(bank)/V_kt.
        const float bankRad = absBank * 3.14159265f / 180.0f;
        const float turnRate = max(1.5f, 1091.0f * tanf(bankRad) / speedKt);
        auto turnDuration = chrono::milliseconds(max(500, static_cast<int>(1000.0f * abs(turnDegrees) / turnRate)));
        // Roll duration proportional to bank magnitude; heavier bank takes longer to establish.
        auto rollDuration = chrono::milliseconds(max(1500, static_cast<int>(absBank * 120.0f)));

        m_host->writeLog(
            "MANEUVER airborneTurn: from %f to %f = %f deg, bank %d, rate %f, Tturn=%lld ms, Tbank=%lld ms",
            fromHeading, toHeading, turnDegrees, bankAngle, turnRate, turnDuration.count(), rollDuration.count());

        auto rollIn = shared_ptr<Maneuver>(new AnimationManeuver<double>(
            "", 
            0,
            bankAngle,
            rollDuration,
            [](const double& from, const double& to, double progress, double& value) {
                value = from + (to - from) * progress; 
            },
            [=](const double& value, double progress) {
                aircraft->setAttitude(aircraft->attitude().withRoll(value));
            }
        ));
        auto rollOut = shared_ptr<Maneuver>(new AnimationManeuver<double>(
            "", 
            bankAngle,
            0,
            rollDuration,
            [](const double& from, const double& to, double progress, double& value) {
                value = from + (to - from) * progress; 
            },
            [=](const double& value, double progress) {
                aircraft->setAttitude(aircraft->attitude().withRoll(value));
            }
        ));
        auto turn = shared_ptr<Maneuver>(new AnimationManeuver<double>(
            "", 
            0,
            turnDegrees,
            turnDuration,
            [](const double& from, const double& to, double progress, double& value) {
                value = from + (to - from) * progress; 
            },
            [=](const double& value, double progress) {
                auto newHeading = GeoMath::addTurnToHeading(fromHeading, value);
                aircraft->setAttitude(aircraft->attitude().withHeading(newHeading));
            }
        ));

        auto holdDuration = (turnDuration > 2 * rollDuration)
            ? turnDuration - 2 * rollDuration
            : chrono::milliseconds(0);

        return parallel(Maneuver::Type::Unspecified, "", {
            sequence(Maneuver::Type::Unspecified, "", {
                rollIn,
                delay(holdDuration),
                rollOut
            }),
            sequence(Maneuver::Type::Unspecified, "", {
                turn, // TODO: vary turn rate by bank angle
                instantAction([=]() {
                    aircraft->setAttitude(aircraft->attitude().withHeading(toHeading));
                })
            }),
        });
    }

    shared_ptr<Maneuver> ManeuverFactory::noopOnHoldingShort(shared_ptr<TaxiEdge> atEdge)
    {
        return nullptr;
    }

    void ManeuverFactory::calculateObstacleScanRect(
        const GeoPoint& location,
        float heading,
        GeoPoint& topLeft,
        GeoPoint& bottomRight,
        float radiusMeters)
    {
        float radius = 0.00001 * radiusMeters;
        // Apply cosine(latitude) correction so longitude offset matches true meters.
        const float latRad = static_cast<float>(location.latitude * M_PI / 180.0);
        const float cosLat = max(0.01f, cosf(latRad));
        float radiusLon = radius / cosLat;
        float halfRadius = radius / 2;
        float halfRadiusLon = radiusLon / 2;
        int sectorIndex = getScanSectorIndex(heading);

        switch (sectorIndex)
        {
        case 0: // -22.5..+22.5
            topLeft = {location.latitude + radius, location.longitude - halfRadiusLon };
            bottomRight = { location.latitude, location.longitude + halfRadiusLon };
            break;
        case 1: // +22.5..+67.5
            topLeft = {location.latitude + radius, location.longitude };
            bottomRight = { location.latitude, location.longitude + radiusLon };
            break;
        case 2:
            topLeft = {location.latitude + halfRadius, location.longitude };
            bottomRight = {location.latitude - halfRadius, location.longitude + radiusLon };
            break;
        case 3:
            topLeft = { location.latitude, location.longitude };
            bottomRight = { location.latitude - radius, location.longitude + radiusLon };
            break;
        case 4:
            topLeft = {location.latitude, location.longitude - halfRadiusLon };
            bottomRight = { location.latitude - radius, location.longitude + halfRadiusLon };
            break;
        case 5:
            topLeft = { location.latitude, location.longitude - radiusLon };
            bottomRight = { location.latitude - radius, location.longitude };
            break;
        case 6:
            topLeft = {location.latitude + halfRadius, location.longitude - radiusLon };
            bottomRight = { location.latitude - halfRadius, location.longitude };
            break;
        case 7:
            topLeft = {location.latitude + radius, location.longitude - radiusLon };
            bottomRight = { location.latitude, location.longitude };
            break;
        }
    }

    Maneuver::SemaphoreState ManeuverFactory::obstacleScanSemaphore(
        shared_ptr<World> world,
        shared_ptr<Flight> ourFlight,
        bool isPushback,
        Maneuver::SemaphoreState previousState,
        chrono::microseconds closedStateTotalDuration)
    {
        auto ourAircraft = ourFlight->aircraft();
        auto ourPhase = ourFlight->phase();

        const float ourSpeedKt = static_cast<float>(abs(ourAircraft->groundSpeedKt()));
        const float baseScanRadius = previousState == Maneuver::SemaphoreState::Open ? 50.0f : 65.0f;
        const float speedLookahead = isPushback ? 6.0f : min(22.0f, ourSpeedKt * 0.9f);
        float scanRadiusMeters = max(40.0f, min(90.0f, baseScanRadius + speedLookahead));
        float ourHeading = isPushback
            ? GeoMath::flipHeading(ourAircraft->attitude().heading())
            : ourAircraft->attitude().heading();

        GeoPoint scanTopLeft;
        GeoPoint scanBottomRight;
        calculateObstacleScanRect(ourAircraft->location(), ourHeading, scanTopLeft, scanBottomRight, scanRadiusMeters);

        const auto isAircraftAnObstacle = [&](shared_ptr<Aircraft> otherAircraft) {
            if (otherAircraft == ourAircraft)
            {
                return false;
            }

            const float distanceMeters = static_cast<float>(GeoMath::getDistanceMeters(
                ourAircraft->location(),
                otherAircraft->location()));
            if (distanceMeters > scanRadiusMeters)
            {
                return false;
            }

            if (otherAircraft->nature() == Actor::Nature::Human && distanceMeters < max(20.0f, scanRadiusMeters * 0.65f))
            {
                return true;
            }

            auto otherPhase = otherAircraft->getFlightOrThrow()->phase();
            if (ourPhase != otherPhase)
            {
                return (ourPhase != Flight::Phase::TurnAround && otherPhase != Flight::Phase::TurnAround);
            }

            float headingToOther = GeoMath::getHeadingFromPoints(ourAircraft->location(), otherAircraft->location());
            float turnToOther = GeoMath::getTurnDegrees(ourHeading, headingToOther);
            float deltaHeading = GeoMath::getTurnDegrees(ourHeading, otherAircraft->attitude().heading());
            bool isSameDirection = abs(deltaHeading) < 60;
            bool isInFront = abs(turnToOther) < 65;
            bool isHeadOn = abs(deltaHeading) > 165;

            if (distanceMeters < 14.0f)
            {
                return true;
            }
            if (isHeadOn && abs(turnToOther) < 85)
            {
                return true;
            }

            const float followConflictDistance = max(26.0f, min(65.0f, ourSpeedKt * 2.4f));
            if (isInFront && isSameDirection && distanceMeters < followConflictDistance)
            {
                return true;
            }

            const bool crossingGeometry = abs(turnToOther) > 15 && abs(turnToOther) < 105 && abs(deltaHeading) > 40 && abs(deltaHeading) < 150;
            if (crossingGeometry)
            {
                if (turnToOther > 0 && distanceMeters < 30.0f)
                {
                    return true; // right-hand priority in crossing conflicts
                }

                if (distanceMeters < 18.0f)
                {
                    return true;
                }
            }

            return false;
        };

        bool obstaclesDetected = world->detectAircraftInRect(scanTopLeft, scanBottomRight, isAircraftAnObstacle);
        if (obstaclesDetected && isPushback && previousState == Maneuver::SemaphoreState::Closed && closedStateTotalDuration >= chrono::seconds(45))
        {
            return Maneuver::SemaphoreState::Open;
        }

        return obstaclesDetected
            ? Maneuver::SemaphoreState::Closed
            : Maneuver::SemaphoreState::Open;
    }

    int ManeuverFactory::getScanSectorIndex(float heading)
    {
        int sector = (int)((heading + 22.5) / 45.0) % 8;
        return sector;
    }

    shared_ptr<Maneuver> ManeuverFactory::repeatingAction(
        function<void()> action,
        chrono::microseconds interval,
        const string& id)
    {
        class RepeatingActionManeuver : public Maneuver
        {
        private:
            function<void()> m_action;
            chrono::microseconds m_interval;
            chrono::microseconds m_elapsedSinceLastAction;
            chrono::microseconds m_lastTimestamp;
        public:
            RepeatingActionManeuver(
                const string& _id,
                function<void()> action,
                chrono::microseconds interval)
                : Maneuver(Type::Unspecified, _id, {}),
                  m_action(action),
                  m_interval(interval),
                  m_elapsedSinceLastAction(chrono::microseconds(0)),
                  m_lastTimestamp(chrono::microseconds(0))
            {
            }

            void progressTo(chrono::microseconds timestamp) override
            {
                if (m_state == State::NotStarted)
                {
                    m_startTimestamp = timestamp;
                    m_state = State::InProgress;
                    m_lastTimestamp = timestamp;
                    // Execute action immediately on start
                    if (m_action)
                    {
                        m_action();
                    }
                    return;
                }

                chrono::microseconds deltaTime = timestamp - m_lastTimestamp;
                m_lastTimestamp = timestamp;
                m_elapsedSinceLastAction += deltaTime;

                // Execute action at each interval
                while (m_elapsedSinceLastAction >= m_interval)
                {
                    m_elapsedSinceLastAction -= m_interval;
                    if (m_action)
                    {
                        m_action();
                    }
                }

                // This maneuver never completes on its own - it runs until cancelled by parent
                m_finishTimestamp = timestamp;
            }

            string getStatusString() const override
            {
                return "RepeatingAction[" + id() + "]";
            }
        };

        return make_shared<RepeatingActionManeuver>(id, action, interval);
    }
}
