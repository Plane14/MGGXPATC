// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once

#include <unordered_map>
#include <unordered_set>

#include "libworld.h"
#include "clearanceFactory.hpp"
#include "intentTypes.hpp"
#include "intentFactory.hpp"
#include "clearanceFactory.hpp"
#include "worldHelper.hpp"
#include "libai.hpp"
#include "stlhelpers.h"

using namespace std;
using namespace world;

#define AI_CONTROLLER_MAP_INTENT(intent_type, handler_func) \
    onIntent<intent_type>([this](shared_ptr<intent_type> intent) { handler_func(intent); })

namespace ai
{
    class AIControllerBase : public Controller
    {
    public:
        typedef function<void(shared_ptr<Intent> intent)> IntentHandlerCallback;
    private:
        WorldHelper m_helper;
        shared_ptr<IntentFactory> m_intentFactory;
        shared_ptr<ClearanceFactory> m_clearanceFactory;
        unordered_map<int, IntentHandlerCallback> m_intentHandlerByCode;
        unordered_set<shared_ptr<Flight>> m_clearedForDepartureTaxi;
        unordered_set<shared_ptr<Flight>> m_departureTaxiHandedOffToTower;
        unordered_set<shared_ptr<Flight>> m_departureTaxiPendingHandoffToTower;
        unordered_set<shared_ptr<Flight>> m_radarTrackedFlights;
        unordered_map<int, shared_ptr<ControllerPosition>> m_pendingRadarHandoffTargetByFlightId;
        int m_nextSquawk = 3101;
    protected:
        IntentFactory& I;
        ClearanceFactory& C;
    public:
        AIControllerBase(shared_ptr<HostServices> _host, int _id, Actor::Gender _gender, shared_ptr<ControllerPosition> _position) :
            Controller(_host, _id, _gender, _position),
            m_helper(_host),
            m_intentFactory(_host->services().get<IntentFactory>()),
            m_clearanceFactory(_host->services().get<ClearanceFactory>()),
            I(*m_intentFactory),
            C(*m_clearanceFactory)
        {
            AI_CONTROLLER_MAP_INTENT(PilotCheckInWithRadarIntent, handlePilotCheckInWithRadar);
            AI_CONTROLLER_MAP_INTENT(PilotHandoffReadbackIntent, handlePilotHandoffReadback);
        }
    public:
        void receiveIntent(shared_ptr<Intent> intent) override
        {
            host()->writeLog(
                "AICONT|callsign[%s] receiveIntent flight[%s] intent code[%d]",
                position()->callSign().c_str(),
                intent->subjectFlight()->callSign().c_str(),
                intent->code());

            try
            {
                IntentHandlerCallback handler;
                if (tryGetValue(m_intentHandlerByCode, intent->code(), handler))
                {
                    handler(intent);
                }
                else if (!fallbackReceiveIntent(intent)) //TODO: get rid of fallbackReceiveIntent
                {
                    host()->writeLog(
                "AICONT|WARNING: callsign[%s] received intent code[%d] from flight[%s], but no handler is registered for this intent code.",
                        position()->callSign().c_str(),
                        intent->code(),
                        intent->subjectFlight()->callSign().c_str());
                }
            }
            catch (const exception &e)
            {
                host()->writeLog(
                    "AICONT|callsign[%s] CRASHED!!! while handling intent code[%d]: %s",
                    position()->callSign().c_str(),
                    intent->code(),
                    e.what());
            }
        }

        void progressTo(chrono::microseconds timestamp) override
        {
            handoffDeparturesToTower();
            handoffTrackedFlightsToNextController();
        }

        void selectActiveRunways(vector<string>& departure, vector<string>& arrival) override
        {
            throw runtime_error(
                "AI controller callsign[" + position()->callSign() + "] cannot selectActiveRunways(): I'm not a local controller");
        }

        void clearFlights() override
        {
        }

    protected:

        //TODO: extract concrete controller classes
        bool fallbackReceiveIntent(shared_ptr<Intent> intent)
        {
            shared_ptr<Intent> reply;

            switch (intent->code())
            {
            case PilotAffirmationIntent::IntentCode:
            case PilotHandoffReadbackIntent::IntentCode:
            case PilotDepartureTaxiReadbackIntent::IntentCode:
            case PilotRunwayCrossReadbackIntent::IntentCode:
            case PilotDepartureHoldShortReadbackIntent::IntentCode:
            case PilotTakeoffClearanceReadbackIntent::IntentCode:
            case PilotContinueApproachReadbackIntent::IntentCode:
            case PilotLandingClearanceReadbackIntent::IntentCode:
            case PilotArrivalTaxiReadbackIntent::IntentCode:
            case PilotGoAroundReadbackIntent::IntentCode:
                break;
            case PilotIfrClearanceRequestIntent::IntentCode:
                {
                    // Generate a valid discrete IFR squawk code.
                    // Squawk codes are 4-digit octal (each digit 0-7); reserved codes
                    // 1200 (VFR), 7500 (hijack), 7600 (comms failure), 7700 (emergency)
                    // must not be issued as discrete codes.
                    // Wrap at 4096 (octal 7777) to prevent integer overflow.
                    m_nextSquawk = (m_nextSquawk >= 4094) ? 3101 : m_nextSquawk + 1;
                    const int raw = m_nextSquawk;
                    int squawk = ((raw / 512) % 8) * 1000 +
                                 ((raw / 64)  % 8) * 100  +
                                 ((raw / 8)   % 8) * 10   +
                                 (raw % 8);
                    if (squawk <= 0 || squawk < 100) squawk = 100;
                    if (squawk == 1200 || squawk == 7500 || squawk == 7600 || squawk == 7700) squawk++;
                    reply = m_intentFactory->deliveryIfrClearanceReply(
                        intent->subjectFlight(),
                         m_clearanceFactory->ifrClearance(intent->subjectFlight(), squawk),
                         intent->id()
                    );
                }
                break;
            case PilotIfrClearanceReadbackIntent::IntentCode:
                {
                    auto clearance = intent->subjectFlight()->findClearanceOrThrow<IfrClearance>(Clearance::Type::IfrClearance);
                    reply = m_intentFactory->deliveryIfrClearanceReadbackCorrect(
                        intent->subjectFlight(),
                        intent->id()
                    );
                }
                break;
            case PilotPushAndStartRequestIntent::IntentCode:
                reply = m_intentFactory->groundPushAndStartReply(
                    intent->subjectFlight(), 
                    m_clearanceFactory->pushAndStartApproval(intent->subjectFlight()),
                    intent->id()
                );
                break;
            case PilotDepartureTaxiRequestIntent::IntentCode:
                reply = m_intentFactory->groundDepartureTaxiReply(
                    intent->subjectFlight(), 
                    m_clearanceFactory->departureTaxiClearance(intent->subjectFlight()),
                    intent->id()
                );
                m_clearedForDepartureTaxi.insert(intent->subjectFlight());
                break;
            case PilotReportHoldingShortIntent::IntentCode:
                {
                    auto typedIntent = dynamic_pointer_cast<PilotReportHoldingShortIntent>(intent);
                    if (typedIntent)
                    {
                        auto flightPlan = intent->subjectFlight()->plan();
                        bool isDepartureRunway = isFlightDepartureRunway(intent->subjectFlight(), typedIntent->runway());
                        if (isDepartureRunway || hasKey(m_departureTaxiHandedOffToTower, intent->subjectFlight()))
                        {
                            reply = I.groundSwitchToTower(intent->subjectFlight(), intent->id());
                            moveDepartureToTower(intent->subjectFlight());
                        }
                        else if (airport()->isRunwayActive(typedIntent->runway()))
                        {
                            host()->writeLog(
                                "AICONT|GND->TWR flight[%s] requested to cross active runway[%s]",
                                intent->subjectFlight()->callSign().c_str(),
                                typedIntent->runway().c_str());

                            auto localController = findLocalControllerOrThrow(intent->subjectFlight());
                            localController->receiveIntent(I.groundCrossRunwayRequestToTower(
                                typedIntent->runway(),
                                intent->subjectFlight(),
                                position(),
                                localController->position(),
                                intent->id()
                            ));
                        }
                        else
                        {
                            reply = I.groundCrossRunwayClearance(m_clearanceFactory->runwayCrossCleaeance(
                                typedIntent->subjectFlight(),
                                typedIntent->runway()
                            ), intent->id());
                        }
                    }
                }
                break;
            case TowerCrossRunwayReplyToGroundIntent::IntentCode:
                {
                    reply = intent;
                    auto typedIntent = dynamic_pointer_cast<TowerCrossRunwayReplyToGroundIntent>(intent);
                    if (typedIntent)
                    {
                        host()->writeLog(
                            "AICONT|GND got reply from TWR for flight[%s] on crossing runway[%s]: %s ; reason[%d]",
                            intent->subjectFlight()->callSign().c_str(),
                            typedIntent->runwayName().c_str(),
                            typedIntent->cleared() ? "APPROVED" : "DECLINED",
                            typedIntent->declineReason());

                        if (typedIntent->cleared())
                        {
                            reply = I.groundCrossRunwayClearance(
                                typedIntent->clearance(),
                                typedIntent->pilotRequestId());
                        }
                        else
                        {
                            reply = I.groundHoldShortRunway(
                                typedIntent->runwayName(),
                                typedIntent->subjectFlight(),
                                position(),
                                typedIntent->declineReason(),
                                typedIntent->pilotRequestId());
                        }
                    }
                }
                break;
            case PilotArrivalCheckInWithGroundIntent::IntentCode:
                {
                    auto typedIntent = dynamic_pointer_cast<PilotArrivalCheckInWithGroundIntent>(intent);
                    auto taxiStartPoint = typedIntent->exitEdge()
                        ? typedIntent->exitEdge()->node2()->location().geo()
                        : intent->subjectFlight()->aircraft()->location();
                    auto clearance = m_clearanceFactory->arrivalTaxiClearance(intent->subjectFlight(), taxiStartPoint);
                    if (!clearance)
                    {
                        host()->writeLog(
                            "AICONT|GND WARNING: unable to create arrival taxi clearance for flight[%s]",
                            intent->subjectFlight()->callSign().c_str());
                        return true;
                    }
                    reply = m_intentFactory->groundArrivalTaxiReply(clearance, intent->id());
                }
                break;
            default:
                return false;
            }

            if (reply)
            {
                position()->frequency()->enqueueTransmission(reply);
            }

            host()->writeLog("AICONT|fallbackReceiveIntent - done");
            return true;
        }

        void transmitCritical(shared_ptr<Intent> intent)
        {
            position()->frequency()->enqueueTransmission(intent);
        }

        void transmit(
            shared_ptr<Intent> intent,
            Frequency::TransmissionCallback  onTransmit = Frequency::noopTRansmissionCallback,
            Frequency::CancellationQueryCallback onQueryCancel = Frequency::noopQueryCancelCallback)
        {
            position()->frequency()->enqueuePushToTalk(
                chrono::milliseconds(intent->replyToId() > 0 ? 0 : 300),
                intent,
                onTransmit,
                onQueryCancel);

//            if (intent->replyToId() > 0)
//            {
//                position()->frequency()->enqueueTransmission(intent);
//            }
//            else
//            {
//                position()->frequency()->enqueuePushToTalk(chrono::milliseconds(300), intent);
//            }
        }

        template <class TConcreteIntent>
        void onIntent(function<void(shared_ptr<TConcreteIntent> intent)> handler)
        {
            const auto handlerAdaptor = [this, handler](shared_ptr<Intent> intent){
                shared_ptr<TConcreteIntent> concreteIntent = dynamic_pointer_cast<TConcreteIntent>(intent);
                if (!concreteIntent)
                {
                    throw runtime_error(
                        "Intent code[" + to_string(intent->code()) + "] cannot be cast to class[" + typeid(TConcreteIntent).name() + "]");
                }
                handler(concreteIntent);
            };

            int key = TConcreteIntent::IntentCode;
            m_intentHandlerByCode.insert({ key, handlerAdaptor });
        }


    private:

        void handlePilotCheckInWithRadar(shared_ptr<PilotCheckInWithRadarIntent> intent)
        {
            if (!intent || !intent->subjectFlight())
            {
                return;
            }

            m_radarTrackedFlights.insert(intent->subjectFlight());
            m_pendingRadarHandoffTargetByFlightId.erase(intent->subjectFlight()->id());

            auto reply = I.controllerRadarContact(intent->subjectFlight(), position(), intent->id());
            if (reply)
            {
                transmit(reply);
            }
        }

        void handlePilotHandoffReadback(shared_ptr<PilotHandoffReadbackIntent> intent)
        {
            if (!intent || !intent->subjectFlight())
            {
                return;
            }

            const auto pending = m_pendingRadarHandoffTargetByFlightId.find(intent->subjectFlight()->id());
            if (pending == m_pendingRadarHandoffTargetByFlightId.end() || !pending->second)
            {
                return;
            }

            if (pending->second->frequency()->khz() == intent->newFrequencyKhz())
            {
                m_radarTrackedFlights.erase(intent->subjectFlight());
                m_pendingRadarHandoffTargetByFlightId.erase(pending);
            }
        }

        double distanceToAirportNm(shared_ptr<Airport> airport, shared_ptr<Flight> flight)
        {
            if (!airport || !flight || !flight->aircraft())
            {
                return 999999.0;
            }

            return GeoMath::getDistanceMeters(
                flight->aircraft()->location(),
                airport->header().datum()) / METERS_IN_1_NAUTICAL_MILE;
        }

        double altitudeAglFeet(shared_ptr<Flight> flight)
        {
            if (!flight || !flight->aircraft())
            {
                return 0.0;
            }

            const auto altitude = flight->aircraft()->altitude();
            switch (altitude.type())
            {
            case Altitude::Type::Ground:
                return 0.0;
            case Altitude::Type::AGL:
                return altitude.feet();
            case Altitude::Type::MSL:
            default:
                return altitude.feet() - host()->queryTerrainElevationAt(flight->aircraft()->location());
            }
        }

        double distanceToLandingPointNm(shared_ptr<Flight> flight)
        {
            if (!flight || !flight->aircraft())
            {
                return 999999.0;
            }

            try
            {
                return GeoMath::getDistanceMeters(
                    flight->aircraft()->location(),
                    m_helper.getLandingPoint(flight)) / METERS_IN_1_NAUTICAL_MILE;
            }
            catch (const exception&)
            {
                return 999999.0;
            }
        }

        bool shouldHandoffDepartureToArea(shared_ptr<Flight> flight)
        {
            return flight && (
                flight->phase() == Flight::Phase::EnRoute ||
                distanceToAirportNm(m_helper.getDepartureAirport(flight), flight) >= 25.0 ||
                altitudeAglFeet(flight) >= 8000.0);
        }

        bool shouldHandoffAreaToApproach(shared_ptr<Flight> flight)
        {
            return flight && (
                flight->phase() == Flight::Phase::Arrival ||
                distanceToAirportNm(m_helper.getArrivalAirport(flight), flight) <= 55.0);
        }

        bool shouldHandoffApproachToTower(shared_ptr<Flight> flight)
        {
            const double distanceNm = distanceToLandingPointNm(flight);
            const double aglFeet = altitudeAglFeet(flight);
            return flight && (
                distanceNm <= 12.0 ||
                (distanceNm <= 18.0 && aglFeet <= 3500.0));
        }

        shared_ptr<ControllerPosition> resolveNextController(shared_ptr<Flight> flight)
        {
            if (!flight || !position())
            {
                return nullptr;
            }

            switch (position()->type())
            {
            case ControllerPosition::Type::Departure:
                {
                    if (shouldHandoffDepartureToArea(flight))
                    {
                        auto area = m_helper.tryGetEnRouteArea(flight);
                        if (area && area != position())
                        {
                            return area;
                        }
                    }

                    if (shouldHandoffAreaToApproach(flight))
                    {
                        auto approach = m_helper.tryGetArrivalApproach(flight, flight->aircraft()->location());
                        if (approach && approach != position())
                        {
                            return approach;
                        }
                    }
                }
                break;
            case ControllerPosition::Type::Area:
                if (shouldHandoffAreaToApproach(flight))
                {
                    {
                        auto approach = m_helper.tryGetArrivalApproach(flight, flight->aircraft()->location());
                        if (approach && approach != position())
                        {
                            return approach;
                        }
                    }

                    {
                        auto tower = m_helper.tryGetArrivalTower(flight, m_helper.getLandingPoint(flight));
                        if (tower && tower != position())
                        {
                            return tower;
                        }
                    }
                }
                break;
            case ControllerPosition::Type::Approach:
                if (shouldHandoffApproachToTower(flight))
                {
                    auto tower = m_helper.tryGetArrivalTower(flight, m_helper.getLandingPoint(flight));
                    if (tower && tower != position())
                    {
                        return tower;
                    }
                }
                break;
            default:
                break;
            }

            return nullptr;
        }

        void handoffTrackedFlightsToNextController()
        {
            if (!position())
            {
                return;
            }

            switch (position()->type())
            {
            case ControllerPosition::Type::Departure:
            case ControllerPosition::Type::Area:
            case ControllerPosition::Type::Approach:
                break;
            default:
                return;
            }

            for (const auto& flight : m_radarTrackedFlights)
            {
                if (!flight)
                {
                    continue;
                }

                auto nextController = resolveNextController(flight);
                if (!nextController || nextController == position())
                {
                    continue;
                }

                const auto pending = m_pendingRadarHandoffTargetByFlightId.find(flight->id());
                if (pending != m_pendingRadarHandoffTargetByFlightId.end() && pending->second == nextController)
                {
                    continue;
                }

                auto handoff = I.controllerHandoff(flight, position(), nextController);
                if (!handoff)
                {
                    continue;
                }

                m_pendingRadarHandoffTargetByFlightId[flight->id()] = nextController;
                const int flightId = flight->id();
                transmit(handoff,
                    Frequency::noopTRansmissionCallback,
                    [this, flightId, nextController]() {
                        const auto pendingIt = m_pendingRadarHandoffTargetByFlightId.find(flightId);
                        return pendingIt == m_pendingRadarHandoffTargetByFlightId.end() || pendingIt->second != nextController;
                    });
            }
        }

        shared_ptr<Controller> findLocalControllerOrThrow(shared_ptr<Flight> flight)
        {
            const GeoPoint location = flight->aircraft()->location();
            vector<shared_ptr<ControllerPosition>> candidates;

            try
            {
                if (auto local = airport()->localAt(location))
                {
                    candidates.push_back(local);
                }
            }
            catch (const exception&)
            {
            }

            auto tower = airport()->tower();
            if (tower)
            {
                const vector<ControllerPosition::Type> fallbackTypes = {
                    ControllerPosition::Type::Local,
                    ControllerPosition::Type::Departure,
                    ControllerPosition::Type::Approach
                };

                for (const auto fallbackType : fallbackTypes)
                {
                    if (auto fallback = tower->tryFindPosition(fallbackType, location))
                    {
                        if (find(candidates.begin(), candidates.end(), fallback) == candidates.end())
                        {
                            candidates.push_back(fallback);
                        }
                    }
                }

                for (const auto& position : tower->positions())
                {
                    if (position && find(candidates.begin(), candidates.end(), position) == candidates.end())
                    {
                        candidates.push_back(position);
                    }
                }
            }

            for (const auto& position : candidates)
            {
                auto controller = position ? position->controller() : nullptr;
                if (controller)
                {
                    return controller;
                }
            }

            throw runtime_error("Could not find local controller at airport [" + airport()->header().icao() + "]");
        }

        bool isFlightDepartureRunway(shared_ptr<Flight> flight, const string& runwayName)
        {
            if (!hasKey(m_clearedForDepartureTaxi, flight) && !hasKey(m_departureTaxiHandedOffToTower, flight))
            {
                return false;
            }

            auto flightPlan = flight->plan();
            bool isDeparting = flight->phase() == Flight::Phase::Departure;

            auto departureRunway = isDeparting
               ? airport()->tryFindRunway(flightPlan->departureRunway())
               : nullptr;
            auto reportedRunway = runwayName.empty()
                ? nullptr
                : airport()->tryFindRunway(runwayName);
            bool isDepartureRunway = isDeparting && departureRunway && reportedRunway == departureRunway;

            host()->writeLog(
                "AICONT|PilotReportHoldingShortIntent, runway[%s] planned[%s] resolved[%s] ? %d",
                runwayName.c_str(),
                departureRunway ? departureRunway->name().c_str() : "N/A",
                reportedRunway ? reportedRunway->name().c_str() : "N/A",
                isDepartureRunway);

            return isDepartureRunway;
        }

        void handoffDeparturesToTower()
        {
            if (airport()->activeDepartureRunways().size() == 0)
            {
                return;
            }

            for (const auto &flight : m_clearedForDepartureTaxi)
            {
                if (hasKey(m_departureTaxiPendingHandoffToTower, flight))
                {
                    continue;
                }

                const Runway::End* departureEnd = nullptr;
                auto plannedRunwayName = flight->plan()->departureRunway();
                if (!plannedRunwayName.empty())
                {
                    try
                    {
                        departureEnd = &airport()->getRunwayEndOrThrow(plannedRunwayName);
                    }
                    catch (const exception&)
                    {
                        departureEnd = nullptr;
                    }
                }

                if (!departureEnd)
                {
                    const auto fallbackRunwayName = airport()->activeDepartureRunways().at(0);
                    departureEnd = &airport()->getRunwayEndOrThrow(fallbackRunwayName);
                }

                //host()->writeLog("AICONT|handoffDeparturesToTower:1");

                GeoPoint location = flight->aircraft()->location();
                float distanceMeters = GeoMath::getDistanceMeters(location, departureEnd->centerlinePoint().geo());

                if (distanceMeters <= 200)
                {
                    //host()->writeLog("AICONT|handoffDeparturesToTower:2");
                    host()->writeLog(
                        "AICONT|GND handing departure [%s] off to TWR",
                        flight->callSign().c_str());

                    shared_ptr<Flight> copyOfFlightPtr = flight;
                    m_departureTaxiPendingHandoffToTower.insert(copyOfFlightPtr);
                    transmit(
                        I.groundSwitchToTower(flight, 0),
                        [this, copyOfFlightPtr](shared_ptr<Transmission> transmission) {
                            moveDepartureToTower(copyOfFlightPtr);
                        },
                        [this, copyOfFlightPtr]{
                            return hasKey(m_departureTaxiHandedOffToTower, copyOfFlightPtr);
                        }
                    );
                    //host()->writeLog("AICONT|handoffDeparturesToTower:3");
                    //host()->writeLog("AICONT|handoffDeparturesToTower:4");
                }
            }

//            for (const auto& flight : handedOff)
//            {
//                //host()->writeLog("AICONT|handoffDeparturesToTower:5");
//                m_clearedForDepartureTaxi.erase(flight);
//                m_departureTaxiHandedOffToTower.insert(flight);
//            }
            //host()->writeLog("AICONT|handoffDeparturesToTower:6");
        }

        void moveDepartureToTower(shared_ptr<Flight> departure)
        {
            m_clearedForDepartureTaxi.erase(departure);
            m_departureTaxiPendingHandoffToTower.erase(departure);
            m_departureTaxiHandedOffToTower.insert(departure);
        }
    };
}
