// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#pragma once

#include <iostream>
#include <functional>
#include <utility>
#include <memory>
#include <thread>
#include <future>
#include <atomic>
#include <unordered_set>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <cmath>

// SDK
#include "XPLMPlugin.h"
#include "XPLMNavigation.h"
#if !XPLM300
#error This plugin requires version 300 of the SDK
#endif

// PPL 
#include "owneddata.h"

// tnc
#include "utils.h"
#include "libworld.h"
#include "libai.hpp"
#include "aircraftPerformanceTable.hpp"
#if IBM
#include "libserver.hpp"
#endif
#include "pluginHostServices.hpp"
#include "pluginMenu.hpp"
#include "nativeTextToSpeechService.hpp"
#include "xplaneWeatherService.hpp"
#include "simplePhraseologyService.hpp"
#include "demoScheduleLoader.hpp"
#include "xpmp2AircraftObjectService.hpp"
#include "xplmSpeakStringTtsService.hpp"
#include "pluginWorldLoader.hpp"
#include "configuration.hpp"
#include "userPilotAssistantWorkflow.hpp"
#include "userAircraft.hpp"
#include "userPilot.hpp"
#include "transcriptInterface.hpp"

using namespace std;
using namespace PPL;

//const char* poc_resource_directory = "/Users/felixb/Desktop/xp/Resources/plugins/tnc/Resources";
//static string resourceDirectory = "/Users/felixb/Desktop/xp/Resources/plugins/tnc/Resources";//PluginPath::prependPluginResourcesPath("");

class PluginInstance
{
private:

    enum class PluginStateId
    {
        Stopped = 0,
        WorldAssembling = 1,
        WorldAssembled = 2,
        SchedulesStarting = 3,
        SchedulesStarted = 4,
        Failed = 5
    };

    class PluginState
    {
    private:
        PluginStateId m_id;
        string m_name;
    protected:
        PluginState(PluginStateId _id, const string& _name) :
            m_id(_id),
            m_name(_name)
        {
        }
    public:
        PluginStateId id() const { return m_id; }
        const string& name() const { return m_name; }
    public:
        virtual void enter()
        {
        }
        virtual void exit()
        {
        }
        virtual void ping()
        {
        }
    };

    class StoppedState : public PluginState
    {
    private:
        shared_ptr<HostServices> m_host;
    public:
        StoppedState(shared_ptr<HostServices> _host) :
            PluginState(PluginStateId::Stopped, "STOPPED"),
            m_host(std::move(_host))
        {
        }
    public:
        void enter() override
        {
            stopServer();
        }
    private:
        void stopServer()
        {
#if IBM
            auto server = m_host->services().get<server::ServerControllerInterface>();

            if (server->running())
            {
                m_host->writeLog("PLUGIN|stopping the server");
                server->beginStop();
                if (server->waitUntilStopped(chrono::seconds(5)))
                {
                    m_host->writeLog("PLUGIN|server successfully stopped");
                }
                else
                {
                    m_host->writeLog("PLUGIN|WARNING: timeout waiting for server to stop");
                }
            }
#endif
        }
    };

    class FailedState : public PluginState
    {
    public:
        FailedState() :
            PluginState(PluginStateId::Failed, "FAILED")
        {
        }
    };

    class WorldAssemblingState : public PluginState
    {
    private:
        shared_ptr<PluginHostServices> m_host;
        future<shared_ptr<World>> m_worldFuture;
        atomic<bool> m_done;
        PluginMenu::Item m_assemblingItem;
        function<void(shared_ptr<World> world)> m_onAssembled;
        function<void()> m_onFailed;
    public:
        WorldAssemblingState(
            shared_ptr<PluginHostServices> _host,
            PluginMenu& _menu,
            function<void(shared_ptr<World> world)> _onAssembled,
            function<void()> _onFailed,
            unordered_set<string> _bubbleIcaos
        ) : PluginState(PluginStateId::WorldAssembling, "WORLD-ASSEMBLING"),
            m_host(std::move(_host)),
            m_assemblingItem(_menu, "World is being assembled, please wait...", [](){}),
            m_onAssembled(std::move(_onAssembled)),
            m_onFailed(std::move(_onFailed)),
            m_done(false),
            m_bubbleIcaos(std::move(_bubbleIcaos))
        {
        }

    public:

        void enter() override
        {
            m_worldFuture = std::async(std::launch::async, [this] {
                try
                {
                    return assembleWorld();
                }
                catch (const exception& e)
                {
                    m_host->writeLog("PLUGIN|WorldAssemblingState background task CRASHED!!! %s", e.what());
                    return shared_ptr<World>();
                }
            });
        }

        void exit() override
        {
            if (!m_done && m_worldFuture.wait_for(chrono::milliseconds(0)) != future_status::ready)
            {
                m_host->writeLog("PLUGIN|WARNING: leaving WORLD-ASSEMBLING state while the assembly is still in progress!");
            }
        }

        void ping() override
        {
            if (m_worldFuture.wait_for(chrono::milliseconds(0)) != future_status::ready)
            {
                m_host->writeLog("PLUGIN|ping WORLD-ASSEMBLING: in progress");
            }
            else if (m_worldFuture.valid())
            {
                m_host->writeLog("PLUGIN|ping WORLD-ASSEMBLING: done");
                auto world = m_worldFuture.get();
                m_done = true;

                if (world)
                {
                    m_host->useWorld(world);
                    startServer();
                    m_onAssembled(world);
                }
                else
                {
                    m_host->writeLog("PLUGIN|ERROR: world was not assembled - plugin will not function (see previous errors).");
                    m_onFailed();
                }
            }
        }

    private:
        unordered_set<string> m_bubbleIcaos;

        shared_ptr<World> assembleWorld()
        {
            try
            {
                PluginWorldLoader loader(m_host);
                if (!m_bubbleIcaos.empty())
                {
                    m_host->writeLog("PLUGIN|assembleWorld: loading %d airports within traffic bubble", static_cast<int>(m_bubbleIcaos.size()));
                    loader.loadWorld(m_bubbleIcaos);
                }
                else
                {
                    loader.loadWorld();
                }
                return loader.getWorld();
            }
            catch (const exception& e)
            {
                m_host->writeLog("PLUGIN|assembleWorld CRASHED!!! %s", e.what());
                return shared_ptr<World>();
            }
        }

        void startServer()
        {
#if IBM
            try
            {
                auto server = m_host->services().get<server::ServerControllerInterface>();

                if (!server->running())
                {
                    m_host->writeLog("PLUGIN|starting the server");
                    server->start(9002);
                }
            }
            catch (const exception& e)
            {
                m_host->writeLog("PLUGIN|startServer CRASHED!!! %s", e.what());
            }
#endif
        }
    };

    class WorldAssembledState : public PluginState
    {
    private:
        shared_ptr<HostServices> m_host;
        PluginMenu::Item m_startSchedules100Item;
        PluginMenu::Item m_startSchedules70Item;
        PluginMenu::Item m_startSchedules50Item;
        PluginMenu::Item m_startRandomTraffic100Item;
        PluginMenu::Item m_startRandomTraffic70Item;
        PluginMenu::Item m_startRandomTraffic50Item;
    public:
        WorldAssembledState(
            shared_ptr<HostServices> _host,
            PluginMenu& _menu,
            function<void(float loadFactor, bool offlineRandomTraffic)> _onStartSchedules
        ) : PluginState(PluginStateId::WorldAssembled, "WORLD-ASSEMBLED"),
            m_host(std::move(_host)),
            m_startSchedules100Item(_menu, "Start Schedules - 100% load", [=]{ _onStartSchedules(1.0, false); }),
            m_startSchedules70Item(_menu, "Start Schedules - 70% load", [=]{ _onStartSchedules(0.7, false); }),
            m_startSchedules50Item(_menu, "Start Schedules - 50% load", [=]{ _onStartSchedules(0.5, false); }),
            m_startRandomTraffic100Item(_menu, "Start Offline Random Traffic - 100% load", [=]{ _onStartSchedules(1.0, true); }),
            m_startRandomTraffic70Item(_menu, "Start Offline Random Traffic - 70% load", [=]{ _onStartSchedules(0.7, true); }),
            m_startRandomTraffic50Item(_menu, "Start Offline Random Traffic - 50% load", [=]{ _onStartSchedules(0.5, true); })
        {
        }
    };

    class SchedulesStartingState : public PluginState
    {
    private:
        shared_ptr<HostServices> m_host;
        shared_ptr<World> m_world;
        float m_loadFactor;
        bool m_offlineRandomTraffic;
        PluginMenu::Item m_worldIsStarting;
        function<void(shared_ptr<Airport> userAirport)> m_onStarted;
        shared_ptr<Airport> m_userAirport;
        future<shared_ptr<Airport>> m_scheduleFuture;
        atomic<bool> m_done;
    public:
        SchedulesStartingState(
            shared_ptr<HostServices> _host,
            shared_ptr<World> _world,
            PluginMenu& _menu,
            float _loadFactor,
            bool _offlineRandomTraffic,
            function<void(shared_ptr<Airport> userAirport)> _onStarted
        ) : PluginState(PluginStateId::SchedulesStarting, "SCHEDULES-STARTING"),
            m_host(std::move(_host)),
            m_world(_world),
            m_worldIsStarting(_menu, "Starting schedules, please wait...", [](){}),
            m_loadFactor(_loadFactor),
            m_offlineRandomTraffic(_offlineRandomTraffic),
            m_onStarted(std::move(_onStarted)),
            m_done(false)
        {
        }

        void enter() override
        {
            m_scheduleFuture = std::async(std::launch::async, [this] {
                try
                {
                    DemoScheduleLoader scheduleLoader(m_host, m_world);
                    scheduleLoader.loadSchedules(m_loadFactor, m_offlineRandomTraffic);

                    m_host->writeLog(
                        "PLUGIN|The world now has [%d] airports, [%d] control facilities, [%d] AI flights",
                        m_world->airports().size(),
                        m_world->controlFacilities().size(),
                        m_world->flights().size());

                    auto userAirport = scheduleLoader.airport();
                    WorldBuilder::tidyAirportElevations(m_host, userAirport);
                    return userAirport;
                }
                catch (const exception& e)
                {
                    m_host->writeLog("PLUGIN|SchedulesStartingState background task CRASHED!!! %s", e.what());
                    return shared_ptr<Airport>();
                }
            });
        }

        void ping() override
        {
            if (m_scheduleFuture.wait_for(chrono::milliseconds(0)) != future_status::ready)
            {
                return;
            }

            if (!m_scheduleFuture.valid())
            {
                return;
            }

            auto userAirport = m_scheduleFuture.get();
            m_done = true;

            if (userAirport)
            {
                logUserAirportElevations(userAirport);
                m_onStarted(userAirport);
            }
            else
            {
                m_host->writeLog("PLUGIN|ERROR: schedules failed to load - see previous errors");
            }
        }

    private:

        void logUserAirportElevations(const shared_ptr<Airport>& airport)
        {
            const auto logRunwayEndElevation = [this, &airport](const Runway::End& end) {
                const auto& centerlinePoint = end.centerlinePoint().geo();
                m_host->writeLog(
                    "PLUGIN|User airport elevations [%s/%s] at (%f,%f) elevation [%f]",
                    airport->header().icao().c_str(),
                    end.name().c_str(),
                    centerlinePoint.latitude,
                    centerlinePoint.longitude,
                    end.elevationFeet());
            };

            for (const auto& runway : airport->runways())
            {
                logRunwayEndElevation(runway->end1());
                logRunwayEndElevation(runway->end2());
            }
        }
    };

    class SchedulesStartedState : public PluginState
    {
    private:
        shared_ptr<HostServices> m_host;
        shared_ptr<World> m_world;
        shared_ptr<AircraftObjectService> m_aircraftObjectService;
        chrono::time_point<chrono::high_resolution_clock, chrono::microseconds> m_lastTickTime;
        //uint64_t m_timeFactor;
        shared_ptr<Airport> m_userAirport;
        shared_ptr<UserPilotAssistantWorkflow> m_userPilotWorkflow;
        shared_ptr<TranscriptInterface> m_transcript;
        shared_ptr<PluginMenu> m_userActionsMenu;
        PluginMenu::Item m_tuneClearanceItem;
        PluginMenu::Item m_tuneGroundItem;
        PluginMenu::Item m_tuneTowerItem;
        PluginMenu::Item m_toggleLabelsItem;
        PluginMenu::Item m_toggleDebugLabelsItem;
        PluginMenu::Item m_toggleAirnavSchedulesItem;
        PluginMenu::Item m_togglePlanefinderSchedulesItem;
        PluginMenu::Item m_time1XItem;
        PluginMenu::Item m_time10XItem;
        PluginMenu::Item m_time20XItem;
        PluginMenu::Item m_stopSchedulesItem;
        PluginMenu::Item m_restartSchedules100Item;
        PluginMenu::Item m_restartSchedules70Item;
        PluginMenu::Item m_restartSchedules50Item;
        bool m_offlineRandomTraffic;
        DataRef<int> m_com1FrequencyKhz;
        DataRef<int> m_simSpeed;
    public:
        SchedulesStartedState(
            shared_ptr<HostServices> _host,
            shared_ptr<World> _world,
            PluginMenu& _menu,
            shared_ptr<Airport> _userAirport,
            bool _offlineRandomTraffic,
            function<void()> _onStopSchedules,
            function<void(float loadFactor, bool offlineRandomTraffic)> _onRestartSchedules
        ) : PluginState(PluginStateId::SchedulesStarted, "SCHEDULES-STARTED"),
            m_host(_host),
            m_world(_world),
            m_userAirport(_userAirport),
            m_offlineRandomTraffic(_offlineRandomTraffic),
            m_transcript(_host->services().get<TranscriptInterface>()),
            m_userActionsMenu(make_shared<PluginMenu>("ATC Actions", _menu)),
            m_tuneClearanceItem(_menu, "COM1 " + _userAirport->header().icao() + " CLR/DEL", [this] { tuneToClearance(); }),
            m_tuneGroundItem(_menu, "COM1 " + _userAirport->header().icao() + " GND", [this] { tuneToGround(); }),
            m_tuneTowerItem(_menu, "COM1 " + _userAirport->header().icao() + " TWR", [this] { tuneToTower(); }),
            m_toggleLabelsItem(_menu, "Toggle AI aircraft labels", [this](){ toggleAIAircraftLabels(); }),
            m_toggleDebugLabelsItem(_menu, "Toggle debug info in labels", [this](){ toggleAIAircraftDebugLabels(); }),
            m_toggleAirnavSchedulesItem(_menu, "Toggle AirNavRadar live schedules", [this](){ toggleAirnavSchedules(); }),
            m_togglePlanefinderSchedulesItem(_menu, "Toggle Plane Finder live schedules", [this](){ togglePlanefinderSchedules(); }),
            m_time1XItem(_menu, "Time X 1", [=](){ m_simSpeed = 1; }),
            m_time10XItem(_menu, "Time X 10", [=](){ m_simSpeed = 10; }),
            m_time20XItem(_menu, "Time X 20", [=](){ m_simSpeed = 20; }),
            m_stopSchedulesItem(_menu, "Stop schedules", std::move(_onStopSchedules)),
            m_restartSchedules100Item(_menu, "Restart schedules with 100% load", [=]{_onRestartSchedules(1.0f, m_offlineRandomTraffic);}),
            m_restartSchedules70Item(_menu, "Restart schedules with 70% load", [=]{_onRestartSchedules(0.7f, m_offlineRandomTraffic);}),
            m_restartSchedules50Item(_menu, "Restart schedules with 50% load", [=]{_onRestartSchedules(0.5f, m_offlineRandomTraffic);}),
            m_com1FrequencyKhz("sim/cockpit2/radios/actuators/com1_frequency_hz_833", PPL::ReadWrite),
            m_simSpeed("sim/time/sim_speed", PPL::ReadWrite)
        {
            m_aircraftObjectService = m_host->services().get<AircraftObjectService>();
            m_lastTickTime = getNow();
            //m_timeFactor = 1;
        }

        void enter() override
        {
            m_transcript->setUserActionsMenu(m_userActionsMenu);
            initUserFlight();
        }

        void ping() override
        {
            auto now = getNow();
            auto microsecondsSinceLastTick = (now - m_lastTickTime) * m_simSpeed; // m_timeFactor;
            auto newWorldTimestamp = m_world->timestamp() + microsecondsSinceLastTick;
            m_lastTickTime = now;

            if (microsecondsSinceLastTick.count() == 0)
            {
                return;
            }

            m_world->progressTo(newWorldTimestamp);
            auto changeSet = m_world->hasChanges()
                ? m_world->takeChanges()
                : nullptr;

            if (changeSet)
            {
                m_aircraftObjectService->processEvents(changeSet);
            }
        }

    private:

        void tuneToClearance()
        {
            auto clearance = m_userAirport->clearanceDeliveryAt(m_userAirport->header().datum());
            m_com1FrequencyKhz = clearance->frequency()->khz();
        }

        void tuneToGround()
        {
            auto ground = m_userAirport->groundAt(m_userAirport->header().datum());
            m_com1FrequencyKhz = ground->frequency()->khz();
        }

        void tuneToTower()
        {
            auto tower = m_userAirport->localAt(m_userAirport->header().datum());
            m_com1FrequencyKhz = tower->frequency()->khz();
        }

        void toggleAIAircraftLabels()
        {
            auto config = m_host->services().get<PluginConfiguration>();
            config->showAIAircraftLabels = !config->showAIAircraftLabels;
            m_host->getWorld()->notifyConfigurationChanged();
        }

        void toggleAIAircraftDebugLabels()
        {
            auto config = m_host->services().get<PluginConfiguration>();
            config->showAIAircraftDebugLabels = !config->showAIAircraftDebugLabels;
            m_host->getWorld()->notifyConfigurationChanged();
        }

        void toggleAirnavSchedules()
        {
            auto config = m_host->services().get<PluginConfiguration>();
            config->enableAirnavSchedules = !config->enableAirnavSchedules;
            m_host->writeLog("PLUGIN|AirNavRadar live schedules %s", config->enableAirnavSchedules ? "enabled" : "disabled");
            m_host->getWorld()->notifyConfigurationChanged();
        }

        void togglePlanefinderSchedules()
        {
            auto config = m_host->services().get<PluginConfiguration>();
            config->enablePlanefinderSchedules = !config->enablePlanefinderSchedules;
            m_host->writeLog("PLUGIN|Plane Finder live schedules %s", config->enablePlanefinderSchedules ? "enabled" : "disabled");
            m_host->getWorld()->notifyConfigurationChanged();
        }

        void initUserFlight()
        {
            auto world = m_host->getWorld();
            auto departureTime = world->currentTime() + 30 * 60; //P-30
            auto arrivalTime = departureTime + 3 * 60 * 60; //3h
            auto userAircraft = UserAircraft::create(m_host);
            auto userModelIcao = userAircraft->modelIcao().empty() ? string("B738") : userAircraft->modelIcao();
            const auto userCategory = AircraftPerformanceTable::classifyFromIcao(userModelIcao);
            const float minimumRunwayLengthMeters = AircraftPerformanceTable::minimumRunwayLengthMeters(userCategory);
            auto destinationIcao = pickDefaultDestinationIcao(minimumRunwayLengthMeters);
            auto userAirlineIcao = userAircraft->airlineIcao();
            auto userFlightNo = userModelIcao;
            auto userCallsign = userAirlineIcao.empty() ? userFlightNo : userAirlineIcao + " " + userFlightNo;

            const auto selectSuitableRunway = [minimumRunwayLengthMeters](const shared_ptr<Airport>& airport, const vector<string>& candidateRunways) -> string
            {
                string bestRunway;
                float bestLengthMeters = -1.0f;

                if (!airport)
                {
                    return bestRunway;
                }

                for (const auto& runwayName : candidateRunways)
                {
                    try
                    {
                        const auto runway = airport->getRunwayOrThrow(runwayName);
                        if (!runway)
                        {
                            continue;
                        }

                        const float lengthMeters = runway->lengthMeters();
                        if (lengthMeters < minimumRunwayLengthMeters || lengthMeters <= bestLengthMeters)
                        {
                            continue;
                        }

                        bestLengthMeters = lengthMeters;
                        bestRunway = runwayName;
                    }
                    catch(const exception&)
                    {
                    }
                }

                if (!bestRunway.empty())
                {
                    return bestRunway;
                }

                try
                {
                    const auto longest = airport->findLongestRunway();
                    if (longest && longest->lengthMeters() >= minimumRunwayLengthMeters)
                    {
                        return longest->end1().name();
                    }
                }
                catch(const exception&)
                {
                }

                return bestRunway;
            };

            //this will be overridden by user's flight plan
            auto flightPlan = shared_ptr<FlightPlan>(new FlightPlan(
                departureTime,
                arrivalTime,
                m_userAirport->header().icao(),
                destinationIcao));

            flightPlan->setAirlineIcao(userAirlineIcao);
            flightPlan->setFlightNo(userFlightNo);
            flightPlan->setCallsign(userCallsign);

            //some of these will be overridden by user's flight plan
            auto userFlight = shared_ptr<Flight>(new Flight(
                m_host,
                1,
                Flight::RulesType::IFR,
                userAirlineIcao,
                userFlightNo,
                userCallsign,
                flightPlan));

            m_host->writeLog("initUserFlight:2");

            auto departureGate = m_userAirport->findClosestParkingStand(userAircraft->location());
            const string departureRunway = selectSuitableRunway(m_userAirport, m_userAirport->activeDepartureRunways());
            if (departureRunway.empty())
            {
                m_host->writeLog(
                    "UPILOT|INIT no suitable departure runway at airport[%s] for aircraft[%s]",
                    m_userAirport->header().icao().c_str(),
                    userModelIcao.c_str());
                return;
            }
            flightPlan->setDepartureGate(departureGate ? departureGate->name() : "N/A");
            flightPlan->setDepartureRunway(departureRunway);

            try
            {
                auto destinationAirport = m_host->getWorld()->getAirport(destinationIcao);
                if (!destinationAirport)
                {
                    throw runtime_error("destination airport not found");
                }
                const string arrivalRunway = selectSuitableRunway(destinationAirport, destinationAirport->activeArrivalRunways());
                if (arrivalRunway.empty())
                {
                    throw runtime_error("no suitable arrival runway found");
                }
                flightPlan->setArrivalAirportIcao(destinationAirport->header().icao());
                flightPlan->setArrivalRunway(arrivalRunway);
            }
            catch(const exception& e)
            {
                m_host->writeLog("UPILOT|INIT could not resolve destination airport [%s]: %s", destinationIcao.c_str(), e.what());
                return;
            }

            m_host->writeLog(
                "UPILOT|INIT departure gate=[%s] runway=[%s]",
                flightPlan->departureGate().c_str(), flightPlan->departureRunway().c_str());

            userFlight->setAircraft(userAircraft);

            m_host->writeLog("initUserFlight:4");

            auto userPilot = UserPilot::create(m_host, "Bob", Actor::Gender::Male, userFlight);

            m_host->writeLog("initUserFlight:5");

            userFlight->setPilot(userPilot);

            m_host->writeLog("initUserFlight:6");

            m_userPilotWorkflow = shared_ptr<UserPilotAssistantWorkflow>(new UserPilotAssistantWorkflow(
                m_host,
                userFlight,
                m_userAirport));

            m_world->addFlight(userFlight);

            m_host->writeLog("initUserFlight:7");
        }

        string pickDefaultDestinationIcao(float minimumRunwayLengthMeters)
        {
            const auto& airports = m_host->getWorld()->airports();
            if (airports.empty())
            {
                try
                {
                    const auto runway = m_userAirport->findLongestRunway();
                    if (runway && runway->lengthMeters() >= minimumRunwayLengthMeters)
                    {
                        return m_userAirport->header().icao();
                    }
                }
                catch(const exception&)
                {
                }

                return "";
            }

            shared_ptr<Airport> bestAirport;
            double bestScore = -1.0;
            const auto referenceDatum = m_userAirport->header().datum();

            const auto airportSupportsMinimumRunwayLength = [minimumRunwayLengthMeters](const shared_ptr<Airport>& airport) -> bool
            {
                if (!airport)
                {
                    return false;
                }

                try
                {
                    const auto runway = airport->findLongestRunway();
                    return runway && runway->lengthMeters() >= minimumRunwayLengthMeters;
                }
                catch(const exception&)
                {
                    return false;
                }
            };

            for (const auto& airport : airports)
            {
                if (!airport || airport->header().icao() == m_userAirport->header().icao())
                {
                    continue;
                }

                if (!airportSupportsMinimumRunwayLength(airport))
                {
                    continue;
                }

                double runwayLengthMeters = 0.0;
                try
                {
                    runwayLengthMeters = airport->findLongestRunway()->lengthMeters();
                }
                catch(const exception&)
                {
                }

                const double distanceMeters = GeoMath::getDistanceMeters(referenceDatum, airport->header().datum());
                const double score = distanceMeters + runwayLengthMeters * 10.0;
                if (score > bestScore)
                {
                    bestScore = score;
                    bestAirport = airport;
                }
            }

            if (bestAirport)
            {
                return bestAirport->header().icao();
            }

            if (airportSupportsMinimumRunwayLength(m_userAirport))
            {
                return m_userAirport->header().icao();
            }

            return "";
        }
    };

private:
    PluginMenu m_menu;
    shared_ptr<PluginState> m_currentState;
    shared_ptr<PluginHostServices> m_hostServices;
    shared_ptr<World> m_world;
    float m_schedulesLoadFactor;
    bool m_offlineRandomTraffic;
    DataRef<double> m_userAircraftLatitude;
    DataRef<double> m_userAircraftLongitude;
public:
    PluginInstance() :
        m_menu("Air Traffic & Control"),
        m_schedulesLoadFactor(0.7f),
        m_offlineRandomTraffic(false),
        m_userAircraftLatitude("sim/flightmodel/position/latitude", PPL::ReadOnly),
        m_userAircraftLongitude("sim/flightmodel/position/longitude", PPL::ReadOnly)
    {
        PrintDebugString("PLUGIN|initializing PluginInstance");

        m_hostServices = createHostServices();
        transitionToState([this]() { return createWorldAssemblingState(); });
        XPLMRegisterFlightLoopCallback(&pluginFlightLoopCallback, -1.0, this);
    }

    ~PluginInstance()
    {
        PrintDebugString("PLUGIN|destroying PluginInstance");

        XPLMUnregisterFlightLoopCallback(&pluginFlightLoopCallback, this);
        if (m_currentState->id() != PluginStateId::Stopped)
        {
            transitionToState([this]() { return createStoppedState(); });
        }
    }

public:

    void notifyAirportLoaded()
    {
        try
        {
            switch (m_currentState->id())
            {
            case PluginStateId::WorldAssembling:
                PrintDebugString("PLUGIN|notifyAirportLoaded pinging state [%s]", m_currentState->name().c_str());
                m_currentState->ping();
                break;
            case PluginStateId::SchedulesStarted:
                transitionToState([this] {
                    return createSchedulesStartingState(m_offlineRandomTraffic);
                });
                break;
            }
        }
        catch (const exception& e)
        {
            m_hostServices->writeLog("PLUGIN|notifyAirportLoaded CRASHED!!! %s", e.what());
        }
    }

private:

    shared_ptr<PluginHostServices> createHostServices()
    {
#if APL
        XPLMDebugString("------ BEGIN ALTEST ------\n");
        NativeTextToSpeechService::init_sound();
        XPLMDebugString("------ END ALTEST ------\n");
#endif

        auto hostServices = shared_ptr<PluginHostServices>(new PluginHostServices());
        auto configuration = make_shared<PluginConfiguration>();
        auto weatherService = shared_ptr<WeatherService>(new XPlaneWeatherService(hostServices));

#if LIN
        auto pluginTts = shared_ptr<XPLMSpeakStringTtsService>(new XPLMSpeakStringTtsService(hostServices));
#else
        auto pluginTts = shared_ptr<NativeTextToSpeechService>(new NativeTextToSpeechService(hostServices));
#endif

        auto phraseologyService = shared_ptr<PhraseologyService>(new SimplePhraseologyService(hostServices));
        auto aircraftObjectService = shared_ptr<Xpmp2AircraftObjectService>(new Xpmp2AircraftObjectService(hostServices));
        auto intentFactory = shared_ptr<IntentFactory>(new IntentFactory(hostServices));
        auto transcriptInterface = shared_ptr<TranscriptInterface>(new MenuBasedTranscriptInterface(hostServices));

        hostServices->services().use<PluginConfiguration>(configuration);
        hostServices->services().use<WeatherService>(weatherService);
        hostServices->services().use<AircraftObjectService>(aircraftObjectService);
        hostServices->services().use<TextToSpeechService>(pluginTts);
        hostServices->services().use<IntentFactory>(intentFactory);
        hostServices->services().use<PhraseologyService>(phraseologyService);
        hostServices->services().use<TranscriptInterface>(transcriptInterface);

#if IBM
        auto serverController = server::ServerControllerInterface::create(hostServices);
        hostServices->services().use<server::ServerControllerInterface>(serverController);
#endif

        ai::contributeComponents(hostServices);

        hostServices->writeLog("PLUGIN|host services initialized");
        return hostServices;
    }

    void transitionToState(function<shared_ptr<PluginState>()> factory)
    {
        if (m_currentState)
        {
            try
            {
                m_hostServices->writeLog("PLUGIN|exiting state[%s]", m_currentState->name().c_str());
                m_currentState->exit();
                m_currentState.reset();
            }
            catch (const exception &e)
            {
                PrintDebugString("PLUGIN|CRASHED while transitioning state (exit)!!! %s", e.what());
            }
        }

        try
        {
            m_currentState = factory();

            if (m_currentState)
            {
                m_hostServices->writeLog("PLUGIN|entering state[%s]", m_currentState->name().c_str());
                m_currentState->enter();
                m_hostServices->writeLog("PLUGIN|transitioned to state[%s]", m_currentState->name().c_str());
            }
        }
        catch (const exception &e)
        {
            PrintDebugString("PLUGIN|CRASHED while transitioning state (enter)!!! %s", e.what());
        }
    }

    shared_ptr<PluginState> createStoppedState()
    {
        return make_shared<StoppedState>(m_hostServices);
    }

    unordered_set<string> collectBubbleAirportIcaos()
    {
        unordered_set<string> icaos;

        auto configuration = m_hostServices->services().get<PluginConfiguration>();
        const float radiusNm = configuration ? configuration->trafficBubbleRadiusNm : 80.0f;

        const float userLat = static_cast<float>(static_cast<double>(m_userAircraftLatitude));
        const float userLon = static_cast<float>(static_cast<double>(m_userAircraftLongitude));

        m_hostServices->writeLog(
            "PLUGIN|collectBubbleAirportIcaos: user at (%f,%f) radius %.0f nm",
            userLat, userLon, radiusNm);

        const float radiusMeters = radiusNm * static_cast<float>(METERS_IN_1_NAUTICAL_MILE);

        XPLMNavRef navRef = XPLMFindFirstNavAidOfType(xplm_Nav_Airport);
        while (navRef != XPLM_NAV_NOT_FOUND)
        {
            XPLMNavType navType = xplm_Nav_Unknown;
            float lat = 0.0f, lon = 0.0f;
            char icaoId[64] = { 0 };

            XPLMGetNavAidInfo(navRef, &navType, &lat, &lon, nullptr, nullptr, nullptr, icaoId, nullptr, nullptr);
            if (navType != xplm_Nav_Airport)
            {
                break;
            }

            if (icaoId[0] != '\0')
            {
                const float dlat = lat - userLat;
                const float dlon = (lon - userLon) * cosf(userLat * 3.14159265f / 180.0f);
                const float approxDistMeters = sqrtf(dlat * dlat + dlon * dlon) * 111120.0f;

                if (approxDistMeters <= radiusMeters * 1.15f)
                {
                    string icaoStr(icaoId);
                    transform(icaoStr.begin(), icaoStr.end(), icaoStr.begin(), [](unsigned char c) {
                        return static_cast<char>(toupper(c));
                    });
                    icaos.insert(icaoStr);
                }
            }

            navRef = XPLMGetNextNavAid(navRef);
        }

        char nearestIcao[10] = { 0 };
        float lat = userLat, lon = userLon;
        XPLMNavRef nearestRef = XPLMFindNavAid(nullptr, nullptr, &lat, &lon, nullptr, xplm_Nav_Airport);
        if (nearestRef != XPLM_NAV_NOT_FOUND)
        {
            XPLMGetNavAidInfo(nearestRef, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nearestIcao, nullptr, nullptr);
            if (nearestIcao[0] != '\0')
            {
                string nearestStr(nearestIcao);
                transform(nearestStr.begin(), nearestStr.end(), nearestStr.begin(), [](unsigned char c) {
                    return static_cast<char>(toupper(c));
                });
                icaos.insert(nearestStr);
            }
        }

        m_hostServices->writeLog("PLUGIN|collectBubbleAirportIcaos: found %d airports within %.0f nm", static_cast<int>(icaos.size()), radiusNm);
        return icaos;
    }

    shared_ptr<PluginState> createWorldAssemblingState()
    {
        auto bubbleIcaos = collectBubbleAirportIcaos();

        const auto onAssembled = [this](shared_ptr<World> world) {
            m_hostServices->writeLog("PLUGIN|createWorldAssemblingState");
            m_world = world;
            transitionToState([this]() {
                return createWorldAssembledState();
            });
        };

        const auto onFailed = [this] {
            transitionToState([this]() {
                return createFailedState();
            });
        };

        return make_shared<WorldAssemblingState>(m_hostServices, m_menu, onAssembled, onFailed, std::move(bubbleIcaos));
    }

    shared_ptr<PluginState> createWorldAssembledState()
    {
        return make_shared<WorldAssembledState>(m_hostServices, m_menu, [this](float loadFactor, bool offlineRandomTraffic) {
            m_schedulesLoadFactor = loadFactor;
            m_offlineRandomTraffic = offlineRandomTraffic;
            transitionToState([this]() {
                return createSchedulesStartingState(m_offlineRandomTraffic);
            });
        });
    }

    shared_ptr<PluginState> createSchedulesStartingState(bool offlineRandomTraffic = false)
    {
        return make_shared<SchedulesStartingState>(
            m_hostServices,
            m_world,
            m_menu,
            m_schedulesLoadFactor,
            offlineRandomTraffic,
            [this, offlineRandomTraffic](shared_ptr<Airport> userAirport) {
                transitionToState([this, userAirport, offlineRandomTraffic]() {
                    return createSchedulesStartedState(userAirport, offlineRandomTraffic);
                });
            }
        );
    }

    shared_ptr<PluginState> createSchedulesStartedState(shared_ptr<Airport> userAirport, bool offlineRandomTraffic = false)
    {
        m_offlineRandomTraffic = offlineRandomTraffic;
        return make_shared<SchedulesStartedState>(
            m_hostServices,
            m_world,
            m_menu,
            userAirport,
            offlineRandomTraffic,
            [this]() {
                transitionToState([this]() {
                    return createWorldAssembledState();
                });
            },
            [this](float loadFactor, bool offlineRandomTraffic) {
                transitionToState([this, loadFactor, offlineRandomTraffic]() {
                    m_schedulesLoadFactor = loadFactor;
                    m_offlineRandomTraffic = offlineRandomTraffic;
                    return createSchedulesStartingState(offlineRandomTraffic);
                });
            }
        );
    }

    shared_ptr<PluginState> createFailedState()
    {
        return make_shared<FailedState>();
    }

    void flightLoopTick()
    {
        if (!m_currentState)
        {
            return;
        }

        try
        {
            m_currentState->ping();
        }
        catch(const exception& e)
        {
            m_hostServices->writeLog(
                "PLUGIN|flightLoopTick ping state[%s] CRASHED!!! %s",
                m_currentState->name().c_str(),
                e.what());
        }
    }

private:

    static chrono::time_point<chrono::high_resolution_clock, chrono::microseconds> getNow()
    {
        return std::chrono::time_point_cast<std::chrono::microseconds>(chrono::high_resolution_clock::now());
    }

    static float pluginFlightLoopCallback(
        float  inElapsedSinceLastCall,
        float  inElapsedTimeSinceLastFlightLoop,
        int    inCounter,
        void * inRefcon)
    {
        if (inRefcon)
        {
            static_cast<PluginInstance*>(inRefcon)->flightLoopTick();
        }

        return -1;
    }

};
