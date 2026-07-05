// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#pragma once

#include <cstring>
#include <string>
#include <chrono>
#include <queue>
#include <vector>
#include <initializer_list>
#include <unordered_set>
#include <unordered_map>
#include <limits>
#include <random>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <future>
#include <thread>

// SDK

#include "XPLMProcessing.h"
#include "XPLMNavigation.h"

// PPL
#include "owneddata.h"

// tnc
#include "utils.h"
#include "libworld.h"
#include "intentFactory.hpp"
#include "libdataxp.h"
#include "aiAircraft.hpp"
#include "aiPilot.hpp"
#include "libai.hpp"
#include "aircraftPerformanceTable.hpp"
#include "simplePhraseologyService.hpp"
#include "aircraftTypeReferenceTable.hpp"
#include "nativeTextToSpeechService.hpp"
#include "pluginHostServices.hpp"
#include "configuration.hpp"
#include "fr24AirportScheduleSource.hpp"
#include "airnavradarAirportScheduleSource.hpp"
#include "planefinderAirportScheduleSource.hpp"

using namespace std;
using namespace PPL;
using namespace world;
using namespace ai;

class DemoScheduleLoader
{
private:
    struct AirportLoadedException
    {
    };

    shared_ptr<HostServices> m_host;
    shared_ptr<World> m_world;
    DataRef<double> m_userAircraftLatitude;
    DataRef<double> m_userAircraftLongitude;
    shared_ptr<Airport> m_airport;

    // Cache for on-demand loaded airports to avoid repeated apt.dat scans
    unordered_map<string, shared_ptr<Airport>> m_airportLoadCache;
    // Cached apt.dat candidate paths to avoid repeated directory enumeration
    mutable vector<string> m_cachedAptDatCandidates;
    // Cached CIFP lookups to avoid re-reading procedure files for every flight
    mutable unordered_map<string, vector<string>> m_cachedProcedureLists;
    mutable unordered_map<string, vector<string>> m_cachedProcedureTracks;
    mutable unordered_map<string, XPCifpReader::ProcedureTrackWithLocations> m_cachedProcedureTracksWithLocations;
    // Gates already promised to live/demo schedules so random traffic can stay clear of deferred arrivals.
    unordered_set<string> m_reservedStandNames;
public:
    DemoScheduleLoader(shared_ptr<HostServices> _host, shared_ptr<World> _world) :
        m_host(_host),
        m_world(_world),
        m_userAircraftLatitude("sim/flightmodel/position/latitude", PPL::ReadOnly),
        m_userAircraftLongitude("sim/flightmodel/position/longitude", PPL::ReadOnly)
    {
    }
public:
    void loadSchedules(float loadFactor, bool offlineRandomTraffic = false)
    {
        m_reservedStandNames.clear();

        string userAirportIcao = getUserAirportIcao();
        m_airport = getAirportOrFallback(userAirportIcao);
        m_airport->selectActiveRunways();
        m_airport->selectArrivalAndDepartureTaxiways();
        logActiveRunwaysBounds();

        m_host->writeLog("SCHEDL|Loading schedules at airport[%s]", m_airport->header().icao().c_str());

        const bool liveSchedulesLoaded = loadLiveSchedules(loadFactor);

        if (!liveSchedulesLoaded)
        {
            m_host->writeLog("SCHEDL|No live schedule data available; using demo schedules instead");
            initDemoSchedules(loadFactor, m_world->currentTime() + 200, m_world->currentTime() + 30);
        }

        if (offlineRandomTraffic)
        {
            m_host->writeLog("SCHEDL|Offline random traffic mode enabled; adding complementary traffic");
            initRandomSchedules(loadFactor, m_world->currentTime() + 200, m_world->currentTime() + 30);
        }

        m_host->writeLog(
            "SCHEDL|Loaded [%d] AI flights at airport[%s]",
            m_world->flights().size(),
            m_airport->header().icao().c_str());
    }

public:

    shared_ptr<Airport> airport() const { return m_airport; }

private:

    void reserveTrafficStand(const shared_ptr<ParkingStand>& stand)
    {
        if (!stand)
        {
            return;
        }

        const string standName = stand->name();
        if (!standName.empty())
        {
            m_reservedStandNames.insert(standName);
        }
    }

    struct RandomTrafficModelPools
    {
        struct RandomTrafficModel
        {
            string icao;
            string name;
            world::Aircraft::Category category;
            float rangeNm = 500.0f;   // Aircraft range in nautical miles for destination selection
            int ceilingFl = 450;    // Maximum operating ceiling in flight levels
        };

        vector<RandomTrafficModel> gaModels;
        vector<RandomTrafficModel> helicopterModels;
        vector<RandomTrafficModel> militaryModels;
    };

    static string normalizeTrafficText(string value)
    {
        value.erase(remove_if(value.begin(), value.end(), [](unsigned char c) {
            return isspace(c);
        }), value.end());

        transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
            return static_cast<char>(toupper(c));
        });

        return value;
    }

    static bool containsAnyToken(const string& value, initializer_list<const char*> tokens)
    {
        const string upper = normalizeTrafficText(value);
        for (const char* token : tokens)
        {
            if (token && *token && upper.find(token) != string::npos)
            {
                return true;
            }
        }
        return false;
    }

    static bool hasMeaningfulTrafficText(const string& value)
    {
        const string normalized = normalizeTrafficText(value);
        return !normalized.empty() && normalized != "-" && normalized != "N/A" && normalized != "NONE" && normalized != "UNKNOWN";
    }

    RandomTrafficModelPools buildInstalledRandomTrafficModels() const
    {
        RandomTrafficModelPools pools;
        unordered_set<string> seenTypes;

        const int installedModelCount = XPMPGetNumberOfInstalledModels();
        if (installedModelCount <= 0)
        {
            if (m_host)
            {
                m_host->writeLog("SCHEDL|No installed CSL models are currently available for random traffic");
            }
            return pools;
        }

        for (int modelIndex = 0; modelIndex < installedModelCount; ++modelIndex)
        {
            string modelName;
            string icao;
            string airline;
            string livery;
            XPMPGetModelInfo2(modelIndex, modelName, icao, airline, livery);

            const string normalizedIcao = normalizeIcao(icao);
            if (normalizedIcao.empty() || !seenTypes.insert(normalizedIcao).second)
            {
                continue;
            }

            const auto category = AircraftPerformanceTable::classifyFromIcao(normalizedIcao);
            const bool hasMeaningfulAirline = hasMeaningfulTrafficText(airline);
            const bool hasMeaningfulLivery = hasMeaningfulTrafficText(livery);

            const bool isHelicopter = category == world::Aircraft::Category::Helicopter ||
                containsAnyToken(modelName, {
                    "HELI", "ROTOR", "HELO", "ROTORCRAFT", "JETRANGER", "LONGRANGER",
                    "HUEY", "ROBINSON", "SIKORSKY", "EUROCOPTER", "AIRBUSHELICOPTERS",
                    "AGUSTA", "AEROSPATIALE", "DAUPHIN", "COLIBRI", "ECUREUIL",
                    "PUMA", "SUPERPUMA", "BLACKHAWK", "SEAHAWK", "CHINOOK",
                    "STALLION", "COBRA", "APACHE", "MD500", "MD600", "MD902",
                    "BOEINGVERTOL", "VERTOL", "MIL"
                }) ||
                containsAnyToken(airline, {
                    "HELI", "ROTOR", "HELO", "ROTORCRAFT", "JETRANGER", "LONGRANGER",
                    "HUEY", "ROBINSON", "SIKORSKY", "EUROCOPTER", "AIRBUSHELICOPTERS",
                    "AGUSTA", "AEROSPATIALE", "DAUPHIN", "COLIBRI", "ECUREUIL",
                    "PUMA", "SUPERPUMA", "BLACKHAWK", "SEAHAWK", "CHINOOK",
                    "STALLION", "COBRA", "APACHE", "MD500", "MD600", "MD902",
                    "BOEINGVERTOL", "VERTOL", "MIL"
                }) ||
                containsAnyToken(livery, {
                    "HELI", "ROTOR", "HELO", "ROTORCRAFT", "JETRANGER", "LONGRANGER",
                    "HUEY", "ROBINSON", "SIKORSKY", "EUROCOPTER", "AIRBUSHELICOPTERS",
                    "AGUSTA", "AEROSPATIALE", "DAUPHIN", "COLIBRI", "ECUREUIL",
                    "PUMA", "SUPERPUMA", "BLACKHAWK", "SEAHAWK", "CHINOOK",
                    "STALLION", "COBRA", "APACHE", "MD500", "MD600", "MD902",
                    "BOEINGVERTOL", "VERTOL", "MIL"
                });

            const bool isMilitary = !isHelicopter && (
                category == world::Aircraft::Category::Fighter ||
                containsAnyToken(normalizedIcao, {
                    "F16", "F18", "FA18", "F15", "F22", "F35", "A10",
                    "C130", "C17", "C30J", "A400", "C5", "C2",
                    "B52", "B1", "B2", "E3", "E6", "E8",
                    "KC10", "KC13", "KC46", "KC30",
                    "P8", "P3", "V22",
                    "EUFI", "RFAL", "TORNA", "GRIPE", "MIRAG", "SU27", "SU30", "SU35", "MIG29", "MIG31",
                    "HAWK", "T38", "T45", "T6", "PC21", "PC9", "M346",
                    "U2", "SR71", "RQ4", "MQ9", "MQ1"
                }) ||
                containsAnyToken(modelName, { "MIL", "MILI", "ARMY", "NAVY", "USAF", "USN", "RAF", "AIRFORCE", "FIGHTER", "TANKER", "RECON", "AWACS", "BOMBER" }) ||
                containsAnyToken(airline, { "MIL", "MILI", "ARMY", "NAVY", "USAF", "USN", "RAF", "AIRFORCE", "FIGHTER", "TANKER", "RECON", "AWACS", "BOMBER" }) ||
                containsAnyToken(livery, { "MIL", "MILI", "ARMY", "NAVY", "USAF", "USN", "RAF", "AIRFORCE", "FIGHTER", "TANKER", "RECON", "AWACS", "BOMBER" }));

            const bool isGa = !isHelicopter && !isMilitary && (
                category == world::Aircraft::Category::LightProp ||
                category == world::Aircraft::Category::Prop ||
                AircraftPerformanceTable::isGeneralAviationTrafficType(normalizedIcao, category) ||
                (category == world::Aircraft::Category::Jet && !hasMeaningfulAirline && !hasMeaningfulLivery));

            if (isMilitary)
            {
                world::Aircraft::Category trafficCategory = category;
                if (containsAnyToken(normalizedIcao, { "C130", "C17", "C30J", "C5", "B52", "B1", "KC10", "KC13", "KC46", "KC30", "E3", "E6", "E8", "P8", "P3" }))
                {
                    trafficCategory = world::Aircraft::Category::Heavy;
                }
                else if (containsAnyToken(normalizedIcao, { "A400", "C2", "V22" }))
                {
                    trafficCategory = world::Aircraft::Category::Turboprop;
                }

                auto perf = AircraftPerformanceTable::lookup(m_host, normalizedIcao, trafficCategory);
                pools.militaryModels.push_back({ normalizedIcao, modelName, trafficCategory, perf.rangeNm, perf.ceilingFl });
            }
            else if (isHelicopter)
            {
                auto perf = AircraftPerformanceTable::lookup(m_host, normalizedIcao, world::Aircraft::Category::Helicopter);
                pools.helicopterModels.push_back({ normalizedIcao, modelName, world::Aircraft::Category::Helicopter, perf.rangeNm, perf.ceilingFl });
            }
            else if (isGa)
            {
                auto perf = AircraftPerformanceTable::lookup(m_host, normalizedIcao, category);
                pools.gaModels.push_back({ normalizedIcao, modelName, category, perf.rangeNm, perf.ceilingFl });
            }
        }

        if (m_host)
        {
            m_host->writeLog(
                "SCHEDL|Installed CSL model pools for random traffic: GA=%d Helicopter=%d Military=%d (installed=%d)",
                static_cast<int>(pools.gaModels.size()),
                static_cast<int>(pools.helicopterModels.size()),
                static_cast<int>(pools.militaryModels.size()),
                installedModelCount);
        }

        return pools;
    }

    string makeTrafficCallsignPrefix(const RandomTrafficModelPools::RandomTrafficModel& model) const
    {
        const string normalizedIcao = normalizeIcao(model.icao);

        AircraftTypeReferenceTable::Entry typeEntry;
        if (!normalizedIcao.empty() && AircraftTypeReferenceTable::tryFindByIcao(normalizedIcao, typeEntry))
        {
            if (!typeEntry.callsign.empty())
            {
                return typeEntry.callsign;
            }

            if (!typeEntry.name.empty())
            {
                return typeEntry.name;
            }
        }

        if (hasMeaningfulTrafficText(model.name))
        {
            return model.name;
        }

        return !normalizedIcao.empty() ? normalizedIcao : "Traffic";
    }

    shared_ptr<Airport> findNearestAirport(const GeoPoint& location, const string& excludedIcao = "", float minimumRunwayLengthMeters = 0.0f)
    {
        shared_ptr<Airport> bestAirport;
        double bestDistanceMeters = numeric_limits<double>::max();

        for (const auto& airport : m_world->airports())
        {
            if (!airport || airport->header().icao() == excludedIcao)
            {
                continue;
            }

            const auto& airportLocation = airport->header().datum();
            if (airportLocation == GeoPoint::empty)
            {
                continue;
            }

            if (minimumRunwayLengthMeters > 0.0f && !airportSupportsMinimumRunwayLength(airport, minimumRunwayLengthMeters))
            {
                continue;
            }

            const double distanceMeters = GeoMath::getDistanceMeters(location, airportLocation);
            if (distanceMeters < bestDistanceMeters)
            {
                bestDistanceMeters = distanceMeters;
                bestAirport = airport;
            }
        }

        return bestAirport;
    }

    static bool airportSupportsMinimumRunwayLength(const shared_ptr<Airport>& airport, float minimumRunwayLengthMeters)
    {
        if (!airport || minimumRunwayLengthMeters <= 0.0f)
        {
            return true;
        }

        try
        {
            const auto runway = airport->findLongestRunway();
            return runway && runway->lengthMeters() >= minimumRunwayLengthMeters;
        }
        catch (const exception&)
        {
            return false;
        }
    }

    static vector<string> filterSuitableRunways(
        const shared_ptr<Airport>& airport,
        const vector<string>& runwayNames,
        float minimumRunwayLengthMeters)
    {
        vector<string> result;
        if (!airport)
        {
            return result;
        }

        for (const auto& runwayName : runwayNames)
        {
            try
            {
                const auto runway = airport->getRunwayOrThrow(runwayName);
                if (runway && runway->lengthMeters() >= minimumRunwayLengthMeters)
                {
                    result.push_back(runwayName);
                }
            }
            catch (const exception&)
            {
            }
        }

        return result;
    }

    static string bestSuitableRunwayName(
        const shared_ptr<Airport>& airport,
        const vector<string>& runwayNames,
        float minimumRunwayLengthMeters)
    {
        if (!airport)
        {
            return "";
        }

        string bestName;
        float bestLengthMeters = -1.0f;

        const auto considerRunway = [&](const shared_ptr<Runway>& runway) {
            if (!runway)
            {
                return;
            }

            const float lengthMeters = runway->lengthMeters();
            if (lengthMeters < minimumRunwayLengthMeters)
            {
                return;
            }

            if (bestName.empty() || lengthMeters > bestLengthMeters)
            {
                bestName = runway->end1().name();
                bestLengthMeters = lengthMeters;
            }
        };

        for (const auto& runwayName : runwayNames)
        {
            try
            {
                considerRunway(airport->getRunwayOrThrow(runwayName));
            }
            catch (const exception&)
            {
            }
        }

        if (!bestName.empty())
        {
            return bestName;
        }

        try
        {
            considerRunway(airport->findLongestRunway());
        }
        catch (const exception&)
        {
        }

        return bestName;
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

    bool tryFindWaypointLocation(const shared_ptr<FlightPlan>& plan, const string& waypointName, GeoPoint& location) const
    {
        if (!plan || waypointName.empty())
        {
            return false;
        }

        const string normalizedWaypointName = normalizeWaypointName(waypointName);
            for (const auto& routeWaypoint : plan->knownWaypoints())
        {
            if (routeWaypoint.location == GeoPoint::empty)
            {
                continue;
            }

            if (normalizeWaypointName(routeWaypoint.name) == normalizedWaypointName)
            {
                location = routeWaypoint.location;
                return true;
            }
        }

        return false;
    }

    int estimateArrivalLeadSeconds(const shared_ptr<FlightPlan>& flightPlan) const
    {
        if (!flightPlan)
        {
            return 240;
        }

        int estimatedSeconds = 0;
        bool hasProcedureLeg = false;

        for (const auto& leg : flightPlan->legs())
        {
            if (!leg)
            {
                continue;
            }

            if (leg->type() != FlightPlan::LegType::Star &&
                leg->type() != FlightPlan::LegType::Approach &&
                leg->type() != FlightPlan::LegType::Landing)
            {
                continue;
            }

            GeoPoint fromPoint;
            GeoPoint toPoint;
            if (!tryFindWaypointLocation(flightPlan, leg->fromNavaid(), fromPoint) ||
                !tryFindWaypointLocation(flightPlan, leg->toNavaid(), toPoint))
            {
                continue;
            }

            const float targetSpeedKt = max(120.0f, leg->targetSpeed() > 0.0f ? leg->targetSpeed() : 180.0f);
            const double speedMetersPerSecond = targetSpeedKt * METERS_IN_1_NAUTICAL_MILE / 3600.0;
            if (speedMetersPerSecond <= 0.0)
            {
                continue;
            }

            const int legSeconds = max(10, static_cast<int>(GeoMath::getDistanceMeters(fromPoint, toPoint) / speedMetersPerSecond + 0.5));
            estimatedSeconds += legSeconds;
            hasProcedureLeg = true;
        }

        if (!hasProcedureLeg)
        {
            return 240;
        }

        return max(estimatedSeconds + 120, 240);
    }

    struct ProcedureRunwaySelection
    {
        string runwayName;
        string procedureName;
        int score = numeric_limits<int>::min();
    };


    static string normalizeProcedureToken(string value)
    {
        value.erase(remove_if(value.begin(), value.end(), [](unsigned char c) {
            return !isalnum(c);
        }), value.end());

        transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
            return static_cast<char>(toupper(c));
        });

        return value;
    }

    static string normalizeRunwayToken(const string& value)
    {
        string normalized = normalizeProcedureToken(value);
        if (normalized.rfind("RW", 0) == 0)
        {
            normalized.erase(0, 2);
        }
        return normalized;
    }

    static string buildCifpCacheKey(
        const string& airportIcao,
        const string& recordType,
        const string& procedureName = "",
        const string& runwayName = "",
        const string& preferredTransition = "")
    {
        return airportIcao + "|" + recordType + "|" + procedureName + "|" + runwayName + "|" + preferredTransition;
    }

    const vector<string>& getCachedProcedures(const string& airportIcao, const string& recordType) const
    {
        const string key = buildCifpCacheKey(airportIcao, recordType);
        auto inserted = m_cachedProcedureLists.emplace(key, vector<string>{});
        if (inserted.second)
        {
            XPCifpReader cifpReader(m_host);
            inserted.first->second = cifpReader.enumProcedures(airportIcao, recordType);
        }
        return inserted.first->second;
    }

    const vector<string>& getCachedProcedureTrack(
        const string& airportIcao,
        const string& recordType,
        const string& procedureName,
        const string& runwayName,
        const string& preferredTransition = "") const
    {
        const string key = buildCifpCacheKey(airportIcao, recordType, procedureName, runwayName, preferredTransition);
        auto inserted = m_cachedProcedureTracks.emplace(key, vector<string>{});
        if (inserted.second)
        {
            XPCifpReader cifpReader(m_host);
            inserted.first->second = cifpReader.readProcedureTrack(airportIcao, recordType, procedureName, runwayName, preferredTransition);
        }
        return inserted.first->second;
    }

    const XPCifpReader::ProcedureTrackWithLocations& getCachedProcedureTrackWithLocations(
        const string& airportIcao,
        const string& recordType,
        const string& procedureName,
        const string& runwayName,
        const string& preferredTransition = "") const
    {
        const string key = buildCifpCacheKey(airportIcao, recordType, procedureName, runwayName, preferredTransition);
        auto inserted = m_cachedProcedureTracksWithLocations.emplace(key, XPCifpReader::ProcedureTrackWithLocations{});
        if (inserted.second)
        {
            XPCifpReader cifpReader(m_host);
            inserted.first->second = cifpReader.readProcedureTrackWithLocations(airportIcao, recordType, procedureName, runwayName, preferredTransition);
        }
        return inserted.first->second;
    }

    bool tryResolveAirportLocation(const string& airportIcao, GeoPoint& location)
    {
        if (tryResolveLoadedAirportLocation(airportIcao, location))
        {
            return true;
        }

        const string normalizedIcao = normalizeIcao(airportIcao);
        if (normalizedIcao.empty())
        {
            return false;
        }

        if (auto airport = tryLoadAirportIntoWorld(normalizedIcao))
        {
            if (airport->header().datum() != GeoPoint::empty)
            {
                location = airport->header().datum();
                return true;
            }
        }

        return false;
    }

    bool tryResolveLoadedAirportLocation(const string& airportIcao, GeoPoint& location) const
    {
        const string normalizedIcao = normalizeIcao(airportIcao);
        if (normalizedIcao.empty())
        {
            return false;
        }

        try
        {
            auto airport = m_world->getAirport(normalizedIcao);
            if (airport && airport->header().datum() != GeoPoint::empty)
            {
                location = airport->header().datum();
                return true;
            }
        }
        catch (const exception&)
        {
        }

        return false;
    }

    static int scoreProcedureNameForRunway(const string& procedureName, const string& runwayName)
    {
        if (procedureName.empty() || runwayName.empty())
        {
            return 0;
        }

        const string procedureToken = normalizeProcedureToken(procedureName);
        const string runwayToken = normalizeRunwayToken(runwayName);
        if (procedureToken.empty() || runwayToken.empty())
        {
            return 0;
        }

        int score = 0;
        if (procedureToken.find(runwayToken) != string::npos)
        {
            score += 900;
        }

        if (runwayToken.size() >= 2)
        {
            const string runwayDigits = runwayToken.substr(0, 2);
            if (procedureToken.find(runwayDigits) != string::npos)
            {
                score += 250;
            }
        }

        return score;
    }

    bool tryResolveProcedureAnchorLocation(
        const string& airportIcao,
        const string& recordType,
        const string& procedureName,
        const string& runwayName,
        bool useFirstWaypoint,
        GeoPoint& location) const
    {
        const auto& track = getCachedProcedureTrack(airportIcao, recordType, procedureName, runwayName, "");
        if (track.empty())
        {
            return false;
        }

        XPNavaidReader navaidReader(m_host);
        const string runwayToken = normalizeRunwayToken(runwayName);

        const auto tryResolve = [&](const string& waypoint) {
            if (normalizeRunwayToken(waypoint) == runwayToken)
            {
                return false;
            }

            return navaidReader.tryResolveWaypoint(airportIcao, waypoint, location);
        };


        if (useFirstWaypoint)
        {
            for (const auto& waypoint : track)
            {
                if (tryResolve(waypoint))
                {
                    return true;
                }
            }
        }
        else
        {
            for (auto it = track.rbegin(); it != track.rend(); ++it)
            {
                if (tryResolve(*it))
                {
                    return true;
                }
            }
        }

        return false;
    }

    int scoreProcedureForRemoteAirport(
        const string& airportIcao,
        const string& recordType,
        const string& procedureName,
        const string& runwayName,
        const GeoPoint& airportLocation,
        const GeoPoint& remoteLocation,
        bool inboundProcedure) const
    {
        int score = scoreProcedureNameForRunway(procedureName, runwayName);

        if (airportLocation == GeoPoint::empty)
        {
            return score;
        }

        // Try to resolve remote location if not provided
        GeoPoint effectiveRemoteLocation = remoteLocation;
        if (effectiveRemoteLocation == GeoPoint::empty)
        {
            // For STAR: try to resolve from procedure name (often contains origin identifier)
            // For SID: try to resolve from first waypoint (often contains destination identifier)
            const auto& track = getCachedProcedureTrack(airportIcao, recordType, procedureName, runwayName, "");
            if (!track.empty())
            {
                XPNavaidReader navaidReader(m_host);
                GeoPoint resolvedLocation;
                // Try to resolve the first/last waypoint as a proxy for remote direction
                const string& anchorWaypoint = inboundProcedure ? track.front() : track.back();
                if (navaidReader.tryResolveWaypoint(airportIcao, anchorWaypoint, resolvedLocation))
                {
                    effectiveRemoteLocation = resolvedLocation;
                }
            }
        }

        if (effectiveRemoteLocation == GeoPoint::empty)
        {
            return score;
        }

        GeoPoint anchorLocation;
        const bool useFirstWaypoint = inboundProcedure;
        if (!tryResolveProcedureAnchorLocation(
                airportIcao,
                recordType,
                procedureName,
                runwayName,
                useFirstWaypoint,
                anchorLocation))
        {
            return score;
        }

        const float remoteBearing = inboundProcedure
            ? GeoMath::getHeadingFromPoints(effectiveRemoteLocation, airportLocation)
            : GeoMath::getHeadingFromPoints(airportLocation, effectiveRemoteLocation);
        const float anchorBearing = inboundProcedure
            ? GeoMath::getHeadingFromPoints(anchorLocation, airportLocation)  // For STAR: bearing from anchor to airport (inbound)
            : GeoMath::getHeadingFromPoints(airportLocation, anchorLocation); // For SID: bearing from airport to anchor (outbound)
        const float turnDegrees = static_cast<float>(fabs(GeoMath::getTurnDegrees(remoteBearing, anchorBearing)));

        score += max(0, 540 - static_cast<int>(turnDegrees * 4.0f));
        if (turnDegrees <= 30.0f)
        {
            score += 250;
        }
        else if (turnDegrees <= 60.0f)
        {
            score += 125;
        }
        else if (turnDegrees >= 135.0f)
        {
            score -= 150;
        }

        return score;
    }

    ProcedureRunwaySelection selectProcedureAndRunway(
        shared_ptr<Airport> airport,
        const vector<string>& candidateRunways,
        const string& recordType,
        const string& remoteAirportIcao,
        bool inboundProcedure,
        float minimumRunwayLengthMeters = 0.0f) const
    {
        ProcedureRunwaySelection bestSelection;
        if (!airport)
        {
            return bestSelection;
        }

        vector<string> runways = filterSuitableRunways(airport, candidateRunways, minimumRunwayLengthMeters);
        if (runways.empty())
        {
            try
            {
                for (const auto& runway : airport->runways())
                {
                    if (!runway)
                    {
                        continue;
                    }

                    if (minimumRunwayLengthMeters > 0.0f && runway->lengthMeters() < minimumRunwayLengthMeters)
                    {
                        continue;
                    }

                    runways.push_back(runway->end1().name());
                }

                if (runways.empty())
                {
                    const auto runway = (normalizeProcedureToken(recordType) == "STAR")
                        ? airport->findPreferredArrivalRunway()
                        : airport->findLongestRunway();
                    if (runway && runway->lengthMeters() >= minimumRunwayLengthMeters)
                    {
                        runways.push_back(runway->end1().name());
                    }
                }
            }
            catch (const exception&)
            {
            }
        }

        if (runways.empty())
        {
            return bestSelection;
        }

        GeoPoint airportLocation = airport->header().datum();
        GeoPoint remoteLocation = GeoPoint::empty;
        const bool hasRemoteLocation = tryResolveLoadedAirportLocation(remoteAirportIcao, remoteLocation);

        // Enumerate all procedures once outside the runway loop to avoid redundant file I/O
        // and ensure consistent selection across all runways
        const auto& allProcedures = getCachedProcedures(airport->header().icao(), recordType);

        for (const auto& runwayName : runways)
        {
            int runwayScore = 0;
            try
            {
                const auto runway = airport->getRunwayOrThrow(runwayName);
                if (!runway || runway->lengthMeters() < minimumRunwayLengthMeters)
                {
                    continue;
                }

                const auto& runwayEnd = runway->getEndOrThrow(runwayName);
                if (hasRemoteLocation && airportLocation != GeoPoint::empty)
                {
                    const float routeBearing = inboundProcedure
                        ? GeoMath::getHeadingFromPoints(remoteLocation, airportLocation)
                        : GeoMath::getHeadingFromPoints(airportLocation, remoteLocation);
                    const float turnDegrees = static_cast<float>(fabs(GeoMath::getTurnDegrees(routeBearing, runwayEnd.heading())));
                    runwayScore += max(0, 360 - static_cast<int>(turnDegrees * 3.0f));
                    if (turnDegrees <= 45.0f)
                    {
                        runwayScore += 150;
                    }
                }
            }
            catch (const exception&)
            {
            }

            // Skip procedure selection if no procedures available - just select best runway
            if (allProcedures.empty())
            {
                int totalScore = runwayScore;
                if (bestSelection.runwayName.empty() || totalScore > bestSelection.score)
                {
                    bestSelection.runwayName = runwayName;
                    bestSelection.procedureName = "";
                    bestSelection.score = totalScore;
                }
                continue;
            }

            string bestProcedureForRunway;
            int bestProcedureScore = numeric_limits<int>::min();
            for (const auto& procedureName : allProcedures)
            {
                const int procedureScore = scoreProcedureForRemoteAirport(
                    airport->header().icao(),
                    recordType,
                    procedureName,
                    runwayName,
                    airportLocation,
                    remoteLocation,
                    inboundProcedure);

                if (bestProcedureForRunway.empty() || procedureScore > bestProcedureScore)
                {
                    bestProcedureForRunway = procedureName;
                    bestProcedureScore = procedureScore;
                }
            }

            // Only use procedure score if it meets minimum threshold (avoid selecting bad matches)
            // Minimum threshold of 100 ensures at least some runway name matching or bearing alignment
            const int effectiveProcedureScore = (bestProcedureScore >= 100) ? bestProcedureScore : 0;
            const int totalScore = runwayScore + (bestProcedureForRunway.empty() ? 0 : effectiveProcedureScore);

            // Prefer this runway/procedure if:
            // 1. No selection made yet, OR
            // 2. This score is higher AND either has a valid procedure or previous also had none
            if (bestSelection.runwayName.empty() ||
                (totalScore > bestSelection.score &&
                 (!bestProcedureForRunway.empty() || bestSelection.procedureName.empty())))
            {
                bestSelection.runwayName = runwayName;
                bestSelection.procedureName = (effectiveProcedureScore > 0) ? bestProcedureForRunway : "";
                bestSelection.score = totalScore;
            }
        }

        return bestSelection;
    }

    string selectSidForRunway(const string& airportIcao, const string& runwayName)
    {
        try
        {
            const auto& sids = getCachedProcedures(airportIcao, "SID");
            if (sids.empty())
            {
                return "";
            }

            // Try to find a SID matching the runway
            string bestSid;
            for (const auto& sid : sids)
            {
                // If this SID name contains the runway number, it's likely for this runway
                if (sid.find(runwayName) != string::npos)
                {
                    return sid;
                }
                // Also check for common runway suffixes like "24L", "06R" etc
                if (runwayName.length() >= 2)
                {
                    string runwayDigits = runwayName.substr(0, 2);
                    if (sid.find(runwayDigits) != string::npos)
                    {
                        bestSid = sid;
                    }
                }
            }
            // Return best matching SID (may be empty if no good match found)
            // Let caller handle fallback instead of arbitrarily picking first alphabetically
            return bestSid;
        }
        catch (const exception&)
        {
            return "";
        }
    }

    string selectStarForRunway(const string& airportIcao, const string& runwayName)
    {
        try
        {
            const auto& stars = getCachedProcedures(airportIcao, "STAR");
            if (stars.empty())
            {
                return "";
            }

            // Try to find a STAR matching the runway
            string bestStar;
            for (const auto& star : stars)
            {
                // If this STAR name contains the runway number, it's likely for this runway
                if (star.find(runwayName) != string::npos)
                {
                    return star;
                }
                // Also check for common runway suffixes
                if (runwayName.length() >= 2)
                {
                    string runwayDigits = runwayName.substr(0, 2);
                    if (star.find(runwayDigits) != string::npos)
                    {
                        bestStar = star;
                    }
                }
            }
            // Return best matching STAR (may be empty if no good match found)
            // Let caller handle fallback instead of arbitrarily picking first alphabetically
            return bestStar;
        }
        catch (const exception&)
        {
            return "";
        }
    }

    string selectApproachForRunway(const string& airportIcao, const string& runwayName)
    {
        try
        {
            const auto& approaches = getCachedProcedures(airportIcao, "APPCH");
            if (approaches.empty())
            {
                return "";
            }

            string bestApproach;
            int bestScore = -1;
            const string runwayToken = normalizeRunwayToken(runwayName);

            for (const auto& approach : approaches)
            {
                int score = 0;
                string approachUpper = approach;
                transform(approachUpper.begin(), approachUpper.end(), approachUpper.begin(), ::toupper);

                if (approachUpper.find("ILS") != string::npos) score += 1000;
                else if (approachUpper.find("RNAV") != string::npos || approachUpper.find("RNP") != string::npos) score += 800;
                else if (approachUpper.find("GPS") != string::npos) score += 700;
                else if (approachUpper.find("VOR") != string::npos) score += 500;
                else if (approachUpper.find("NDB") != string::npos) score += 300;
                else if (approachUpper.find("LOC") != string::npos) score += 600;
                else score += 100;

                if (!runwayToken.empty())
                {
                    if (approachUpper.find(runwayToken) != string::npos)
                    {
                        score += 500;
                        if (approachUpper.find(runwayToken) == approachUpper.length() - runwayToken.length() ||
                            approachUpper.find(runwayToken + "Y") != string::npos ||
                            approachUpper.find(runwayToken + "Z") != string::npos ||
                            approachUpper.find("-" + runwayToken) != string::npos)
                        {
                            score += 300;
                        }
                    }
                    else if (runwayToken.length() >= 2)
                    {
                        string digits = runwayToken.substr(0, 2);
                        if (approachUpper.find(digits) != string::npos)
                        {
                            score += 100;
                        }
                    }
                }

                if (score > bestScore)
                {
                    bestScore = score;
                    bestApproach = approach;
                }
            }

            return bestApproach;
        }
        catch (const exception&)
        {
            return "";
        }
    }

    string selectStarTransition(
        const string& airportIcao,
        const string& starName,
        const string& runwayName,
        const GeoPoint& originLocation,
        const GeoPoint& airportLocation)
    {
        try
        {
            if (originLocation == GeoPoint::empty || airportLocation == GeoPoint::empty)
            {
                return "";
            }

            const auto& starWithLocations = getCachedProcedureTrackWithLocations(
                airportIcao, "STAR", starName, runwayName, "");

            if (starWithLocations.waypoints.empty())
            {
                return "";
            }

            const float inboundBearing = GeoMath::getHeadingFromPoints(originLocation, airportLocation);

            string bestTransition;
            float bestScore = -1.0f;
            unordered_map<string, vector<XPCifpReader::WaypointWithLocation>> transitions;
            string currentTransition;

            for (const auto& wp : starWithLocations.waypoints)
            {
                if (!wp.hasLocation && !wp.name.empty())
                {
                    currentTransition = wp.name;
                }
                else if (!currentTransition.empty())
                {
                    transitions[currentTransition].push_back(wp);
                }
            }

            if (transitions.empty() && starWithLocations.waypoints.size() >= 2)
            {
                for (size_t i = 0; i < min(size_t(3), starWithLocations.waypoints.size()); ++i)
                {
                    const auto& wp = starWithLocations.waypoints[i];
                    if (wp.hasLocation)
                    {
                        transitions[wp.name].push_back(wp);
                    }
                }
            }

            for (const auto& transition : transitions)
            {
                const auto& transitionName = transition.first;
                const auto& waypoints = transition.second;
                if (waypoints.empty()) continue;
                const auto& entryWp = waypoints.front();
                if (!entryWp.hasLocation) continue;

                const GeoPoint entryPoint(entryWp.latitude, entryWp.longitude);
                const float entryBearing = GeoMath::getHeadingFromPoints(entryPoint, airportLocation);

                float turnDegrees = static_cast<float>(fabs(GeoMath::getTurnDegrees(inboundBearing, entryBearing)));
                float score = 180.0f - turnDegrees;

                if (score > bestScore)
                {
                    bestScore = score;
                    bestTransition = transitionName;
                }
            }

            return bestTransition;
        }
        catch (const exception&)
        {
            return "";
        }
    }

    string getUserAirportIcao()
    {
        char airportIcaoId[10] = { 0 };
        float lat = m_userAircraftLatitude;
        float lon = m_userAircraftLongitude;
        m_host->writeLog("SCHEDL|User airport lookup: user aircraft is at (%f,%f)", lat, lon);

        XPLMNavRef navRef = XPLMFindNavAid( nullptr, nullptr, &lat, &lon, nullptr, xplm_Nav_Airport);
        if (navRef != XPLM_NAV_NOT_FOUND)
        {
            XPLMGetNavAidInfo(navRef, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, airportIcaoId, nullptr, nullptr);
        }

        if (strlen(airportIcaoId) > 0)
        {
            m_host->writeLog("SCHEDL|User airport lookup: FOUND [%s]", airportIcaoId);
            return airportIcaoId;
        }

        if (!m_world->airports().empty())
        {
            const GeoPoint userLocation((float)m_userAircraftLatitude, (float)m_userAircraftLongitude);
            auto fallbackAirport = findNearestAirport(userLocation);
            if (!fallbackAirport && !m_world->airports().empty())
            {
                fallbackAirport = m_world->airports().front();
            }

            if (fallbackAirport)
            {
                m_host->writeLog(
                    "SCHEDL|User airport lookup: NOT FOUND! - falling back to nearest loaded airport [%s]",
                    fallbackAirport->header().icao().c_str());
                return fallbackAirport->header().icao();
            }
        }

        m_host->writeLog("SCHEDL|User airport lookup: NOT FOUND! - no airports available");
        return "";
    }

    shared_ptr<Airport> getAirportOrFallback(const string& icao)
    {
        try
        {
            return m_world->getAirport(icao);
        }
        catch(const exception&)
        {
        }

        if (auto loadedAirport = tryLoadAirportIntoWorld(icao))
        {
            m_host->writeLog(
                "SCHEDL|Airport [%s] not found in world - loaded on demand from apt.dat as [%s]",
                icao.c_str(),
                loadedAirport->header().icao().c_str());
            return loadedAirport;
        }

        if (!m_world->airports().empty())
        {
            const GeoPoint userLocation((float)m_userAircraftLatitude, (float)m_userAircraftLongitude);
            auto fallbackAirport = findNearestAirport(userLocation, icao);
            if (!fallbackAirport && !m_world->airports().empty())
            {
                fallbackAirport = m_world->airports().front();
            }

            if (fallbackAirport)
            {
                m_host->writeLog(
                    "SCHEDL|Airport [%s] not found in world - using nearest fallback airport [%s]",
                    icao.c_str(),
                    fallbackAirport->header().icao().c_str());
                return fallbackAirport;
            }
        }

        throw runtime_error("DemoScheduleLoader::getAirportOrFallback: no airports available in world");
    }

    vector<string> buildAptDatCandidates() const
    {
        if (!m_cachedAptDatCandidates.empty())
        {
            return m_cachedAptDatCandidates;
        }

        vector<string> candidatePaths;

        appendAptDatCandidatesForSceneryRoot("Custom Scenery", candidatePaths);
        appendAptDatCandidatesForSceneryRoot("Global Scenery", candidatePaths);

        candidatePaths.push_back(m_host->getHostFilePath({ "Resources", "default scenery", "default apt dat", "Earth nav data", "apt.dat" }));

        m_cachedAptDatCandidates = candidatePaths;
        return m_cachedAptDatCandidates;
    }

    void appendAptDatCandidatesForSceneryRoot(const string& sceneryRoot, vector<string>& candidatePaths) const
    {
        try
        {
            for (const auto& sceneryPack : m_host->findFilesInHostDirectory({ sceneryRoot }))
            {
                if (sceneryPack.empty() || sceneryPack == "." || sceneryPack == ".." || sceneryPack == "scenery_packs.ini")
                {
                    continue;
                }

                candidatePaths.push_back(m_host->getHostFilePath({ sceneryRoot, sceneryPack, "Earth nav data", "apt.dat" }));
            }
        }
        catch(const exception& e)
        {
            m_host->writeLog("SCHEDL|failed to enumerate %s packages: %s", sceneryRoot.c_str(), e.what());
        }
    }

    string normalizeIcao(const string& value) const
    {
        string result = value;
        transform(result.begin(), result.end(), result.begin(), [](unsigned char c) {
            return static_cast<char>(toupper(c));
        });
        return result;
    }

    shared_ptr<Airport> tryLoadAirportIntoWorld(const string& icao)
    {
        const string normalizedIcao = normalizeIcao(icao);
        if (normalizedIcao.empty())
        {
            return nullptr;
        }

        // Check cache first
        auto cacheIt = m_airportLoadCache.find(normalizedIcao);
        if (cacheIt != m_airportLoadCache.end())
        {
            return cacheIt->second;
        }

        const auto candidatePaths = buildAptDatCandidates();
        for (const auto& candidatePath : candidatePaths)
        {
            try
            {
                auto input = m_host->openFileForRead(candidatePath);
                if (!input)
                {
                    continue;
                }

                shared_ptr<Airport> foundAirport;
                XPAptDatReader aptDatReader(m_host);

                try
                {
                    aptDatReader.readAptDat(
                        *input,
                        WorldBuilder::assembleSampleAirportControlZone,
                        [&](const Airport::Header& header) {
                            return normalizeIcao(header.icao()) == normalizedIcao;
                        },
                        [&](shared_ptr<Airport> airport) {
                            foundAirport = airport;
                            throw AirportLoadedException();
                        },
                        false);
                }
                catch (const AirportLoadedException&)
                {
                }

                if (foundAirport)
                {
                    const string loadedIcao = normalizeIcao(foundAirport->header().icao());
                    if (loadedIcao == normalizedIcao)
                    {
                        auto addedAirport = m_world->addAirport(foundAirport);
                        // Store in cache to avoid repeated scans
                        m_airportLoadCache[normalizedIcao] = addedAirport;
                        return addedAirport;
                    }

                    m_host->writeLog(
                        "SCHEDL|Airport [%s] loaded from [%s] but resolved as [%s] - ignoring mismatched airport",
                        icao.c_str(),
                        candidatePath.c_str(),
                        foundAirport->header().icao().c_str());
                }
            }
            catch(const exception&)
            {
            }
        }

        // Cache negative result to avoid repeated failed scans
        m_airportLoadCache[normalizedIcao] = nullptr;
        return nullptr;
    }

    string pickOtherAirportIcao(const string& referenceIcao, float minimumRunwayLengthMeters = 0.0f)
    {
        const auto& airports = m_world->airports();
        if (airports.empty())
        {
            return referenceIcao;
        }

        shared_ptr<Airport> referenceAirport;
        try
        {
            referenceAirport = m_world->getAirport(referenceIcao);
        }
        catch(const exception&)
        {
        }

        if (!referenceAirport)
        {
            for (const auto& airport : airports)
            {
                if (airport && airport->header().icao() != referenceIcao && airportSupportsMinimumRunwayLength(airport, minimumRunwayLengthMeters))
                {
                    return airport->header().icao();
                }
            }

            // Return first airport as ultimate fallback
            for (const auto& airport : airports)
            {
                if (airport && airportSupportsMinimumRunwayLength(airport, minimumRunwayLengthMeters))
                {
                    return airport->header().icao();
                }
            }
            return "";
        }

        double bestScore = -1.0;
        string bestIcao;

        for (const auto& airport : airports)
        {
            if (!airport || airport->header().icao() == referenceIcao)
            {
                continue;
            }

            if (!airportSupportsMinimumRunwayLength(airport, minimumRunwayLengthMeters))
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

            const double distanceMeters = GeoMath::getDistanceMeters(
                referenceAirport->header().datum(),
                airport->header().datum());
            // Favour closer airports with good runways for realistic regional traffic.
            // Previous formula (distance + runway*10) picked the farthest airport.
            const double distanceNm = distanceMeters / 1852.0;
            const double proximityScore = max(0.0, 500.0 - distanceNm);
            const double score = proximityScore + runwayLengthMeters * 0.01;

            if (score > bestScore)
            {
                bestScore = score;
                bestIcao = airport->header().icao();
            }
        }

        if (bestIcao.empty() && referenceAirport && airportSupportsMinimumRunwayLength(referenceAirport, minimumRunwayLengthMeters))
        {
            return referenceIcao;
        }

        return bestIcao;
    }

    void initDemoSchedules(float loadFactor, time_t firstDepartureTime, time_t firstArrivalTime)
    {
        unordered_map<string, string> callSignByAirline = {
            { "DAL", "Delta" },
            { "AAL", "American" },
            { "SWA", "Southwest" },
        };

        string activeDepartureRunway;
        string activeArrivalRunway1;
        string activeArrivalRunway2;
        int arrivalIndex = 0;
        const string localIcao = m_airport->header().icao();
        const float minimumRunwayLengthMeters = AircraftPerformanceTable::minimumRunwayLengthMeters(world::Aircraft::Category::Jet);
        const string fallbackIcao = pickOtherAirportIcao(localIcao, minimumRunwayLengthMeters);

        const auto findActiveRunways = [this, &activeDepartureRunway, &activeArrivalRunway1, &activeArrivalRunway2] {
            const auto& departure = m_airport->activeDepartureRunways();
            const auto& arrival = m_airport->activeArrivalRunways();

            activeDepartureRunway = !departure.empty() ? departure.at(0) : "";
            activeArrivalRunway1 = !arrival.empty() ? arrival.at(0) : "";
            activeArrivalRunway2 = !arrival.empty() ? arrival.at(arrival.size() - 1) : "";
        };

        const auto addOutboundFlight = [this, &callSignByAirline, &activeDepartureRunway, &minimumRunwayLengthMeters](
            const string& model, const string& airline, int flightId, const string& destination, time_t departureTime, shared_ptr<ParkingStand> gate
        ) {
            if (destination.empty())
            {
                throw runtime_error("SCHEDL|no suitable outbound destination airport available");
            }

            reserveTrafficStand(gate);

            string callSign = getValueOrThrow(callSignByAirline, airline);
            auto flightPlan = shared_ptr<FlightPlan>(new FlightPlan(departureTime, departureTime + 60 * 60 * 3, m_airport->header().icao(), destination));
            flightPlan->setDepartureGate(gate->name());

            auto departureSelection = selectProcedureAndRunway(
                m_airport,
                m_airport->activeDepartureRunways(),
                "SID",
                destination,
                false,
                minimumRunwayLengthMeters);
            const string departureRunway = departureSelection.runwayName.empty()
                ? bestSuitableRunwayName(m_airport, m_airport->activeDepartureRunways(), minimumRunwayLengthMeters)
                : departureSelection.runwayName;
            if (departureRunway.empty())
            {
                throw runtime_error("SCHEDL|no suitable outbound departure runway available");
            }
            flightPlan->setDepartureRunway(departureRunway);
            if (!departureSelection.procedureName.empty())
            {
                flightPlan->setSid(departureSelection.procedureName);
            }

            auto destinationAirport = m_host->getWorld()->getAirport(destination);
            if (!destinationAirport)
            {
                throw runtime_error("SCHEDL|no suitable random outbound destination airport object available");
            }

            auto arrivalSelection = selectProcedureAndRunway(
                destinationAirport,
                destinationAirport->activeArrivalRunways(),
                "STAR",
                m_airport->header().icao(),
                true,
                minimumRunwayLengthMeters);
            const string arrivalRunway = arrivalSelection.runwayName.empty()
                ? bestSuitableRunwayName(destinationAirport, destinationAirport->activeArrivalRunways(), minimumRunwayLengthMeters)
                : arrivalSelection.runwayName;
            if (arrivalRunway.empty())
            {
                throw runtime_error("SCHEDL|no suitable outbound arrival runway available");
            }
            flightPlan->setArrivalRunway(arrivalRunway);
            if (!arrivalSelection.procedureName.empty())
            {
                flightPlan->setStar(arrivalSelection.procedureName);
            }

            auto flight = shared_ptr<Flight>(new Flight(m_host, flightId, Flight::RulesType::IFR, airline, to_string(flightId), callSign + " " + to_string(flightId), flightPlan));

            auto aircraft = m_host->createAIAircraft(model, airline, to_string(flightId), world::Aircraft::Category::Jet);
            flight->setAircraft(aircraft);

            auto pilot = m_host->createAIPilot(flight);
            flight->setPilot(pilot);
            flight->setPhase(Flight::Phase::TurnAround);

            m_world->addFlightColdAndDark(flight);
        };

        const auto addInboundFlight = [this, &callSignByAirline, &activeArrivalRunway1, &activeArrivalRunway2, &arrivalIndex, &minimumRunwayLengthMeters](
            const string& model, const string& airline, int flightId, const string& origin, time_t arrivalTime, shared_ptr<ParkingStand> gate
        ) {
            m_host->writeLog("SCHEDL|adding inbound flight id[%d]", flightId);
            if (origin.empty())
            {
                throw runtime_error("SCHEDL|no suitable inbound origin airport available");
            }

            reserveTrafficStand(gate);

            string callSign = getValueOrThrow(callSignByAirline, airline);
            auto arrivalSelection = selectProcedureAndRunway(
                m_airport,
                m_airport->activeArrivalRunways(),
                "STAR",
                origin,
                true,
                minimumRunwayLengthMeters);
            string arrivalRunway = arrivalSelection.runwayName.empty()
                ? bestSuitableRunwayName(m_airport, { activeArrivalRunway1, activeArrivalRunway2 }, minimumRunwayLengthMeters)
                : arrivalSelection.runwayName;
            if (arrivalRunway.empty())
            {
                throw runtime_error("SCHEDL|no suitable inbound arrival runway available");
            }
            auto flightPlan = shared_ptr<FlightPlan>(new FlightPlan(arrivalTime - 60 * 60 * 3, arrivalTime, origin, m_airport->header().icao()));
            flightPlan->setArrivalGate(gate->name());
            flightPlan->setArrivalRunway(arrivalRunway);

            if (!arrivalSelection.procedureName.empty())
            {
                flightPlan->setStar(arrivalSelection.procedureName);

                // Select STAR transition based on origin airport bearing
                GeoPoint originLocation = GeoPoint::empty;
                GeoPoint airportLocation = m_airport->header().datum();
                if (tryResolveLoadedAirportLocation(origin, originLocation) && airportLocation != GeoPoint::empty)
                {
                    string transition = selectStarTransition(
                        m_airport->header().icao(),
                        arrivalSelection.procedureName,
                        arrivalRunway,
                        originLocation,
                        airportLocation);
                    if (!transition.empty())
                    {
                        flightPlan->setStarTransition(transition);
                    }
                }
            }

            // Select approach for arrival runway
            string approach = selectApproachForRunway(m_airport->header().icao(), arrivalRunway);
            if (!approach.empty())
            {
                flightPlan->setApproach(approach);
            }

            auto flight = shared_ptr<Flight>(new Flight(m_host, flightId, Flight::RulesType::IFR, airline, to_string(flightId), callSign + " " + to_string(flightId), flightPlan));

            auto aircraft = m_host->createAIAircraft(model, airline, to_string(flightId), world::Aircraft::Category::Jet);
            flight->setAircraft(aircraft);

            auto pilot = m_host->createAIPilot(flight);
            flight->setPilot(pilot);
            flight->setPhase(Flight::Phase::Arrival);

            auto copyOfWorld = m_world;
            auto copyOfAirport = m_airport;
            const time_t arrivalStartTime = max(arrivalTime - 240, m_world->currentTime() + 30);
            m_world->deferUntil(
                "addInboundFlight/" + flight->callSign(),
                arrivalStartTime,
                [flight, copyOfWorld, copyOfAirport, arrivalRunway](){
                    const auto& landingRunwayEnd = copyOfWorld->getRunwayEnd(copyOfAirport->header().icao(), arrivalRunway);
                    copyOfWorld->addFlight(flight);
                    flight->aircraft()->setOnFinal(landingRunwayEnd);
                }
            );
        };

        const float normalSecondsBetweenDepartures = 210;
        const float normalSecondsBetweenArrivals = 210;
        const float normalLoadFactor = 0.7f;
        int secondsBetweenDepartures = normalSecondsBetweenDepartures * normalLoadFactor / loadFactor;
        int secondsBetweenArrivals = normalSecondsBetweenArrivals * normalLoadFactor / loadFactor;
        string outboundDestinationIcao = pickOtherAirportIcao(m_airport->header().icao(), minimumRunwayLengthMeters);
        string inboundOriginIcao = pickOtherAirportIcao(m_airport->header().icao(), minimumRunwayLengthMeters);

        m_host->writeLog(
            "SCHEDL|LOADFACTOR [%f] secondsBetweenArrivals=[%d] secondsBetweenDepartures=[%d]",
            loadFactor, secondsBetweenArrivals, secondsBetweenDepartures);

        vector<shared_ptr<ParkingStand>> gates;
        findGatesForFlights(gates, loadFactor);
        findActiveRunways();

        int index = 0;
        time_t nextDepartureTime = firstDepartureTime;
        time_t nextArrivalTime = firstArrivalTime;

        vector<string> airlineOptions = { "DAL", "AAL", "SWA" };
        vector<string> modelOptions = { "B738" /*, "A320"*/ };

        for (const auto& gate : gates)
        {
            index++;
            int flightId = 100 + index;
            const string& airline = airlineOptions[index % airlineOptions.size()];
            const string& model = modelOptions[index % modelOptions.size()];
            
            try
            {
                if ((index % 2) == 1)
                {
                    time_t departureTime = nextDepartureTime;
                    nextDepartureTime += secondsBetweenDepartures;
                    addOutboundFlight(model, airline, flightId, outboundDestinationIcao, departureTime, gate);
                }
                else
                {
                    time_t arrivalTime = nextArrivalTime;
                    nextArrivalTime += secondsBetweenArrivals;
                    addInboundFlight(model, airline, flightId, inboundOriginIcao, arrivalTime, gate);
                }
            }
            catch(const std::exception& e)
            {
                m_host->writeLog("SCHEDL|CRASHED while adding AI flight!!! %s", e.what());
            }
        }
    }

    void initRandomSchedules(float loadFactor, time_t firstDepartureTime, time_t firstArrivalTime)
    {
        const string localIcao = m_airport->header().icao();
        string activeDepartureRunway;
        string activeArrivalRunway1;
        string activeArrivalRunway2;
        const auto findActiveRunways = [this, &activeDepartureRunway, &activeArrivalRunway1, &activeArrivalRunway2] {
            const auto& departure = m_airport->activeDepartureRunways();
            const auto& arrival = m_airport->activeArrivalRunways();

            activeDepartureRunway = !departure.empty() ? departure.at(0) : "";
            activeArrivalRunway1 = !arrival.empty() ? arrival.at(0) : "";
            activeArrivalRunway2 = !arrival.empty() ? arrival.at(arrival.size() - 1) : "";
        };
        findActiveRunways();

        const auto randomTrafficModels = buildInstalledRandomTrafficModels();

        const auto pickRandomIndex = [this](size_t maxValue) -> size_t {
            if (maxValue == 0)
            {
                return 0;
            }
            return static_cast<size_t>(m_host->getNextRandom(static_cast<int>(maxValue)) % static_cast<int>(maxValue));
        };

        const auto makeCallSign = [&](const RandomTrafficModelPools::RandomTrafficModel& model, int flightId) {
            return makeTrafficCallsignPrefix(model) + " " + to_string(flightId);
        };

        // RVSM semi-circular cruising levels (ICAO Doc 7030).
        // Eastbound (000-179): odd thousands; Westbound (180-359): even thousands.
        const auto applyRvsmSemiCircular = [](float desiredAltFeet, float routeHeading) -> float {
            const bool eastbound = routeHeading >= 0.0f && routeHeading < 180.0f;
            const int desiredFL = static_cast<int>(desiredAltFeet / 100.0f);
            if (desiredAltFeet >= 29000.0f)
            {
                const int baseFL = eastbound ? 290 : 300;
                int fl = baseFL;
                while (fl + 20 <= desiredFL) { fl += 20; }
                return static_cast<float>(fl) * 100.0f;
            }
            const int thousands = static_cast<int>(desiredAltFeet / 1000.0f);
            if (eastbound)
                return (thousands % 2 == 1) ? thousands * 1000.0f : (thousands - 1) * 1000.0f;
            else
                return (thousands % 2 == 0) ? thousands * 1000.0f : (thousands - 1) * 1000.0f;
        };

        // Helper to calculate appropriate cruise altitude (capped at aircraft ceiling)
        const auto calculateCruiseAltitudeFeet = [this, &applyRvsmSemiCircular](int ceilingFl, Flight::RulesType rulesType, float routeHeading) -> float {
            float maxAltitudeFeet = ceilingFl * 100.0f;

            if (rulesType == Flight::RulesType::VFR)
            {
                // VFR hemispheric rule: odd+500 eastbound, even+500 westbound
                const bool eastbound = routeHeading >= 0.0f && routeHeading < 180.0f;
                float baseAltitude = min(8500.0f, maxAltitudeFeet * 0.8f);
                const int eastSteps[] = { 3500, 5500, 7500 };
                const int westSteps[] = { 4500, 6500, 8500 };
                const int stepCount = 3;
                int selectedIndex = m_host->getNextRandom(stepCount);
                float selected = eastbound
                    ? static_cast<float>(eastSteps[selectedIndex])
                    : static_cast<float>(westSteps[selectedIndex]);
                return min(selected, baseAltitude);
            }

            // IFR: choose cruise altitude based on aircraft ceiling, then apply RVSM
            float desiredAltitudeFeet = 24000.0f;
            if (ceilingFl <= 150)
            {
                desiredAltitudeFeet = min(10000.0f, maxAltitudeFeet * 0.9f);
            }
            else if (ceilingFl <= 250)
            {
                desiredAltitudeFeet = min(18000.0f, maxAltitudeFeet * 0.9f);
            }
            else if (ceilingFl <= 350)
            {
                desiredAltitudeFeet = min(26000.0f, maxAltitudeFeet * 0.9f);
            }

            float capped = min(desiredAltitudeFeet, maxAltitudeFeet * 0.95f);
            return applyRvsmSemiCircular(capped, routeHeading);
        };

        // Helper to convert meters to nautical miles
        const auto metersToNm = [](double meters) -> float {
            return static_cast<float>(meters / 1852.0);
        };

        // Find airports within range of the local airport, filtering by maxRangeNm
        const auto findAirportsWithinRange = [this, &localIcao, &metersToNm](float maxRangeNm, float minimumRunwayLengthMeters) -> vector<shared_ptr<Airport>> {
            vector<shared_ptr<Airport>> validAirports;
            const auto localAirport = m_world->getAirport(localIcao);
            if (!localAirport)
            {
                return validAirports;
            }

            const auto& localLocation = localAirport->header().datum();
            for (const auto& airport : m_world->airports())
            {
                if (!airport || airport->header().icao() == localIcao)
                {
                    continue;
                }

                const double distanceMeters = GeoMath::getDistanceMeters(localLocation, airport->header().datum());
                const float distanceNm = metersToNm(distanceMeters);
                if (distanceNm <= maxRangeNm && airportSupportsMinimumRunwayLength(airport, minimumRunwayLengthMeters))
                {
                    validAirports.push_back(airport);
                }
            }

            sort(validAirports.begin(), validAirports.end(), [&](const shared_ptr<Airport>& lhs, const shared_ptr<Airport>& rhs) {
                const double lhsDistance = GeoMath::getDistanceMeters(localLocation, lhs->header().datum());
                const double rhsDistance = GeoMath::getDistanceMeters(localLocation, rhs->header().datum());
                if (abs(lhsDistance - rhsDistance) > 1.0)
                {
                    return lhsDistance < rhsDistance;
                }
                return lhs->header().icao() < rhs->header().icao();
            });

            return validAirports;
        };

        const auto pickAirportIcaoByPreferenceAndRange = [this, &localIcao, &pickRandomIndex, &findAirportsWithinRange](bool preferNearby, float maxRangeNm, float minimumRunwayLengthMeters) {
            // Get airports within the aircraft's range
            auto airportsInRange = findAirportsWithinRange(maxRangeNm, minimumRunwayLengthMeters);

            if (airportsInRange.empty())
            {
                // Fallback: if no airports in range, use the nearest airport that still satisfies runway length
                auto nearestAirport = findNearestAirport(m_airport->header().datum(), localIcao, minimumRunwayLengthMeters);
                if (nearestAirport)
                {
                    return nearestAirport->header().icao();
                }
                return pickOtherAirportIcao(localIcao, minimumRunwayLengthMeters);
            }

            if (preferNearby)
            {
                // Return the nearest airport from those in range
                return airportsInRange.front()->header().icao();
            }

            // Randomly pick from airports in range
            const size_t index = pickRandomIndex(airportsInRange.size());
            return airportsInRange[index]->header().icao();
        };

        const auto isUserAircraftParkedAtStand = [this](const shared_ptr<ParkingStand>& stand)->bool {
            GeoPoint userAircraftLocation((float)m_userAircraftLatitude, (float)m_userAircraftLongitude);
            auto distanceToUserAircraft = GeoMath::getDistanceMeters(userAircraftLocation, stand->location().geo());
            return (distanceToUserAircraft < 50);
        };

        unordered_set<string> occupiedStandNames = m_reservedStandNames;
        for (const auto& flight : m_world->flights())
        {
            if (!flight || !flight->plan())
            {
                continue;
            }

            const auto& plan = flight->plan();
            if (plan->departureAirportIcao() == localIcao && !plan->departureGate().empty())
            {
                occupiedStandNames.insert(plan->departureGate());
            }
            if (plan->arrivalAirportIcao() == localIcao && !plan->arrivalGate().empty())
            {
                occupiedStandNames.insert(plan->arrivalGate());
            }
        }

        const auto isOccupiedByExistingTraffic = [&](const shared_ptr<ParkingStand>& stand) {
            return (stand && occupiedStandNames.find(stand->name()) != occupiedStandNames.end());
        };

        const auto nameContainsAny = [](const string& value, initializer_list<const char*> tokens) {
            string upper = value;
            transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) {
                return static_cast<char>(toupper(c));
            });

            for (const char* token : tokens)
            {
                if (strstr(upper.c_str(), token))
                {
                    return true;
                }
            }

            return false;
        };

        const auto isMilitaryStand = [&](const shared_ptr<ParkingStand>& stand) {
            return (
                stand->hasOperationType(world::Aircraft::OperationType::Military) ||
                stand->hasAircraftCategory(world::Aircraft::Category::Fighter) ||
                nameContainsAny(stand->name(), { "MIL", "MILI", "RAMP", "BASE", "ARMY" }) ||
                stand->type() == ParkingStand::Type::Hangar ||
                stand->type() == ParkingStand::Type::Remote);
        };

        const auto isHelicopterStand = [&](const shared_ptr<ParkingStand>& stand) {
            return (
                stand->hasAircraftCategory(world::Aircraft::Category::Helicopter) ||
                nameContainsAny(stand->name(), { "HEL", "HELI", "HANG", "PAD", "ROTOR" }) ||
                stand->type() == ParkingStand::Type::Hangar ||
                stand->type() == ParkingStand::Type::Remote);
        };

        const auto isGeneralAviationStand = [&](const shared_ptr<ParkingStand>& stand) {
            return (
                stand->hasOperationType(world::Aircraft::OperationType::GA) ||
                stand->hasAircraftCategory(world::Aircraft::Category::LightProp) ||
                stand->hasAircraftCategory(world::Aircraft::Category::Prop) ||
                stand->hasAircraftCategory(world::Aircraft::Category::Turboprop) ||
                stand->hasAircraftCategory(world::Aircraft::Category::Jet) ||
                nameContainsAny(stand->name(), { "GA", "GENERAL", "GRASS", "DIRT", "TIE", "FUEL" }));
        };

        enum class TrafficBias
        {
            GeneralAviation,
            Helicopter,
            Military
        };

        const auto countAirportStandsMatching = [&](const shared_ptr<Airport>& airport, const function<bool(const shared_ptr<ParkingStand>&)>& predicate) -> size_t {
            if (!airport)
            {
                return 0;
            }

            size_t count = 0;
            for (const auto& stand : airport->parkingStands())
            {
                if (stand && predicate(stand))
                {
                    ++count;
                }
            }
            return count;
        };

        const auto longestAirportRunwayMeters = [](const shared_ptr<Airport>& airport) -> float {
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
        };

        const auto airportLooksLikeInternationalHub = [&](const shared_ptr<Airport>& airport) {
            if (!airport)
            {
                return false;
            }

            const float longestRunwayMeters = longestAirportRunwayMeters(airport);
            const size_t runwayCount = airport->runways().size();
            const size_t standCount = airport->parkingStands().size();
            const bool nameSuggestsHub = nameContainsAny(airport->header().name(), {
                "INTERNATIONAL", "INTL", "INTERCONTINENTAL"
            });

            if (airport->parallelRunwayGroupCount() > 1 || runwayCount >= 4 || standCount >= 60)
            {
                return true;
            }

            if (nameSuggestsHub && (longestRunwayMeters >= 2400.0f || standCount >= 18 || !airport->header().iata().empty()))
            {
                return true;
            }

            return longestRunwayMeters >= 3400.0f && (standCount >= 28 || !airport->header().iata().empty());
        };

        const auto airportHasMilitaryActivity = [&](const shared_ptr<Airport>& airport) {
            return countAirportStandsMatching(airport, isMilitaryStand) > 0 ||
                nameContainsAny(airport ? airport->header().name() : string(), {
                    "AFB", "AIR BASE", "AIRBASE", "AIR FORCE", "MILITARY",
                    "NAVAL", "NAS ", "JOINT", "ARMY", "BASE"
                });
        };

        const auto airportLooksGaFriendly = [&](const shared_ptr<Airport>& airport) {
            if (!airport || airportLooksLikeInternationalHub(airport))
            {
                return false;
            }

            const float longestRunwayMeters = longestAirportRunwayMeters(airport);
            const size_t standCount = airport->parkingStands().size();
            const size_t gaStandCount = countAirportStandsMatching(airport, isGeneralAviationStand);
            return gaStandCount > 0 ||
                longestRunwayMeters <= 2600.0f ||
                standCount <= 20 ||
                airport->header().iata().empty();
        };

        const auto chooseMilitaryMissionProfile = [this](const RandomTrafficModelPools::RandomTrafficModel& model) {
            const string& icao = model.icao;
            const bool isFighter = model.category == world::Aircraft::Category::Fighter;
            const bool isTransport = containsAnyToken(icao, { "C130", "C17", "C30J", "C5", "C2", "A400", "V22", "KC10", "KC13", "KC46", "KC30" });
            const bool isBomber = containsAnyToken(icao, { "B52", "B1", "B2" });
            const bool isIsr = containsAnyToken(icao, { "E3", "E6", "E8", "P8", "P3", "U2", "RQ4", "MQ9", "MQ1" });

            if (isFighter)
            {
                const int missionRoll = m_host->getNextRandom(100);
                if (missionRoll < 34)
                {
                    return AIAircraft::MissionProfile::Training;
                }
                if (missionRoll < 72)
                {
                    return AIAircraft::MissionProfile::Patrol;
                }
                return AIAircraft::MissionProfile::LowLevel;
            }
            if (isTransport)
            {
                const int missionRoll = m_host->getNextRandom(100);
                return missionRoll < 70 ? AIAircraft::MissionProfile::Patrol : AIAircraft::MissionProfile::Training;
            }
            if (isBomber || isIsr)
            {
                return AIAircraft::MissionProfile::Patrol;
            }
            return AIAircraft::MissionProfile::None;
        };

        const auto missionProfileName = [](AIAircraft::MissionProfile missionProfile) -> const char* {
            switch (missionProfile)
            {
            case AIAircraft::MissionProfile::Training: return "training";
            case AIAircraft::MissionProfile::Patrol:   return "patrol";
            case AIAircraft::MissionProfile::LowLevel: return "low_level";
            default:                                   return "none";
            }
        };

        const auto adjustCruiseAltitudeForMission = [](float baseCruiseAltitudeFeet,
                                                       const shared_ptr<Airport>& fromAirport,
                                                       const shared_ptr<Airport>& toAirport,
                                                       const RandomTrafficModelPools::RandomTrafficModel& model,
                                                       AIAircraft::MissionProfile missionProfile) -> float {
            if (missionProfile == AIAircraft::MissionProfile::None || model.category != world::Aircraft::Category::Fighter)
            {
                return baseCruiseAltitudeFeet;
            }

            const float maxAltitudeFeet = model.ceilingFl * 100.0f;
            const float highestFieldElevationFeet = max(
                fromAirport ? fromAirport->header().elevation() : 0.0f,
                toAirport ? toAirport->header().elevation() : 0.0f);

            switch (missionProfile)
            {
            case AIAircraft::MissionProfile::Training:
                return min(baseCruiseAltitudeFeet, min(maxAltitudeFeet * 0.50f, max(6000.0f, highestFieldElevationFeet + 6500.0f)));
            case AIAircraft::MissionProfile::Patrol:
                return min(baseCruiseAltitudeFeet, min(maxAltitudeFeet * 0.65f, max(9000.0f, highestFieldElevationFeet + 12000.0f)));
            case AIAircraft::MissionProfile::LowLevel:
                return min(baseCruiseAltitudeFeet, min(maxAltitudeFeet * 0.35f, max(2500.0f, highestFieldElevationFeet + 2500.0f)));
            default:
                return baseCruiseAltitudeFeet;
            }
        };

        const auto pickAirportIcaoForTraffic = [this,
                                                &localIcao,
                                                &metersToNm,
                                                &pickRandomIndex,
                                                &findAirportsWithinRange,
                                                &airportLooksLikeInternationalHub,
                                                &airportHasMilitaryActivity,
                                                &airportLooksGaFriendly,
                                                &countAirportStandsMatching,
                                                &isGeneralAviationStand,
                                                &isHelicopterStand,
                                                &isMilitaryStand,
                                                &longestAirportRunwayMeters](
            const RandomTrafficModelPools::RandomTrafficModel& model,
            TrafficBias trafficBias,
            bool preferNearbyAirport,
            AIAircraft::MissionProfile missionProfile) {
            const float minimumRunwayLengthMeters = AircraftPerformanceTable::minimumRunwayLengthMeters(model.category);
            auto airportsInRange = findAirportsWithinRange(model.rangeNm, minimumRunwayLengthMeters);

            if (airportsInRange.empty())
            {
                auto nearestAirport = findNearestAirport(m_airport->header().datum(), localIcao, minimumRunwayLengthMeters);
                if (nearestAirport)
                {
                    return nearestAirport->header().icao();
                }
                return pickOtherAirportIcao(localIcao, minimumRunwayLengthMeters);
            }

            struct Candidate
            {
                shared_ptr<Airport> airport;
                float distanceNm;
                double score;
            };

            vector<Candidate> candidates;
            candidates.reserve(airportsInRange.size());

            for (const auto& airport : airportsInRange)
            {
                const float distanceNm = metersToNm(GeoMath::getDistanceMeters(m_airport->header().datum(), airport->header().datum()));
                const size_t gaStandCount = countAirportStandsMatching(airport, isGeneralAviationStand);
                const size_t heliStandCount = countAirportStandsMatching(airport, isHelicopterStand);
                const size_t militaryStandCount = countAirportStandsMatching(airport, isMilitaryStand);
                const size_t standCount = airport->parkingStands().size();
                const float longestRunwayMeters = longestAirportRunwayMeters(airport);
                const bool hub = airportLooksLikeInternationalHub(airport);

                double score = preferNearbyAirport
                    ? max(0.0, 90.0 - distanceNm * 0.45)
                    : max(0.0, 40.0 - distanceNm * 0.08);

                switch (trafficBias)
                {
                case TrafficBias::GeneralAviation:
                    if (hub)
                    {
                        score -= 180.0;
                    }
                    if (airportLooksGaFriendly(airport))
                    {
                        score += 45.0;
                    }
                    score += min(54.0, static_cast<double>(gaStandCount) * 12.0);
                    if (longestRunwayMeters >= 650.0f && longestRunwayMeters <= 2400.0f)
                    {
                        score += 28.0;
                    }
                    else if (longestRunwayMeters > 3200.0f)
                    {
                        score -= 55.0;
                    }
                    if (standCount <= 20)
                    {
                        score += 20.0;
                    }
                    else if (standCount > 45)
                    {
                        score -= 35.0;
                    }
                    if (distanceNm <= 120.0f)
                    {
                        score += 20.0;
                    }
                    break;

                case TrafficBias::Helicopter:
                    if (hub)
                    {
                        score -= 70.0;
                    }
                    score += min(60.0, static_cast<double>(heliStandCount) * 18.0);
                    if (distanceNm <= 80.0f)
                    {
                        score += 26.0;
                    }
                    if (standCount <= 25)
                    {
                        score += 12.0;
                    }
                    break;

                case TrafficBias::Military:
                    score += min(90.0, static_cast<double>(militaryStandCount) * 18.0);
                    if (airportHasMilitaryActivity(airport))
                    {
                        score += 55.0;
                    }
                    else
                    {
                        score -= 45.0;
                    }
                    if (hub)
                    {
                        score -= 25.0;
                    }
                    if (missionProfile == AIAircraft::MissionProfile::Training || missionProfile == AIAircraft::MissionProfile::LowLevel)
                    {
                        score += max(0.0, 42.0 - distanceNm * 0.22);
                    }
                    else if (missionProfile == AIAircraft::MissionProfile::Patrol)
                    {
                        score += max(0.0, 25.0 - abs(distanceNm - 220.0f) * 0.05);
                    }
                    break;
                }

                candidates.push_back({ airport, distanceNm, score });
            }

            sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
                if (abs(lhs.score - rhs.score) > 0.001)
                {
                    return lhs.score > rhs.score;
                }
                if (abs(lhs.distanceNm - rhs.distanceNm) > 0.01f)
                {
                    return lhs.distanceNm < rhs.distanceNm;
                }
                return lhs.airport->header().icao() < rhs.airport->header().icao();
            });

            const size_t selectionWindow = min(candidates.size(), preferNearbyAirport ? size_t(3) : size_t(6));
            const size_t selectedIndex = selectionWindow > 1 ? pickRandomIndex(selectionWindow) : 0;
            return candidates.at(selectedIndex).airport->header().icao();
        };

        vector<shared_ptr<ParkingStand>> remainingStands;
        for (const auto& stand : m_airport->parkingStands())
        {
            if (stand && !isUserAircraftParkedAtStand(stand) && !isOccupiedByExistingTraffic(stand))
            {
                remainingStands.push_back(stand);
            }
        }

        vector<shared_ptr<ParkingStand>> militaryStands;
        vector<shared_ptr<ParkingStand>> helicopterStands;
        vector<shared_ptr<ParkingStand>> generalAviationStands;
        vector<shared_ptr<ParkingStand>> genericStands;

        for (const auto& stand : remainingStands)
        {
            if (isMilitaryStand(stand))
            {
                militaryStands.push_back(stand);
            }
            else if (isHelicopterStand(stand))
            {
                helicopterStands.push_back(stand);
            }
            else if (isGeneralAviationStand(stand))
            {
                generalAviationStands.push_back(stand);
            }
            else
            {
                genericStands.push_back(stand);
            }
        }

        const auto pullFallbackStands = [&](vector<shared_ptr<ParkingStand>>& target, size_t minimumCount) {
            while (target.size() < minimumCount && !genericStands.empty())
            {
                target.push_back(genericStands.back());
                genericStands.pop_back();
            }
        };

        pullFallbackStands(militaryStands, militaryStands.empty() ? 1 : 0);
        pullFallbackStands(helicopterStands, helicopterStands.empty() ? 1 : 0);
        pullFallbackStands(generalAviationStands, generalAviationStands.empty() ? 1 : 0);

        if (!genericStands.empty())
        {
            generalAviationStands.insert(generalAviationStands.end(), genericStands.begin(), genericStands.end());
            genericStands.clear();
        }

        const auto pickRandomSubset = [&](const vector<shared_ptr<ParkingStand>>& pool) {
            vector<shared_ptr<ParkingStand>> selected;
            if (pool.empty())
            {
                return selected;
            }

            vector<shared_ptr<ParkingStand>> remaining = pool;
            size_t requestedCount = static_cast<size_t>(remaining.size() * loadFactor);
            if (requestedCount == 0)
            {
                requestedCount = 1;
            }
            requestedCount = min(requestedCount, remaining.size());

            for (size_t i = 0 ; i < requestedCount && !remaining.empty() ; ++i)
            {
                const size_t index = pickRandomIndex(remaining.size());
                selected.push_back(remaining.at(index));
                remaining.erase(remaining.begin() + static_cast<long>(index));
            }

            return selected;
        };

        const auto pickRemoteArrivalStand = [this, &pickRandomIndex, &isHelicopterStand, &isGeneralAviationStand, &isMilitaryStand](
            const shared_ptr<Airport>& airport,
            TrafficBias trafficBias) -> shared_ptr<ParkingStand> {
            if (!airport)
            {
                return nullptr;
            }

            vector<shared_ptr<ParkingStand>> preferred;
            vector<shared_ptr<ParkingStand>> fallback;
            for (const auto& airportStand : airport->parkingStands())
            {
                if (!airportStand)
                {
                    continue;
                }

                fallback.push_back(airportStand);
                switch (trafficBias)
                {
                case TrafficBias::Helicopter:
                    if (isHelicopterStand(airportStand))
                    {
                        preferred.push_back(airportStand);
                    }
                    break;
                case TrafficBias::Military:
                    if (isMilitaryStand(airportStand))
                    {
                        preferred.push_back(airportStand);
                    }
                    break;
                case TrafficBias::GeneralAviation:
                default:
                    if (isGeneralAviationStand(airportStand))
                    {
                        preferred.push_back(airportStand);
                    }
                    break;
                }
            }

            const auto& candidates = preferred.empty() ? fallback : preferred;
            if (candidates.empty())
            {
                return nullptr;
            }

            return candidates.at(pickRandomIndex(candidates.size()));
        };

        militaryStands = pickRandomSubset(militaryStands);
        helicopterStands = pickRandomSubset(helicopterStands);
        generalAviationStands = pickRandomSubset(generalAviationStands);

        unordered_set<int> usedFlightIds;
        int nextRandomFlightId = 1;
        for (const auto& flight : m_world->flights())
        {
            if (!flight)
            {
                continue;
            }

            usedFlightIds.insert(flight->id());
            nextRandomFlightId = max(nextRandomFlightId, flight->id() + 1);
        }

        const auto allocateRandomFlightId = [&usedFlightIds, &nextRandomFlightId]() {
            while (usedFlightIds.find(nextRandomFlightId) != usedFlightIds.end())
            {
                ++nextRandomFlightId;
            }

            const int flightId = nextRandomFlightId;
            usedFlightIds.insert(flightId);
            ++nextRandomFlightId;
            return flightId;
        };

        const auto addOutboundFlight = [this, &makeCallSign, &calculateCruiseAltitudeFeet, &adjustCruiseAltitudeForMission, &pickRemoteArrivalStand](
            const RandomTrafficModelPools::RandomTrafficModel& model,
            const string& airline,
            int flightId,
            const string& destination,
            time_t departureTime,
            time_t arrivalTime,
            shared_ptr<ParkingStand> stand,
            Flight::RulesType rulesType,
            AIAircraft::MissionProfile missionProfile
        ) {
            string callSign = makeCallSign(model, flightId);
            const float minimumRunwayLengthMeters = AircraftPerformanceTable::minimumRunwayLengthMeters(model.category);
            const bool runwayFreeHelicopter = model.category == world::Aircraft::Category::Helicopter;

            if (destination.empty())
            {
                throw runtime_error("SCHEDL|no suitable random outbound destination airport available");
            }

            auto flightPlan = shared_ptr<FlightPlan>(new FlightPlan(departureTime, arrivalTime, m_airport->header().icao(), destination));
            flightPlan->setDepartureGate(stand->name());

            auto destinationAirport = m_host->getWorld()->getAirport(destination);
            if (!destinationAirport)
            {
                throw runtime_error("SCHEDL|no suitable random outbound destination airport object available");
            }

            if (runwayFreeHelicopter)
            {
                auto arrivalStand = pickRemoteArrivalStand(destinationAirport, TrafficBias::Helicopter);
                if (arrivalStand)
                {
                    flightPlan->setArrivalGate(arrivalStand->name());
                }
            }

            if (!runwayFreeHelicopter)
            {
                auto departureSelection = selectProcedureAndRunway(
                    m_airport,
                    m_airport->activeDepartureRunways(),
                    "SID",
                    destination,
                    false,
                    minimumRunwayLengthMeters);
                const string departureRunway = departureSelection.runwayName.empty()
                    ? bestSuitableRunwayName(m_airport, m_airport->activeDepartureRunways(), minimumRunwayLengthMeters)
                    : departureSelection.runwayName;
                if (departureRunway.empty())
                {
                    throw runtime_error("SCHEDL|no suitable random outbound departure runway available");
                }
                flightPlan->setDepartureRunway(departureRunway);
                if (rulesType == Flight::RulesType::IFR && !departureSelection.procedureName.empty())
                {
                    flightPlan->setSid(departureSelection.procedureName);
                }
            }

            if (!runwayFreeHelicopter || flightPlan->arrivalGate().empty())
            {
                auto arrivalSelection = selectProcedureAndRunway(
                    destinationAirport,
                    destinationAirport->activeArrivalRunways(),
                    "STAR",
                    m_airport->header().icao(),
                    true,
                    minimumRunwayLengthMeters);
                const string arrivalRunway = arrivalSelection.runwayName.empty()
                    ? bestSuitableRunwayName(destinationAirport, destinationAirport->activeArrivalRunways(), minimumRunwayLengthMeters)
                    : arrivalSelection.runwayName;
                if (arrivalRunway.empty())
                {
                    throw runtime_error("SCHEDL|no suitable random outbound arrival runway available");
                }
                flightPlan->setArrivalRunway(arrivalRunway);
                if (rulesType == Flight::RulesType::IFR && !arrivalSelection.procedureName.empty())
                {
                    flightPlan->setStar(arrivalSelection.procedureName);
                }
            }

            // Compute route heading for RVSM semi-circular altitude assignment
            const float outboundHeading = GeoMath::getHeadingFromPoints(
                m_airport->header().datum(), destinationAirport->header().datum());
            float cruiseAltitudeFeet = calculateCruiseAltitudeFeet(model.ceilingFl, rulesType, outboundHeading);
            cruiseAltitudeFeet = adjustCruiseAltitudeForMission(cruiseAltitudeFeet, m_airport, destinationAirport, model, missionProfile);
            flightPlan->setCruiseAltitudeFeet(cruiseAltitudeFeet);

            auto flight = shared_ptr<Flight>(new Flight(m_host, flightId, rulesType, airline, to_string(flightId), callSign, flightPlan));

            auto aircraft = m_host->createAIAircraft(model.icao, airline, to_string(flightId), model.category);
            flight->setAircraft(aircraft);
            if (auto aiAircraft = dynamic_pointer_cast<AIAircraft>(aircraft))
            {
                aiAircraft->setMissionProfile(missionProfile);
            }

            auto pilot = m_host->createAIPilot(flight);
            flight->setPilot(pilot);
            flight->setPhase(Flight::Phase::TurnAround);

            m_world->addFlightColdAndDark(flight);
            return flight;
        };

        const auto addInboundFlight = [this, &makeCallSign, &activeArrivalRunway1, &activeArrivalRunway2, &calculateCruiseAltitudeFeet, &adjustCruiseAltitudeForMission](
            const RandomTrafficModelPools::RandomTrafficModel& model,
            const string& airline,
            int flightId,
            const string& origin,
            time_t arrivalTime,
            time_t departureTime,
            shared_ptr<ParkingStand> stand,
            Flight::RulesType rulesType,
            int& arrivalIndex,
            AIAircraft::MissionProfile missionProfile
        ) {
            string callSign = makeCallSign(model, flightId);
            const float minimumRunwayLengthMeters = AircraftPerformanceTable::minimumRunwayLengthMeters(model.category);
            const bool runwayFreeHelicopter = model.category == world::Aircraft::Category::Helicopter;

            if (origin.empty())
            {
                throw runtime_error("SCHEDL|no suitable random inbound origin airport available");
            }

            (void)arrivalIndex;

            auto originAirport = m_host->getWorld()->getAirport(origin);
            if (!originAirport)
            {
                throw runtime_error("SCHEDL|no suitable random inbound origin airport object available");
            }

            auto flightPlan = shared_ptr<FlightPlan>(new FlightPlan(departureTime, arrivalTime, origin, m_airport->header().icao()));
            flightPlan->setArrivalGate(stand->name());

            string arrivalRunway;
            if (!runwayFreeHelicopter)
            {
                auto arrivalSelection = selectProcedureAndRunway(
                    m_airport,
                    m_airport->activeArrivalRunways(),
                    "STAR",
                    origin,
                    true,
                    minimumRunwayLengthMeters);
                arrivalRunway = arrivalSelection.runwayName.empty()
                    ? bestSuitableRunwayName(m_airport, { activeArrivalRunway1, activeArrivalRunway2 }, minimumRunwayLengthMeters)
                    : arrivalSelection.runwayName;
                if (arrivalRunway.empty())
                {
                    throw runtime_error("SCHEDL|no suitable random inbound arrival runway available");
                }
                flightPlan->setArrivalRunway(arrivalRunway);

                if (rulesType == Flight::RulesType::IFR && !arrivalSelection.procedureName.empty())
                {
                    flightPlan->setStar(arrivalSelection.procedureName);

                    GeoPoint originLocation = GeoPoint::empty;
                    GeoPoint airportLocation = m_airport->header().datum();
                    if (tryResolveLoadedAirportLocation(origin, originLocation) && airportLocation != GeoPoint::empty)
                    {
                        string transition = selectStarTransition(
                            m_airport->header().icao(),
                            arrivalSelection.procedureName,
                            arrivalRunway,
                            originLocation,
                            airportLocation);
                        if (!transition.empty())
                        {
                            flightPlan->setStarTransition(transition);
                        }
                    }
                }
            }

            if (!runwayFreeHelicopter)
            {
                auto departureSelection = selectProcedureAndRunway(
                    originAirport,
                    originAirport->activeDepartureRunways(),
                    "SID",
                    m_airport->header().icao(),
                    false,
                    minimumRunwayLengthMeters);
                const string departureRunway = departureSelection.runwayName.empty()
                    ? bestSuitableRunwayName(originAirport, originAirport->activeDepartureRunways(), minimumRunwayLengthMeters)
                    : departureSelection.runwayName;
                if (departureRunway.empty())
                {
                    throw runtime_error("SCHEDL|no suitable random inbound departure runway available");
                }
                flightPlan->setDepartureRunway(departureRunway);

                if (!departureSelection.procedureName.empty())
                {
                    flightPlan->setSid(departureSelection.procedureName);
                }
            }

            if (rulesType == Flight::RulesType::IFR && !arrivalRunway.empty())
            {
                string approach = selectApproachForRunway(m_airport->header().icao(), arrivalRunway);
                if (!approach.empty())
                {
                    flightPlan->setApproach(approach);
                }
            }

            // Compute route heading for RVSM semi-circular altitude assignment
            const float inboundHeading = GeoMath::getHeadingFromPoints(
                originAirport->header().datum(), m_airport->header().datum());
            float cruiseAltitudeFeet = calculateCruiseAltitudeFeet(model.ceilingFl, rulesType, inboundHeading);
            cruiseAltitudeFeet = adjustCruiseAltitudeForMission(cruiseAltitudeFeet, originAirport, m_airport, model, missionProfile);
            flightPlan->setCruiseAltitudeFeet(cruiseAltitudeFeet);

            auto flight = shared_ptr<Flight>(new Flight(m_host, flightId, rulesType, airline, to_string(flightId), callSign, flightPlan));

            auto aircraft = m_host->createAIAircraft(model.icao, airline, to_string(flightId), model.category);
            flight->setAircraft(aircraft);
            if (auto aiAircraft = dynamic_pointer_cast<AIAircraft>(aircraft))
            {
                aiAircraft->setMissionProfile(missionProfile);
            }

            auto pilot = m_host->createAIPilot(flight);
            flight->setPilot(pilot);
            flight->setPhase(Flight::Phase::Arrival);

            auto copyOfWorld = m_world;
            auto copyOfAirport = m_airport;
            const time_t arrivalStartTime = max(arrivalTime - 240, m_world->currentTime() + 30);
            m_world->deferUntil(
                "randomArrival/" + flight->callSign(),
                arrivalStartTime,
                [flight, copyOfWorld, copyOfAirport, arrivalRunway]() {
                    copyOfWorld->addFlight(flight);

                    if (arrivalRunway.empty())
                    {
                        auto landingStand = copyOfAirport->getParkingStandOrThrow(flight->plan()->arrivalGate());
                        auto aiAircraft = dynamic_pointer_cast<AIAircraft>(flight->aircraft());
                        auto aiPilot = dynamic_pointer_cast<AIPilot>(flight->pilot());
                        if (aiAircraft && aiPilot)
                        {
                            aiAircraft->setOnHelipadFinal(landingStand);
                            aiAircraft->setManeuver(aiPilot->getHelipadFinalToGate(landingStand));
                        }
                        else
                        {
                            flight->aircraft()->park(landingStand);
                        }
                        return;
                    }

                    const auto& landingRunwayEnd = copyOfWorld->getRunwayEnd(copyOfAirport->header().icao(), arrivalRunway);
                    flight->aircraft()->setOnFinal(landingRunwayEnd);
                }
            );
            return flight;
        };

        const auto configureFormationPair = [this, &missionProfileName](
            const shared_ptr<Flight>& leaderFlight,
            const shared_ptr<Flight>& wingmanFlight,
            AIAircraft::MissionProfile missionProfile) {
            auto leaderAircraft = leaderFlight ? dynamic_pointer_cast<AIAircraft>(leaderFlight->aircraft()) : nullptr;
            auto wingmanAircraft = wingmanFlight ? dynamic_pointer_cast<AIAircraft>(wingmanFlight->aircraft()) : nullptr;
            if (!leaderAircraft || !wingmanAircraft)
            {
                return;
            }

            leaderAircraft->setMissionProfile(missionProfile);
            leaderAircraft->setFormationRole(AIAircraft::FormationRole::Leader);

            const double lateralOffsetNm = (m_host->getNextRandom(100) < 50) ? -0.18 : 0.18;
            const double trailOffsetNm = missionProfile == AIAircraft::MissionProfile::Patrol ? 0.55 : 0.32;
            const double verticalOffsetFt = missionProfile == AIAircraft::MissionProfile::Patrol ? -80.0 : -40.0;
            wingmanAircraft->setMissionProfile(missionProfile);
            wingmanAircraft->setFormationLeader(leaderAircraft, trailOffsetNm, lateralOffsetNm, verticalOffsetFt);

            m_host->writeLog(
                "SCHEDL|Configured military formation pair leader[%s] wingman[%s] mission[%s]",
                leaderFlight->callSign().c_str(),
                wingmanFlight->callSign().c_str(),
                missionProfileName(missionProfile));
        };

        const auto addTrafficGroup = [this,
                                      &pickRandomIndex,
                                      &pickAirportIcaoForTraffic,
                                      &addOutboundFlight,
                                      &addInboundFlight,
                                      &allocateRandomFlightId,
                                      &chooseMilitaryMissionProfile,
                                      &configureFormationPair](
            const string& operatorIcao,
            const vector<RandomTrafficModelPools::RandomTrafficModel>& models,
            const vector<shared_ptr<ParkingStand>>& stands,
            TrafficBias trafficBias,
            bool preferNearbyAirport,
            time_t groupFirstDepartureTime,
            time_t groupFirstArrivalTime,
            int departureSpacingSeconds,
            int arrivalSpacingSeconds,
            float arrivalShare,
            Flight::RulesType rulesType
        ) {
            if (stands.empty() || models.empty())
            {
                return;
            }

            int arrivalIndex = 0;
            time_t nextDepartureTime = groupFirstDepartureTime;
            time_t nextArrivalTime = groupFirstArrivalTime;

            for (size_t index = 0 ; index < stands.size() ; )
            {
                const auto& stand = stands.at(index);
                const auto& model = models.at(pickRandomIndex(models.size()));
                const bool isTacticalFormationCandidate =
                    model.category == world::Aircraft::Category::Fighter ||
                    containsAnyToken(model.icao, { "C130", "C17", "C30J", "A400", "V22" });
                const int formationChance = model.category == world::Aircraft::Category::Fighter ? 42 : 18;
                const bool pairedMilitaryFormation =
                    trafficBias == TrafficBias::Military &&
                    isTacticalFormationCandidate &&
                    index + 1 < stands.size() &&
                    m_host->getNextRandom(100) < formationChance;
                const AIAircraft::MissionProfile missionProfile =
                    trafficBias == TrafficBias::Military ? chooseMilitaryMissionProfile(model) : AIAircraft::MissionProfile::None;

                try
                {
                    const bool createArrival = (index % 2 == 1) || (m_host->getNextRandom(100) / 100.0f < arrivalShare);
                    const bool missionPrefersNearby =
                        preferNearbyAirport ||
                        missionProfile == AIAircraft::MissionProfile::Training ||
                        missionProfile == AIAircraft::MissionProfile::LowLevel;

                    if (pairedMilitaryFormation)
                    {
                        const int leaderFlightId = allocateRandomFlightId();
                        const int wingmanFlightId = allocateRandomFlightId();
                        const auto& wingmanStand = stands.at(index + 1);

                        if (!createArrival)
                        {
                            const time_t leaderDepartureTime = nextDepartureTime;
                            nextDepartureTime += departureSpacingSeconds;
                            const time_t wingmanDepartureTime = leaderDepartureTime + max(20, min(60, departureSpacingSeconds / 3));
                            const string destination = pickAirportIcaoForTraffic(model, trafficBias, missionPrefersNearby, missionProfile);
                            const time_t leaderArrivalTime = leaderDepartureTime + departureSpacingSeconds * 24;
                            const time_t wingmanArrivalTime = wingmanDepartureTime + departureSpacingSeconds * 24;

                            auto leaderFlight = addOutboundFlight(model, operatorIcao, leaderFlightId, destination, leaderDepartureTime, leaderArrivalTime, stand, rulesType, missionProfile);
                            auto wingmanFlight = addOutboundFlight(model, operatorIcao, wingmanFlightId, destination, wingmanDepartureTime, wingmanArrivalTime, wingmanStand, rulesType, missionProfile);
                            configureFormationPair(leaderFlight, wingmanFlight, missionProfile);
                        }
                        else
                        {
                            const time_t leaderArrivalTime = nextArrivalTime;
                            nextArrivalTime += arrivalSpacingSeconds;
                            const time_t wingmanArrivalTime = leaderArrivalTime + max(20, min(60, arrivalSpacingSeconds / 3));
                            const string origin = pickAirportIcaoForTraffic(model, trafficBias, missionPrefersNearby, missionProfile);
                            const time_t leaderDepartureTime = leaderArrivalTime - arrivalSpacingSeconds * 12;
                            const time_t wingmanDepartureTime = wingmanArrivalTime - arrivalSpacingSeconds * 12;

                            auto leaderFlight = addInboundFlight(model, operatorIcao, leaderFlightId, origin, leaderArrivalTime, leaderDepartureTime, stand, rulesType, arrivalIndex, missionProfile);
                            auto wingmanFlight = addInboundFlight(model, operatorIcao, wingmanFlightId, origin, wingmanArrivalTime, wingmanDepartureTime, wingmanStand, rulesType, arrivalIndex, missionProfile);
                            configureFormationPair(leaderFlight, wingmanFlight, missionProfile);
                        }

                        index += 2;
                    }
                    else
                    {
                        const int flightId = allocateRandomFlightId();
                        if (!createArrival)
                        {
                            const time_t departureTime = nextDepartureTime;
                            nextDepartureTime += departureSpacingSeconds;
                            const string destination = pickAirportIcaoForTraffic(model, trafficBias, missionPrefersNearby, missionProfile);
                            const time_t arrivalTime = departureTime + departureSpacingSeconds * 24;
                            addOutboundFlight(model, operatorIcao, flightId, destination, departureTime, arrivalTime, stand, rulesType, missionProfile);
                        }
                        else
                        {
                            const time_t arrivalTime = nextArrivalTime;
                            nextArrivalTime += arrivalSpacingSeconds;
                            const string origin = pickAirportIcaoForTraffic(model, trafficBias, missionPrefersNearby, missionProfile);
                            const time_t departureTime = arrivalTime - arrivalSpacingSeconds * 12;
                            addInboundFlight(model, operatorIcao, flightId, origin, arrivalTime, departureTime, stand, rulesType, arrivalIndex, missionProfile);
                        }

                        ++index;
                    }
                }
                catch(const std::exception& e)
                {
                    m_host->writeLog("SCHEDL|Random traffic flight creation failed: %s", e.what());
                    ++index;
                }
            }
        };

        // --- Weather and time-of-day awareness for GA/helicopter traffic ---
        float gaWeatherFactor = 1.0f;
        float heliWeatherFactor = 1.0f;
        float timeOfDayFactor = 1.0f;
        {
            auto weatherService = m_host->services().get<WeatherService>();
            if (weatherService)
            {
                auto wx = weatherService->getWeatherAt(m_airport->header().datum(), 0.0f);
                if (wx.available)
                {
                    const float visMeters = wx.visibilityMeters;
                    const float windKt = wx.windSpeedMetersPerSecond * 1.94384f;
                    const float precipRate = wx.precipitationRate;

                    // VFR GA: suppress in IMC (vis < 5km), reduce in marginal VFR
                    if (visMeters > 0.0f && visMeters < 5000.0f)
                    {
                        gaWeatherFactor = 0.0f;
                        m_host->writeLog("SCHEDL|GA traffic suppressed: visibility %.0f m below VFR minimum", visMeters);
                    }
                    else if (visMeters > 0.0f && visMeters < 8000.0f)
                    {
                        gaWeatherFactor *= 0.35f;
                    }
                    if (windKt > 25.0f)
                    {
                        gaWeatherFactor *= max(0.1f, 1.0f - (windKt - 25.0f) / 20.0f);
                    }
                    if (precipRate > 0.3f)
                    {
                        gaWeatherFactor *= max(0.15f, 1.0f - precipRate * 0.6f);
                    }

                    // Helicopters: reduce in severe weather but less affected by visibility
                    if (visMeters > 0.0f && visMeters < 1500.0f)
                    {
                        heliWeatherFactor *= 0.15f;
                    }
                    else if (visMeters > 0.0f && visMeters < 3000.0f)
                    {
                        heliWeatherFactor *= 0.5f;
                    }
                    if (windKt > 35.0f)
                    {
                        heliWeatherFactor *= max(0.1f, 1.0f - (windKt - 35.0f) / 15.0f);
                    }
                    if (precipRate > 0.5f)
                    {
                        heliWeatherFactor *= max(0.2f, 1.0f - precipRate * 0.45f);
                    }

                    m_host->writeLog(
                        "SCHEDL|Weather factors: GA=%.2f Heli=%.2f (vis=%.0fm wind=%.0fkt precip=%.2f)",
                        gaWeatherFactor, heliWeatherFactor, visMeters, windKt, precipRate);
                }
            }

            // Time-of-day: GA/heli traffic peaks in daylight, drops at night
            const time_t now = m_world->currentTime();
            struct tm localTimeBuf;
            memset(&localTimeBuf, 0, sizeof(localTimeBuf));
#if IBM
            const struct tm* ltPtr = localtime(&now);
            if (ltPtr) { localTimeBuf = *ltPtr; }
#else
            localtime_r(&now, &localTimeBuf);
#endif
            const int hour = localTimeBuf.tm_hour;
            if (hour >= 7 && hour <= 19)
            {
                timeOfDayFactor = 1.0f;
            }
            else if (hour >= 5 && hour < 7)
            {
                timeOfDayFactor = 0.3f + 0.7f * (hour - 5) / 2.0f;
            }
            else if (hour > 19 && hour <= 21)
            {
                timeOfDayFactor = 0.3f + 0.7f * (21 - hour) / 2.0f;
            }
            else
            {
                timeOfDayFactor = 0.1f;
            }
            m_host->writeLog("SCHEDL|Time-of-day factor: %.2f (hour=%d)", timeOfDayFactor, hour);
        }

        const float effectiveGaLoad = loadFactor * gaWeatherFactor * timeOfDayFactor;
        const float effectiveHeliLoad = loadFactor * heliWeatherFactor * timeOfDayFactor;

        const int gaDepartureSpacingSeconds = max(90, static_cast<int>(210.0f * 0.7f / max(effectiveGaLoad, 0.05f)));
        const int gaArrivalSpacingSeconds = gaDepartureSpacingSeconds;
        const int helicopterDepartureSpacingSeconds = max(60, static_cast<int>(140.0f * 0.7f / max(effectiveHeliLoad, 0.05f)));
        const int helicopterArrivalSpacingSeconds = helicopterDepartureSpacingSeconds;
        const int militaryBaseDepartureSpacing = max(120, static_cast<int>(260.0f * 0.7f / max(loadFactor, 0.1f)));
        const int militaryArrivalSpacingSeconds = militaryBaseDepartureSpacing;

        addTrafficGroup("GA", randomTrafficModels.gaModels, generalAviationStands, TrafficBias::GeneralAviation, true, firstDepartureTime, firstArrivalTime, gaDepartureSpacingSeconds, gaArrivalSpacingSeconds, 0.45f, Flight::RulesType::VFR);
        addTrafficGroup("HLC", randomTrafficModels.helicopterModels, helicopterStands, TrafficBias::Helicopter, true, firstDepartureTime + 30, firstArrivalTime + 15, helicopterDepartureSpacingSeconds, helicopterArrivalSpacingSeconds, 0.55f, Flight::RulesType::VFR);
        addTrafficGroup("MIL", randomTrafficModels.militaryModels, militaryStands, TrafficBias::Military, false, firstDepartureTime + 60, firstArrivalTime + 30, militaryBaseDepartureSpacing, militaryArrivalSpacingSeconds, 0.35f, Flight::RulesType::IFR);

        m_host->writeLog(
            "SCHEDL|Loaded random offline traffic at airport[%s]",
            m_airport->header().icao().c_str());
    }

    bool loadLiveSchedules(float loadFactor)
    {
        // Collect from multiple sources (best-effort) with FR24 as primary.
        // AirNavRadar and PlaneFinder are opt-in sources enabled from the menu.
        vector<Fr24ScheduleEntry> fr24Deps, fr24Arrs;
        vector<Fr24ScheduleEntry> anrDeps, anrArrs;
        vector<Fr24ScheduleEntry> pfDeps, pfArrs;
        auto configuration = m_host->services().get<PluginConfiguration>();
        const bool useAirnav = configuration && configuration->enableAirnavSchedules;
        const bool usePlanefinder = configuration && configuration->enablePlanefinderSchedules;

        // Fetch schedule sources sequentially to avoid concurrent heavy fetches that
        // can trigger anti-bot blocks or overwhelm logging buffers.
        Fr24AirportScheduleSource fr24(m_host);
        bool fr24Ok = fr24.tryLoadAirportSchedules(m_airport->header().icao(), fr24Deps, fr24Arrs);

        bool airnavOk = false;
        bool planefinderOk = false;

        if (useAirnav)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            AirnavradarAirportScheduleSource airnav(m_host);
            airnavOk = airnav.tryLoadAirportSchedules(m_airport->header().icao(), anrDeps, anrArrs);
        }

        if (usePlanefinder)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            PlanefinderAirportScheduleSource planefinder(m_host);
            planefinderOk = planefinder.tryLoadAirportSchedules(m_airport->header().icao(), m_airport->header().iata(), pfDeps, pfArrs);
        }

        bool any = fr24Ok || airnavOk || planefinderOk;
        if (!any)
        {
            return false;
        }

        vector<Fr24ScheduleEntry> departures;
        vector<Fr24ScheduleEntry> arrivals;

        // Merge with cross-source deduplication (mode-sensitive).
        // Normalize flight numbers and treat entries with the same route and
        // nearby scheduled time as duplicates.
        auto normalizeCode = [&](string code) {
            code.erase(remove_if(code.begin(), code.end(), [](unsigned char c) {
                return isspace(c);
            }), code.end());
            transform(code.begin(), code.end(), code.begin(), [](unsigned char c) {
                return static_cast<char>(toupper(c));
            });
            return code;
        };

        auto normalizeFlightNumber = [&](string flight) {
            flight.erase(remove_if(flight.begin(), flight.end(), [](unsigned char c) {
                return isspace(c);
            }), flight.end());
            transform(flight.begin(), flight.end(), flight.begin(), [](unsigned char c) {
                return static_cast<char>(toupper(c));
            });
            return flight;
        };

        auto normalizeCallsign = [&](string callsign) {
            callsign.erase(remove_if(callsign.begin(), callsign.end(), [](unsigned char c) {
                return isspace(c);
            }), callsign.end());
            transform(callsign.begin(), callsign.end(), callsign.begin(), [](unsigned char c) {
                return static_cast<char>(toupper(c));
            });
            return callsign;
        };

        const long long timeToleranceSeconds = 300;
        unordered_map<string, vector<time_t>> seen;
        auto mergeEntries = [&](const vector<Fr24ScheduleEntry>& src, vector<Fr24ScheduleEntry>& dest, const char* modeLabel) {
            for (const auto& e : src)
            {
                if (e.flightNumber.empty())
                {
                    continue;
                }

                const string key = string(modeLabel) + "|" + normalizeFlightNumber(e.flightNumber) + "|" + normalizeCode(e.originIcao) + "|" + normalizeCode(e.destinationIcao);
                auto& times = seen[key];
                bool duplicate = false;
                for (time_t existing : times)
                {
                    if (existing != 0 && e.scheduledTime != 0 && llabs(static_cast<long long>(existing) - static_cast<long long>(e.scheduledTime)) <= timeToleranceSeconds)
                    {
                        duplicate = true;
                        break;
                    }
                }

                if (!duplicate)
                {
                    times.push_back(e.scheduledTime);
                    dest.push_back(e);
                }
            }
        };

        mergeEntries(fr24Deps, departures, "D");
        mergeEntries(anrDeps, departures, "D");
        mergeEntries(pfDeps, departures, "D");

        mergeEntries(fr24Arrs, arrivals, "A");
        mergeEntries(anrArrs, arrivals, "A");
        mergeEntries(pfArrs, arrivals, "A");

        vector<shared_ptr<ParkingStand>> gates;
        findGatesForFlights(gates, loadFactor);
        if (gates.empty())
        {
            m_host->writeLog("SCHEDL|Live schedules are available but no gates can be used at airport[%s]", m_airport->header().icao().c_str());
            return false;
        }

        const auto collectTraffic = [&](const vector<Fr24ScheduleEntry>& sourceEntries, bool isDeparture, vector<pair<Fr24ScheduleEntry, bool>>& traffic) {
            for (const auto& entry : sourceEntries)
            {
                traffic.push_back({ entry, isDeparture });
            }
        };

        vector<pair<Fr24ScheduleEntry, bool>> traffic;
        collectTraffic(departures, true, traffic);
        collectTraffic(arrivals, false, traffic);

        if (traffic.empty())
        {
            return false;
        }

        sort(traffic.begin(), traffic.end(), [](const pair<Fr24ScheduleEntry, bool>& a, const pair<Fr24ScheduleEntry, bool>& b) {
            if (a.first.scheduledTime != b.first.scheduledTime)
            {
                if (a.first.scheduledTime == 0) return false;
                if (b.first.scheduledTime == 0) return true;
                return a.first.scheduledTime < b.first.scheduledTime;
            }
            if (a.second != b.second)
            {
                return a.second;
            }
            return a.first.flightNumber < b.first.flightNumber;
        });

        const float normalSecondsBetweenDepartures = 210;
        const float normalSecondsBetweenArrivals = 210;
        const float normalLoadFactor = 0.7f;
        int secondsBetweenDepartures = normalSecondsBetweenDepartures * normalLoadFactor / loadFactor;
        int secondsBetweenArrivals = normalSecondsBetweenArrivals * normalLoadFactor / loadFactor;

        string activeDepartureRunway;
        string activeArrivalRunway1;
        string activeArrivalRunway2;
        {
            const auto& departure = m_airport->activeDepartureRunways();
            const auto& arrival = m_airport->activeArrivalRunways();

            activeDepartureRunway = !departure.empty() ? departure.at(0) : "";
            activeArrivalRunway1 = !arrival.empty() ? arrival.at(0) : "";
            activeArrivalRunway2 = !arrival.empty() ? arrival.at(arrival.size() - 1) : "";
        }

        const string localIcao = m_airport->header().icao();
        const float minimumRunwayLengthMeters = AircraftPerformanceTable::minimumRunwayLengthMeters(world::Aircraft::Category::Jet);
        const string fallbackIcao = pickOtherAirportIcao(localIcao, minimumRunwayLengthMeters);

        auto chooseRemoteAirport = [this, localIcao, fallbackIcao](const string& remoteIcao, const char* roleLabel) {

            const auto tryResolveLoaded = [this](const string& icao) -> shared_ptr<Airport> {
                const string normalizedIcao = normalizeIcao(icao);
                if (normalizedIcao.empty())
                {
                    return nullptr;
                }

                try
                {
                    return m_world->getAirport(normalizedIcao);
                }
                catch (const exception&)
                {
                    return nullptr;
                }
            };

            const string normalizedRemoteIcao = normalizeIcao(remoteIcao);
            if (!normalizedRemoteIcao.empty())
            {
                if (auto resolved = tryResolveLoaded(normalizedRemoteIcao))
                {
                    if (normalizeIcao(resolved->header().icao()) != localIcao)
                    {
                        return resolved;
                    }

                    m_host->writeLog(
                        "SCHEDL|Live %s airport [%s] resolves to local airport [%s] - using fallback [%s]",
                        roleLabel,
                        remoteIcao.c_str(),
                        localIcao.c_str(),
                        fallbackIcao.c_str());
                }
                else if (auto loadedAirport = tryLoadAirportIntoWorld(normalizedRemoteIcao))
                {
                    if (normalizeIcao(loadedAirport->header().icao()) != localIcao)
                    {
                        m_host->writeLog(
                            "SCHEDL|Live %s airport [%s] loaded on demand as [%s]",
                            roleLabel,
                            remoteIcao.c_str(),
                            loadedAirport->header().icao().c_str());
                        return loadedAirport;
                    }

                    m_host->writeLog(
                        "SCHEDL|Live %s airport [%s] loaded on demand but resolves to local airport [%s] - using fallback [%s]",
                        roleLabel,
                        remoteIcao.c_str(),
                        localIcao.c_str(),
                        fallbackIcao.c_str());
                }
                else
                {
                    m_host->writeLog(
                        "SCHEDL|Live %s airport [%s] not found - using fallback [%s]",
                        roleLabel,
                        remoteIcao.c_str(),
                        fallbackIcao.c_str());
                }
            }

            if (auto fallbackAirport = tryResolveLoaded(fallbackIcao))
            {
                return fallbackAirport;
            }

            return m_airport;
        };

        const auto retargetLiveFlightForDiversion = [this, &chooseRemoteAirport, &normalizeCode, &normalizeCallsign, &normalizeFlightNumber, &minimumRunwayLengthMeters](const Fr24ScheduleEntry& entry) -> bool {
            const string diversionIcao = normalizeIcao(
                !entry.divertedDestinationIcao.empty() ? entry.divertedDestinationIcao : entry.destinationIcao);
            if (diversionIcao.empty() || !entry.diverted)
            {
                return false;
            }

            const string entryFlightNo = normalizeFlightNumber(entry.flightNumber);
            const string entryCallsign = normalizeCallsign(entry.callsign);
            const string entryAirline = normalizeCode(entry.airlineIcao);

            for (const auto& existingFlight : m_world->flights())
            {
                if (!existingFlight || !existingFlight->plan())
                {
                    continue;
                }

                const string existingFlightNo = normalizeFlightNumber(existingFlight->flightNo());
                const string existingCallsign = normalizeCallsign(existingFlight->callSign());
                const string existingAirline = normalizeCode(existingFlight->airlineIcao());

                const bool callsignMatch = !entryCallsign.empty() && entryCallsign == existingCallsign;
                const bool flightNoAirlineMatch = !entryFlightNo.empty() && !entryAirline.empty() &&
                    entryFlightNo == existingFlightNo && entryAirline == existingAirline;
                if (!callsignMatch && !flightNoAirlineMatch)
                {
                    continue;
                }

                const string currentArrivalIcao = normalizeIcao(existingFlight->plan()->arrivalAirportIcao());
                if (currentArrivalIcao == diversionIcao)
                {
                    return true;
                }

                auto divertedAirport = chooseRemoteAirport(diversionIcao, "diversion");
                if (!divertedAirport)
                {
                    return false;
                }

                auto updatedPlan = shared_ptr<FlightPlan>(new FlightPlan(*existingFlight->plan()));
                updatedPlan->setArrivalAirportIcao(divertedAirport->header().icao());

                auto arrivalSelection = selectProcedureAndRunway(
                    divertedAirport,
                    divertedAirport->activeArrivalRunways(),
                    "STAR",
                    updatedPlan->departureAirportIcao(),
                    true,
                    minimumRunwayLengthMeters);

                const string divertedRunway = arrivalSelection.runwayName.empty()
                    ? bestSuitableRunwayName(divertedAirport, divertedAirport->activeArrivalRunways(), minimumRunwayLengthMeters)
                    : arrivalSelection.runwayName;
                if (divertedRunway.empty())
                {
                    return false;
                }
                updatedPlan->setArrivalRunway(divertedRunway);

                if (!arrivalSelection.procedureName.empty())
                {
                    updatedPlan->setStar(arrivalSelection.procedureName);
                }

                const string divertedApproach = selectApproachForRunway(divertedAirport->header().icao(), divertedRunway);
                if (!divertedApproach.empty())
                {
                    updatedPlan->setApproach(divertedApproach);
                }

                existingFlight->setPlan(updatedPlan);

                m_host->writeLog(
                    "SCHEDL|Live diversion applied flight[%s] from[%s] to[%s]",
                    existingFlight->callSign().c_str(),
                    currentArrivalIcao.c_str(),
                    normalizeIcao(updatedPlan->arrivalAirportIcao()).c_str());

                return true;
            }

            return false;
        };

        const auto resolveModel = [&normalizeCode](const Fr24ScheduleEntry& entry) {
            const string aircraftIcao = normalizeCode(entry.aircraftIcao);
            if (!aircraftIcao.empty())
            {
                AircraftTypeReferenceTable::Entry aircraft;
                if (AircraftTypeReferenceTable::tryFindByIcao(aircraftIcao, aircraft))
                {
                    return aircraft.icao;
                }
            }

            const string airlineIcao = normalizeCode(entry.airlineIcao);
            AirlineReferenceTable::Entry airline;
            const bool knownAirline = !airlineIcao.empty() && AirlineReferenceTable::tryFindByIcao(airlineIcao, airline);
            const string airlineName = knownAirline ? normalizeCode(airline.name) : "";

            // Prefer the most common narrowbody families for carriers where the
            // live feed often omits the exact aircraft type. Keep B738 as a
            // last resort so the visual fallback is still predictable.
            if (airlineIcao == "VY" || airlineName.find("VUELING") != string::npos)
            {
                return string("A320");
            }

            if (airlineIcao == "IB" || airlineIcao == "IBE" || airlineIcao == "I2" || airlineName.find("IBERIA") != string::npos)
            {
                return string("A320");
            }

            if (airlineIcao == "U2" || airlineName.find("EASYJET") != string::npos)
            {
                return string("A320");
            }

            if (airlineIcao == "W4" || airlineName.find("WIZZ") != string::npos)
            {
                return string("A20N");
            }

            if (airlineIcao == "FR" || airlineIcao == "RYR" || airlineName.find("RYANAIR") != string::npos)
            {
                return string("B738");
            }

            return string("B738");
        };

        auto addLiveDepartureFlight = [this, &chooseRemoteAirport, &activeDepartureRunway, &resolveModel, &fallbackIcao, &minimumRunwayLengthMeters] (
            const Fr24ScheduleEntry& entry,
            int flightId,
            shared_ptr<ParkingStand> gate,
            time_t departureTime)
        {
            reserveTrafficStand(gate);

            string model = resolveModel(entry);
            string airline = entry.airlineIcao;
            string callsign = entry.callsign.empty() ? (airline.empty() ? model : airline + " " + entry.flightNumber) : entry.callsign;
            string destination = !entry.destinationIcao.empty() ? entry.destinationIcao : fallbackIcao;
            auto destinationAirport = chooseRemoteAirport(destination, "destination");

            auto flightPlan = shared_ptr<FlightPlan>(new FlightPlan(departureTime, departureTime + 60 * 60 * 3, m_airport->header().icao(), destinationAirport->header().icao()));
            flightPlan->setDepartureGate(gate->name());
            flightPlan->setArrivalAirportIcao(destinationAirport->header().icao());

            auto departureSelection = selectProcedureAndRunway(
                m_airport,
                m_airport->activeDepartureRunways(),
                "SID",
                destinationAirport->header().icao(),
                false,
                minimumRunwayLengthMeters);
            const string departureRunway = departureSelection.runwayName.empty()
                ? bestSuitableRunwayName(m_airport, m_airport->activeDepartureRunways(), minimumRunwayLengthMeters)
                : departureSelection.runwayName;
            if (departureRunway.empty())
            {
                throw runtime_error("SCHEDL|no suitable live departure runway available");
            }
            flightPlan->setDepartureRunway(departureRunway);

            auto arrivalSelection = selectProcedureAndRunway(
                destinationAirport,
                destinationAirport->activeArrivalRunways(),
                "STAR",
                m_airport->header().icao(),
                true,
                minimumRunwayLengthMeters);
            const string arrivalRunway = arrivalSelection.runwayName.empty()
                ? bestSuitableRunwayName(destinationAirport, destinationAirport->activeArrivalRunways(), minimumRunwayLengthMeters)
                : arrivalSelection.runwayName;
            if (arrivalRunway.empty())
            {
                throw runtime_error("SCHEDL|no suitable live arrival runway available");
            }
            flightPlan->setArrivalRunway(arrivalRunway);

            if (!departureSelection.procedureName.empty())
            {
                flightPlan->setSid(departureSelection.procedureName);
            }

            if (!arrivalSelection.procedureName.empty())
            {
                flightPlan->setStar(arrivalSelection.procedureName);
            }

            flightPlan->setAirlineIcao(airline);
            flightPlan->setFlightNo(entry.flightNumber);
            flightPlan->setCallsign(callsign);

            auto flight = shared_ptr<Flight>(new Flight(m_host, flightId, Flight::RulesType::IFR, airline, entry.flightNumber, callsign, flightPlan));
            auto aircraft = m_host->createAIAircraft(model, airline, entry.flightNumber, world::Aircraft::Category::Jet);
            flight->setAircraft(aircraft);
            auto pilot = m_host->createAIPilot(flight);
            flight->setPilot(pilot);
            flight->setPhase(Flight::Phase::TurnAround);
            m_world->addFlightColdAndDark(flight);
        };

        auto addLiveArrivalFlight = [this, &chooseRemoteAirport, &activeArrivalRunway1, &activeArrivalRunway2, &resolveModel, &fallbackIcao, &minimumRunwayLengthMeters] (
            const Fr24ScheduleEntry& entry,
            int flightId,
            shared_ptr<ParkingStand> gate,
            time_t arrivalTime,
            int& arrivalIndex)
        {
            reserveTrafficStand(gate);

            string model = resolveModel(entry);
            string airline = entry.airlineIcao;
            string callsign = entry.callsign.empty() ? (airline.empty() ? model : airline + " " + entry.flightNumber) : entry.callsign;
            string origin = !entry.originIcao.empty() ? entry.originIcao : fallbackIcao;
            auto originAirport = chooseRemoteAirport(origin, "origin");

            auto arrivalSelection = selectProcedureAndRunway(
                m_airport,
                m_airport->activeArrivalRunways(),
                "STAR",
                originAirport->header().icao(),
                true,
                minimumRunwayLengthMeters);
            string arrivalRunway = arrivalSelection.runwayName.empty()
                ? bestSuitableRunwayName(m_airport, { activeArrivalRunway1, activeArrivalRunway2 }, minimumRunwayLengthMeters)
                : arrivalSelection.runwayName;
            if (arrivalRunway.empty())
            {
                throw runtime_error("SCHEDL|no suitable live arrival runway available");
            }

            auto flightPlan = shared_ptr<FlightPlan>(new FlightPlan(arrivalTime - 60 * 60 * 3, arrivalTime, originAirport->header().icao(), m_airport->header().icao()));
            flightPlan->setArrivalGate(gate->name());
            flightPlan->setArrivalRunway(arrivalRunway);
            flightPlan->setDepartureAirportIcao(originAirport->header().icao());

            if (!arrivalSelection.procedureName.empty())
            {
                flightPlan->setStar(arrivalSelection.procedureName);

                // Select STAR transition based on origin airport bearing
                GeoPoint originLocation = originAirport->header().datum();
                GeoPoint airportLocation = m_airport->header().datum();
                if (originLocation != GeoPoint::empty && airportLocation != GeoPoint::empty)
                {
                    string transition = selectStarTransition(
                        m_airport->header().icao(),
                        arrivalSelection.procedureName,
                        arrivalRunway,
                        originLocation,
                        airportLocation);
                    if (!transition.empty())
                    {
                        flightPlan->setStarTransition(transition);
                    }
                }
            }

            // Select approach for arrival runway
            string approach = selectApproachForRunway(m_airport->header().icao(), arrivalRunway);
            if (!approach.empty())
            {
                flightPlan->setApproach(approach);
            }

            auto departureSelection = selectProcedureAndRunway(
                originAirport,
                originAirport->activeDepartureRunways(),
                "SID",
                m_airport->header().icao(),
                false,
                minimumRunwayLengthMeters);
            const string departureRunway = departureSelection.runwayName.empty()
                ? bestSuitableRunwayName(originAirport, originAirport->activeDepartureRunways(), minimumRunwayLengthMeters)
                : departureSelection.runwayName;
            if (departureRunway.empty())
            {
                throw runtime_error("SCHEDL|no suitable live departure runway available");
            }
            flightPlan->setDepartureRunway(departureRunway);
            if (!departureSelection.procedureName.empty())
            {
                flightPlan->setSid(departureSelection.procedureName);
            }

            flightPlan->setAirlineIcao(airline);
            flightPlan->setFlightNo(entry.flightNumber);
            flightPlan->setCallsign(callsign);

            auto flight = shared_ptr<Flight>(new Flight(m_host, flightId, Flight::RulesType::IFR, airline, entry.flightNumber, callsign, flightPlan));
            auto aircraft = m_host->createAIAircraft(model, airline, entry.flightNumber, world::Aircraft::Category::Jet);
            flight->setAircraft(aircraft);
            auto pilot = m_host->createAIPilot(flight);
            flight->setPilot(pilot);
            flight->setPhase(Flight::Phase::Arrival);

            auto copyOfWorld = m_world;
            auto copyOfAirport = m_airport;
            const time_t arrivalStartTime = max(arrivalTime - estimateArrivalLeadSeconds(flight->plan()), m_world->currentTime() + 30);
            m_world->deferUntil(
                "fr24Arrival/" + flight->callSign(),
                arrivalStartTime,
                [flight, copyOfWorld, copyOfAirport, arrivalRunway]() {
                    const auto& landingRunwayEnd = copyOfWorld->getRunwayEnd(copyOfAirport->header().icao(), arrivalRunway);
                    copyOfWorld->addFlight(flight);
                    flight->aircraft()->setOnFinal(landingRunwayEnd);
                });
        };

        const auto resolveTrafficTime = [&](time_t& nextTime, const Fr24ScheduleEntry& entry, int spacingSeconds) -> time_t
        {
            if (entry.scheduledTime > 0)
            {
                const time_t resolvedTime = entry.scheduledTime;
                nextTime = max(nextTime + spacingSeconds, resolvedTime + spacingSeconds);
                return resolvedTime;
            }

            const time_t resolvedTime = nextTime;
            nextTime += spacingSeconds;
            return resolvedTime;
        };

        int diversionUpdates = 0;
        for (const auto& trafficItem : traffic)
        {
            if (retargetLiveFlightForDiversion(trafficItem.first))
            {
                diversionUpdates++;
            }
        }
        if (diversionUpdates > 0)
        {
            m_host->writeLog("SCHEDL|Applied [%d] live diversion update(s)", diversionUpdates);
        }

        size_t requestedCount = static_cast<size_t>(traffic.size() * loadFactor);
        if (requestedCount == 0)
        {
            requestedCount = 1;
        }
        requestedCount = min(requestedCount, gates.size());

        time_t nextDepartureTime = m_world->currentTime() + 200;
        time_t nextArrivalTime = m_world->currentTime() + 30;
        int arrivalIndex = 0;

        for (size_t index = 0 ; index < requestedCount ; ++index)
        {
            const auto& trafficItem = traffic.at(index);
            const auto& gate = gates.at(index % gates.size());
            int flightId = 1000 + static_cast<int>(index);

            try
            {
                if (trafficItem.second)
                {
                    const time_t departureTime = resolveTrafficTime(nextDepartureTime, trafficItem.first, secondsBetweenDepartures);
                    addLiveDepartureFlight(trafficItem.first, flightId, gate, departureTime);
                }
                else
                {
                    const time_t arrivalTime = resolveTrafficTime(nextArrivalTime, trafficItem.first, secondsBetweenArrivals);
                    addLiveArrivalFlight(trafficItem.first, flightId, gate, arrivalTime, arrivalIndex);
                }
            }
            catch(const exception& e)
            {
                m_host->writeLog("SCHEDL|FR24 live schedule flight creation failed: %s", e.what());
            }
        }

        m_host->writeLog(
            "SCHEDL|Loaded [%d] live FR24 AI flights at airport[%s]",
            m_world->flights().size(),
            m_airport->header().icao().c_str());

        return true;
    }

    void findGatesForFlights(vector<shared_ptr<ParkingStand>>& found, float loadFactor)
    {
        GeoPoint userAircraftLocation((float)m_userAircraftLatitude, (float)m_userAircraftLongitude);

        const int nameCheckBufferSize = 32;
        char nameCheckBuffer[nameCheckBufferSize + 1] = { 0 };

        const auto isUserAircraftParkedAtGate = [&](const shared_ptr<ParkingStand>& gate)->bool {
            auto distanceToUserAircraft = GeoMath::getDistanceMeters(userAircraftLocation, gate->location().geo());
            return (distanceToUserAircraft < 50);
        };

        const auto isPassengerGateName = [&](const string& name)->bool {
            strncpy(nameCheckBuffer, name.c_str(), nameCheckBufferSize);
            for (int i = 0 ; i < name.length() && i < nameCheckBufferSize ; i++)
            {
                nameCheckBuffer[i] = toupper(nameCheckBuffer[i]);
            }
            return (
                !strstr(nameCheckBuffer, "HEL") &&
                !strstr(nameCheckBuffer, "MILI") &&
                !strstr(nameCheckBuffer, "RAMP") &&
                (!strstr(nameCheckBuffer, "GA") || strstr(nameCheckBuffer, "GATE")) &&
                !strstr(nameCheckBuffer, "G.A") &&
                !strstr(nameCheckBuffer, "GENERAL") &&
                !strstr(nameCheckBuffer, "GRASS") &&
                !strstr(nameCheckBuffer, "DIRT") &&
                !strstr(nameCheckBuffer, "FUEL") &&
                !strstr(nameCheckBuffer, "CARGO") &&
                !strstr(nameCheckBuffer, "HANG") &&
                !strstr(nameCheckBuffer, "TIE") &&
                !strstr(nameCheckBuffer, "MAINT") &&
                !strstr(nameCheckBuffer, "DOCK"));
        };

        const auto canUseGateForAIFlights = [&](const shared_ptr<ParkingStand>& gate)->bool {
            if (isUserAircraftParkedAtGate(gate))
            {
                m_host->writeLog(
                    "SCHEDL|Skipping gate[%s] looks like the user aircraft is parked here!",
                    gate->name().c_str());
                return false;
            }

            bool canUse = (
                gate->type() == ParkingStand::Type::Gate &&
                gate->hasOperationType(world::Aircraft::OperationType::Airline) &&
                !gate->hasOperationType(world::Aircraft::OperationType::Cargo) &&
                (gate->name().length() < 10 || isPassengerGateName(gate->name())));

            return canUse;
        };

        const vector<shared_ptr<ParkingStand>>& allGates = m_airport->parkingStands();
        vector<shared_ptr<ParkingStand>> usableGates;
        copy_if(allGates.begin(), allGates.end(), back_inserter(usableGates), canUseGateForAIFlights);
        m_host->writeLog(
            "SCHEDL|Found [%d/%d] gates for AI flights, skipped [%d]",
            usableGates.size(), allGates.size(), allGates.size() - usableGates.size());

        vector<unsigned int> indices(usableGates.size());
        iota(indices.begin(), indices.end(), 0);
        shuffle(indices.begin(), indices.end(), std::default_random_engine());
        int requestedCount = (int)(usableGates.size() * loadFactor);

        for (int i = 0 ; i < indices.size() && i < requestedCount ; i++)
        {
            const shared_ptr<ParkingStand>& gate = usableGates.at(indices.at(i));
            found.push_back(gate);
        }

        m_host->writeLog(
            "SCHEDL|Picked [%d/%d] gates for AI flights at load factor[%f]",
            found.size(),
            requestedCount,
            loadFactor);
    }

    void logActiveRunwaysBounds()
    {
        const auto logBounds = [this](shared_ptr<Runway> runway) {
            const auto& bounds = runway->bounds();
            m_host->writeLog(
                "LSCHED|RWY-BOUNDS[%s]: A[%f,%f] B[%f,%f] C[%f,%f] D[%f,%f] minLat[%f] maxLat[%f] minLon[%f] maxLon[%f]",
                runway->name().c_str(),
                bounds.A.latitude, bounds.A.longitude,
                bounds.B.latitude, bounds.B.longitude,
                bounds.C.latitude, bounds.C.longitude,
                bounds.D.latitude, bounds.D.longitude,
                bounds.minLatitude, bounds.maxLatitude,
                bounds.minLongitude, bounds.maxLongitude);
        };

        for (const auto& name : m_airport->activeArrivalRunways())
        {
            logBounds(m_airport->getRunwayOrThrow(name));
        }

        for (const auto& name : m_airport->activeDepartureRunways())
        {
            logBounds(m_airport->getRunwayOrThrow(name));
        }
    }
};
