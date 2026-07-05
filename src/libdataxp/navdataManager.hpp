// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../libworld/libworld.h"
#include "xpCifpReader.hpp"
#include "xpNavDataReader.hpp"
#include "xpAltitudeReader.hpp"
#include "holdingPattern.hpp"
#include "holdingEntry.hpp"
#include "missedApproach.hpp"
#include "procedureValidator.hpp"

using namespace std;
using namespace world;

class XPNavdataManager
{
private:
    // Cache for parsed procedures
    struct ProcedureCacheEntry
    {
        XPCifpReader::ProcedureTrackWithLocations procedureTrack;
        XPCifpReader::ProcedureTrackWithLocations missedTrack;
        chrono::steady_clock::time_point timestamp;
    };

    // Cache for holding patterns
    struct HoldingCacheEntry
    {
        vector<XPHoldingPatternReader::HoldingPattern> patterns;
        chrono::steady_clock::time_point timestamp;
    };

    shared_ptr<HostServices> m_host;

    // Component readers
    unique_ptr<XPCifpReader> m_cifpReader;
    unique_ptr<XPNavaidReader> m_navaidReader;
    unique_ptr<XPMinimumAltitudeReader> m_altitudeReader;
    unique_ptr<XPHoldingPatternReader> m_holdingReader;
    unique_ptr<XPHoldingEntryCalculator> m_entryCalculator;
    unique_ptr<XPMissedApproachReader> m_missedApproachReader;
    unique_ptr<XPProcedureValidator> m_validator;

    // Caches
    mutable mutex m_cacheMutex;
    mutable unordered_map<string, ProcedureCacheEntry> m_procedureCache;
    mutable unordered_map<string, HoldingCacheEntry> m_holdingCache;
    static constexpr chrono::seconds CACHE_TTL = chrono::seconds(300);

    static string normalizeKey(const string& text)
    {
        string result;
        for (char c : text)
        {
            if (isalnum(static_cast<unsigned char>(c)))
            {
                result.push_back(static_cast<char>(toupper(static_cast<unsigned char>(c))));
            }
        }
        return result;
    }

    static string trimCopy(const string& text)
    {
        const auto begin = text.find_first_not_of(" \t\r\n");
        if (begin == string::npos)
        {
            return "";
        }

        const auto end = text.find_last_not_of(" \t\r\n");
        return text.substr(begin, end - begin + 1);
    }

    static string uppercaseCopy(string text)
    {
        transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
            return static_cast<char>(toupper(c));
        });
        return text;
    }

    static string normalizeToken(const string& text)
    {
        string result;
        for (char c : trimCopy(text))
        {
            if (isalnum(static_cast<unsigned char>(c)))
            {
                result.push_back(static_cast<char>(toupper(static_cast<unsigned char>(c))));
            }
        }
        return result;
    }

    static string normalizeRunwayToken(const string& text)
    {
        string result = normalizeToken(text);
        if (result.rfind("RW", 0) == 0)
        {
            result.erase(0, 2);
        }
        return result;
    }

    static bool matchesToken(const string& lhs, const string& rhs)
    {
        return normalizeToken(lhs) == normalizeToken(rhs);
    }

    static bool matchesRunwayToken(const string& lhs, const string& rhs)
    {
        return normalizeRunwayToken(lhs) == normalizeRunwayToken(rhs);
    }

    static string extractRunwayToken(const string& text)
    {
        const string normalized = uppercaseCopy(trimCopy(text));
        string runway;
        bool sawDigit = false;

        for (char c : normalized)
        {
            if (isdigit(static_cast<unsigned char>(c)))
            {
                runway.push_back(c);
                sawDigit = true;
            }
            else if (sawDigit && (c == 'L' || c == 'R' || c == 'C' || c == 'B' || c == 'M' || c == 'S'))
            {
                runway.push_back(c);
                break;
            }
            else if (sawDigit)
            {
                break;
            }
        }

        return runway;
    }

    bool isCacheValid(const chrono::steady_clock::time_point& timestamp) const
    {
        return chrono::steady_clock::now() - timestamp < CACHE_TTL;
    }

public:
    explicit XPNavdataManager(shared_ptr<HostServices> host) :
        m_host(std::move(host)),
        m_cifpReader(nullptr),
        m_navaidReader(nullptr),
        m_altitudeReader(nullptr),
        m_holdingReader(nullptr),
        m_entryCalculator(nullptr),
        m_missedApproachReader(nullptr),
        m_validator(nullptr),
        m_cacheMutex(),
        m_procedureCache(),
        m_holdingCache()
    {
        if (m_host)
        {
            m_cifpReader = make_unique<XPCifpReader>(m_host);
            m_navaidReader = make_unique<XPNavaidReader>(m_host);
            m_altitudeReader = make_unique<XPMinimumAltitudeReader>(m_host);
            m_holdingReader = make_unique<XPHoldingPatternReader>(m_host);
            m_entryCalculator = make_unique<XPHoldingEntryCalculator>();
            m_missedApproachReader = make_unique<XPMissedApproachReader>(m_host);
            m_validator = make_unique<XPProcedureValidator>(m_host);
        }
    }

    // ==================== Procedure Reading ====================

    struct ProcedureResult
    {
        vector<string> procedureWaypoints;
        vector<string> missedWaypoints;
        XPCifpReader::ProcedureTrackWithLocations procedureTrack;
        XPCifpReader::ProcedureTrackWithLocations missedTrack;
        bool hasMissedApproach = false;
    };

    // Read a complete approach procedure with missed approach
    ProcedureResult readApproachProcedure(
        const string& airportIcao,
        const string& procedureName,
        const string& preferredRunway = "",
        const string& preferredTransition = "") const
    {
        ProcedureResult result;

        if (!m_cifpReader || !m_host)
        {
            return result;
        }

        // Check cache first
        const string cacheKey = normalizeKey(airportIcao) + "|" + normalizeKey(procedureName) +
            "|" + normalizeKey(preferredRunway) + "|" + normalizeKey(preferredTransition);

        {
            lock_guard<mutex> lock(m_cacheMutex);
            const auto it = m_procedureCache.find(cacheKey);
            if (it != m_procedureCache.end() && isCacheValid(it->second.timestamp))
            {
                result.procedureTrack = it->second.procedureTrack;
                result.missedTrack = it->second.missedTrack;
                result.hasMissedApproach = !result.missedTrack.waypoints.empty();

                // Collapse waypoints
                for (const auto& wp : result.procedureTrack.waypoints)
                {
                    result.procedureWaypoints.push_back(wp.name);
                }
                for (const auto& wp : result.missedTrack.waypoints)
                {
                    result.missedWaypoints.push_back(wp.name);
                }

                return result;
            }
        }

        try
        {
            auto input = m_host->openFileForRead(
                m_host->getHostFilePath({ "Custom Data", "CIFP", airportIcao + ".dat" }));
            if (!input)
            {
                input = m_host->openFileForRead(
                    m_host->getHostFilePath({ "Resources", "default data", "CIFP", airportIcao + ".dat" }));
            }
            if (!input)
            {
                return result;
            }

            const auto selection = m_cifpReader->readApproachProcedureTracksWithLocations(
                *input, procedureName, preferredRunway, preferredTransition);

            result.procedureTrack = selection.procedureTrack;
            result.missedTrack = selection.missedTrack;
            result.hasMissedApproach = !result.missedTrack.waypoints.empty();

            // Collapse waypoints
            for (const auto& wp : result.procedureTrack.waypoints)
            {
                result.procedureWaypoints.push_back(wp.name);
            }
            for (const auto& wp : result.missedTrack.waypoints)
            {
                result.missedWaypoints.push_back(wp.name);
            }

            // Update cache
            lock_guard<mutex> lock(m_cacheMutex);
            ProcedureCacheEntry entry;
            entry.procedureTrack = result.procedureTrack;
            entry.missedTrack = result.missedTrack;
            entry.timestamp = chrono::steady_clock::now();
            m_procedureCache[cacheKey] = std::move(entry);
        }
        catch (const exception&)
        {
            return result;
        }

        return result;
    }

    // Read SID procedure
    vector<string> readSid(
        const string& airportIcao,
        const string& procedureName,
        const string& preferredRunway = "",
        const string& preferredTransition = "") const
    {
        if (!m_cifpReader || !m_host)
        {
            return {};
        }

        try
        {
            auto input = m_host->openFileForRead(
                m_host->getHostFilePath({ "Custom Data", "CIFP", airportIcao + ".dat" }));
            if (!input)
            {
                input = m_host->openFileForRead(
                    m_host->getHostFilePath({ "Resources", "default data", "CIFP", airportIcao + ".dat" }));
            }
            if (!input)
            {
                return {};
            }

            return m_cifpReader->readProcedureTrack(*input, "SID", procedureName, preferredRunway, preferredTransition);
        }
        catch (const exception&)
        {
            return {};
        }
    }

    // Read STAR procedure
    vector<string> readStar(
        const string& airportIcao,
        const string& procedureName,
        const string& preferredRunway = "",
        const string& preferredTransition = "") const
    {
        if (!m_cifpReader || !m_host)
        {
            return {};
        }

        try
        {
            auto input = m_host->openFileForRead(
                m_host->getHostFilePath({ "Custom Data", "CIFP", airportIcao + ".dat" }));
            if (!input)
            {
                input = m_host->openFileForRead(
                    m_host->getHostFilePath({ "Resources", "default data", "CIFP", airportIcao + ".dat" }));
            }
            if (!input)
            {
                return {};
            }

            return m_cifpReader->readProcedureTrack(*input, "STAR", procedureName, preferredRunway, preferredTransition);
        }
        catch (const exception&)
        {
            return {};
        }
    }

    // Enumerate available procedures
    vector<string> enumProcedures(const string& airportIcao, const string& recordType) const
    {
        if (!m_cifpReader || !m_host)
        {
            return {};
        }

        try
        {
            auto input = m_host->openFileForRead(
                m_host->getHostFilePath({ "Custom Data", "CIFP", airportIcao + ".dat" }));
            if (!input)
            {
                input = m_host->openFileForRead(
                    m_host->getHostFilePath({ "Resources", "default data", "CIFP", airportIcao + ".dat" }));
            }
            if (!input)
            {
                return {};
            }

            return m_cifpReader->enumProcedures(*input, recordType);
        }
        catch (const exception&)
        {
            return {};
        }
    }

    // ==================== Holding Pattern Support ====================

    struct HoldingPatternInfo
    {
        XPHoldingPatternReader::HoldingPattern pattern;
        XPHoldingEntryCalculator::EntryProcedure entryProcedure;
        bool hasPattern = false;
        bool hasEntryProcedure = false;
    };

    // Get holding pattern information for a fix
    HoldingPatternInfo getHoldingPatternInfo(
        const string& fixIdent,
        const string& region = "",
        float inboundCourse = 0.0f,
        float currentTrack = 0.0f) const
    {
        HoldingPatternInfo info;

        if (!m_holdingReader || !m_entryCalculator)
        {
            return info;
        }

        // Query holding pattern
        XPHoldingPatternReader::HoldingPattern pattern;
        const bool found = m_holdingReader->queryHoldingPattern(fixIdent, region, pattern);
        if (!found)
        {
            return info;
        }

        info.pattern = pattern;
        info.hasPattern = true;

        // Calculate entry procedure if we have course information
        if (inboundCourse > 0.0f && currentTrack > 0.0f)
        {
            info.entryProcedure = m_entryCalculator->calculateEntryProcedure(
                inboundCourse, currentTrack, info.pattern.isRightTurn);
            info.hasEntryProcedure = true;
        }

        return info;
    }

    // Check if a fix has a holding pattern
    bool hasHoldingPattern(const string& fixIdent) const
    {
        if (!m_holdingReader)
        {
            return false;
        }
        return m_holdingReader->hasHoldingPattern(fixIdent);
    }

    // Get all holding patterns for a fix
    vector<XPHoldingPatternReader::HoldingPattern> getHoldingPatterns(const string& fixIdent) const
    {
        if (!m_holdingReader)
        {
            return {};
        }
        return m_holdingReader->queryHoldingPatterns(fixIdent);
    }

    // ==================== Missed Approach Support ====================

    // Read missed approach procedure
    XPMissedApproachReader::MissedApproachTrack readMissedApproach(
        const string& airportIcao,
        const string& procedureName,
        const string& preferredRunway = "",
        const string& preferredTransition = "") const
    {
        if (!m_missedApproachReader)
        {
            return {};
        }
        return m_missedApproachReader->readMissedApproachTrack(
            airportIcao, procedureName, preferredRunway, preferredTransition);
    }

    // Check if procedure has missed approach
    bool hasMissedApproach(const string& airportIcao, const string& procedureName) const
    {
        if (!m_missedApproachReader)
        {
            return false;
        }
        return m_missedApproachReader->hasMissedApproach(airportIcao, procedureName);
    }

    // Get missed approach holding fix
    // Returns nullptr if no holding fix found, otherwise returns pointer to string
    const string* getMissedApproachHoldingFix(
        const string& airportIcao,
        const string& procedureName) const
    {
        if (!m_missedApproachReader)
        {
            return nullptr;
        }
        return m_missedApproachReader->getMissedApproachHoldingFix(airportIcao, procedureName);
    }

    // ==================== Validation ====================

    // Validate a complete procedure
    XPProcedureValidator::ValidationResult validateProcedure(
        const vector<XPCifpReader::WaypointWithLocation>& waypoints,
        const string& procedureType,
        float aircraftSpeedKnots = 250.0f,
        float aircraftAltitudeFeet = 0.0f) const
    {
        if (!m_validator)
        {
            return {};
        }
        return m_validator->validateProcedureTrack(
            waypoints, procedureType, aircraftSpeedKnots, aircraftAltitudeFeet);
    }

    // Validate altitude constraints
    XPProcedureValidator::ValidationResult validateAltitudeConstraints(
        const vector<XPCifpReader::WaypointWithLocation>& waypoints,
        float aircraftAltitudeFeet,
        float aircraftRateOfClimbFpm = 1000.0f) const
    {
        if (!m_validator)
        {
            return {};
        }
        return m_validator->validateAltitudeConstraints(
            waypoints, aircraftAltitudeFeet, aircraftRateOfClimbFpm);
    }

    // Validate speed constraints
    XPProcedureValidator::ValidationResult validateSpeedConstraints(
        const vector<XPCifpReader::WaypointWithLocation>& waypoints,
        float aircraftSpeedKnots) const
    {
        if (!m_validator)
        {
            return {};
        }
        return m_validator->validateSpeedConstraints(waypoints, aircraftSpeedKnots);
    }

    // Validate RNAV/RNP procedures
    XPProcedureValidator::ValidationResult validateRnavRnp(
        const vector<XPCifpReader::WaypointWithLocation>& waypoints,
        const string& procedureType) const
    {
        if (!m_validator)
        {
            return {};
        }
        return m_validator->validateRnavRnp(waypoints, procedureType);
    }

    // Validate aircraft compatibility
    XPProcedureValidator::ValidationResult validateAircraftCompatibility(
        const vector<XPCifpReader::WaypointWithLocation>& waypoints,
        float aircraftMaxSpeedKnots,
        float aircraftMaxAltitudeFeet,
        float aircraftMaxClimbRateFpm) const
    {
        if (!m_validator)
        {
            return {};
        }
        return m_validator->validateAircraftCompatibility(
            waypoints, aircraftMaxSpeedKnots, aircraftMaxAltitudeFeet, aircraftMaxClimbRateFpm);
    }

    // ==================== Waypoint Resolution ====================

    // Resolve waypoint location
    bool resolveWaypoint(const string& waypointName, GeoPoint& location) const
    {
        if (!m_navaidReader)
        {
            return false;
        }
        return m_navaidReader->tryResolveWaypoint(waypointName, location);
    }

    // Resolve waypoint with context
    bool resolveWaypoint(
        const string& contextIcao,
        const string& waypointName,
        GeoPoint& location) const
    {
        if (!m_navaidReader)
        {
            return false;
        }
        return m_navaidReader->tryResolveWaypoint(contextIcao, waypointName, location);
    }

    // ==================== Altitude Support ====================

    // Get minimum altitude for a leg
    float getMinimumAltitudeForLeg(
        const string& contextIcao,
        const string& fromName,
        const GeoPoint& fromLocation,
        const string& toName,
        const GeoPoint& toLocation,
        float baseAltitudeFeet) const
    {
        if (!m_altitudeReader)
        {
            return baseAltitudeFeet;
        }
        return m_altitudeReader->minimumAltitudeForLeg(
            contextIcao, fromName, fromLocation, toName, toLocation, baseAltitudeFeet);
    }

    // Get MORA floor at a point
    bool getMoraFloorAt(const GeoPoint& point, float& altitudeFeet) const
    {
        if (!m_altitudeReader)
        {
            return false;
        }
        return m_altitudeReader->tryGetMoraFloorAt(point, altitudeFeet);
    }

    // Get MSA floor at a fix
    bool getMsaFloor(const string& airportIcao, const string& fixIdent, float& altitudeFeet) const
    {
        if (!m_altitudeReader)
        {
            return false;
        }
        return m_altitudeReader->tryGetMsaFloor(airportIcao, fixIdent, altitudeFeet);
    }

    // ==================== Cache Management ====================

    // Clear all caches
    void clearCache()
    {
        lock_guard<mutex> lock(m_cacheMutex);
        m_procedureCache.clear();
        m_holdingCache.clear();
    }

    // Clear procedure cache
    void clearProcedureCache()
    {
        lock_guard<mutex> lock(m_cacheMutex);
        m_procedureCache.clear();
    }

    // Clear holding pattern cache
    void clearHoldingCache()
    {
        lock_guard<mutex> lock(m_cacheMutex);
        m_holdingCache.clear();
    }

    // Get cache statistics
    struct CacheStatistics
    {
        size_t procedureCacheSize = 0;
        size_t holdingCacheSize = 0;
    };

    CacheStatistics getCacheStatistics() const
    {
        lock_guard<mutex> lock(m_cacheMutex);
        CacheStatistics stats;
        stats.procedureCacheSize = m_procedureCache.size();
        stats.holdingCacheSize = m_holdingCache.size();
        return stats;
    }
};
