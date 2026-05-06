// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#pragma once

#include <algorithm>
#include <cctype>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../libworld/libworld.h"

using namespace std;
using namespace world;

class XPCifpReader
{
private:
    struct ProcedureRecord
    {
        int sequence = -1;
        string procedureName;
        string branchKey;
        string waypoint;
        string waypointDescription;
        string pathTerminator;
        float latitude = 0.0f;
        float longitude = 0.0f;
        bool hasLocation = false;
        // Altitude/speed constraints from the CIFP row layout
        float altitudeConstraint = 0.0f;  // Feet or FL*100
        char altitudeConstraintType = 0;  // '+', '-', ' ' (at/above, at/below, at)
        float speedConstraint = 0.0f;       // Knots
        char speedConstraintType = 0;       // '+', '-', ' ' (min, max, exact)
    };

    struct RawTrack
    {
        string procedureName;
        string branchKey;
        vector<ProcedureRecord> records;
    };

    // Static cache for procedure lists to avoid repeated file I/O
    struct ProcedureListCacheEntry
    {
        vector<string> procedures;
        chrono::steady_clock::time_point timestamp;
    };
    inline static mutex s_procedureCacheMutex;
    inline static unordered_map<string, ProcedureListCacheEntry> s_procedureCache;
    inline static const chrono::seconds s_cacheTTL = chrono::seconds(300); // 5 minute TTL

    shared_ptr<HostServices> m_host;

public:
    struct WaypointWithLocation
    {
        string name;
        float latitude = 0.0f;
        float longitude = 0.0f;
        bool hasLocation = false;
        // Altitude/speed constraints from the CIFP row layout
        float altitudeConstraint = 0.0f;
        char altitudeConstraintType = 0;
        float speedConstraint = 0.0f;
        char speedConstraintType = 0;

        WaypointWithLocation() = default;
        WaypointWithLocation(const string& n, float lat, float lon, bool hasLoc)
            : name(n), latitude(lat), longitude(lon), hasLocation(hasLoc) {}
        WaypointWithLocation(const string& n, float lat, float lon, bool hasLoc,
                             float altConstr, char altType, float spdConstr, char spdType)
            : name(n), latitude(lat), longitude(lon), hasLocation(hasLoc),
              altitudeConstraint(altConstr), altitudeConstraintType(altType),
              speedConstraint(spdConstr), speedConstraintType(spdType) {}
    };
    
    struct ProcedureTrackSelection
    {
        vector<string> procedureTrack;
        vector<string> missedTrack;
    };
    
    struct ProcedureTrackWithLocations
    {
        vector<WaypointWithLocation> waypoints;
    };

    explicit XPCifpReader(shared_ptr<HostServices> host) :
        m_host(std::move(host))
    {
    }

public:
    vector<string> readProcedureTrack(
        const string& airportIcao,
        const string& recordType,
        const string& procedureName,
        const string& preferredRunway = "",
        const string& preferredTransition = "") const
    {
        if (!m_host)
        {
            return {};
        }

        try
        {
            auto filePath = m_host->getHostFilePath({ "Custom Data", "CIFP", airportIcao + ".dat" });
            auto input = m_host->openFileForRead(filePath);
            if (!input)
            {
                return {};
            }
            return readProcedureTrack(*input, recordType, procedureName, preferredRunway, preferredTransition);
        }
        catch (const exception&)
        {
            return {};
        }
    }

    ProcedureTrackWithLocations readProcedureTrackWithLocations(
        const string& airportIcao,
        const string& recordType,
        const string& procedureName,
        const string& preferredRunway = "",
        const string& preferredTransition = "") const
    {
        if (!m_host)
        {
            return {};
        }

        try
        {
            auto filePath = m_host->getHostFilePath({ "Custom Data", "CIFP", airportIcao + ".dat" });
            auto input = m_host->openFileForRead(filePath);
            if (!input)
            {
                return {};
            }
            return readProcedureTrackWithLocations(*input, recordType, procedureName, preferredRunway, preferredTransition);
        }
        catch (const exception&)
        {
            return {};
        }
    }

    vector<string> readProcedureTrack(
        istream& input,
        const string& recordType,
        const string& procedureName,
        const string& preferredRunway = "",
        const string& preferredTransition = "") const
    {
        auto tracks = parseProcedureTracks(input, recordType, procedureName);
        const RawTrack* selectedTrack = selectProcedureTrack(tracks, procedureName, preferredRunway, preferredTransition);
        return selectedTrack
            ? collapseWaypoints(selectedTrack->records)
            : vector<string>{};
    }

    ProcedureTrackWithLocations readProcedureTrackWithLocations(
        istream& input,
        const string& recordType,
        const string& procedureName,
        const string& preferredRunway = "",
        const string& preferredTransition = "") const
    {
        auto tracks = parseProcedureTracks(input, recordType, procedureName);
        const RawTrack* selectedTrack = selectProcedureTrack(tracks, procedureName, preferredRunway, preferredTransition);
        return selectedTrack
            ? collapseWaypointsWithLocations(selectedTrack->records)
            : ProcedureTrackWithLocations{};
    }

    ProcedureTrackSelection readApproachProcedureTracks(
        const string& airportIcao,
        const string& procedureName,
        const string& preferredRunway = "",
        const string& preferredTransition = "") const
    {
        if (!m_host)
        {
            return {};
        }

        try
        {
            auto filePath = m_host->getHostFilePath({ "Custom Data", "CIFP", airportIcao + ".dat" });
            auto input = m_host->openFileForRead(filePath);
            if (!input)
            {
                return {};
            }
            return readApproachProcedureTracks(*input, procedureName, preferredRunway, preferredTransition);
        }
        catch (const exception&)
        {
            return {};
        }
    }

    ProcedureTrackSelection readApproachProcedureTracks(
        istream& input,
        const string& procedureName,
        const string& preferredRunway = "",
        const string& preferredTransition = "") const
    {
        auto tracks = parseProcedureTracks(input, "APPCH", procedureName);
        const RawTrack* selectedTrack = selectProcedureTrack(tracks, procedureName, preferredRunway, preferredTransition);
        return selectedTrack
            ? splitApproachTrack(*selectedTrack, preferredRunway)
            : ProcedureTrackSelection{};
    }

    struct ProcedureTrackWithLocationsSelection
    {
        ProcedureTrackWithLocations procedureTrack;
        ProcedureTrackWithLocations missedTrack;
    };

    ProcedureTrackWithLocationsSelection readApproachProcedureTracksWithLocations(
        istream& input,
        const string& procedureName,
        const string& preferredRunway = "",
        const string& preferredTransition = "") const
    {
        auto tracks = parseProcedureTracks(input, "APPCH", procedureName);
        const RawTrack* selectedTrack = selectProcedureTrack(tracks, procedureName, preferredRunway, preferredTransition);
        return selectedTrack
            ? splitApproachTrackWithLocations(*selectedTrack, preferredRunway)
            : ProcedureTrackWithLocationsSelection{};
    }

    ProcedureTrackWithLocationsSelection readApproachProcedureTracksWithLocations(
        const string& airportIcao,
        const string& procedureName,
        const string& preferredRunway = "",
        const string& preferredTransition = "") const
    {
        if (!m_host)
        {
            return {};
        }

        try
        {
            auto filePath = m_host->getHostFilePath({ "Custom Data", "CIFP", airportIcao + ".dat" });
            auto input = m_host->openFileForRead(filePath);
            if (!input)
            {
                return {};
            }
            return readApproachProcedureTracksWithLocations(*input, procedureName, preferredRunway, preferredTransition);
        }
        catch (const exception&)
        {
            return {};
        }
    }

    vector<string> enumProcedures(const string& airportIcao, const string& recordType) const
    {
        vector<string> result;
        if (!m_host)
        {
            return result;
        }

        // Check cache first
        const string cacheKey = airportIcao + "|" + recordType;
        {
            lock_guard<mutex> lock(s_procedureCacheMutex);
            auto it = s_procedureCache.find(cacheKey);
            if (it != s_procedureCache.end())
            {
                const auto& entry = it->second;
                if (chrono::steady_clock::now() - entry.timestamp < s_cacheTTL)
                {
                    return entry.procedures;
                }
                // Cache expired, remove it
                s_procedureCache.erase(it);
            }
        }

        try
        {
            auto filePath = m_host->getHostFilePath({ "Custom Data", "CIFP", airportIcao + ".dat" });
            auto input = m_host->openFileForRead(filePath);
            if (!input)
            {
                return result;
            }
            result = enumProcedures(*input, recordType);

            // Store in cache
            if (!result.empty())
            {
                lock_guard<mutex> lock(s_procedureCacheMutex);
                ProcedureListCacheEntry entry;
                entry.procedures = result;
                entry.timestamp = chrono::steady_clock::now();
                s_procedureCache[cacheKey] = std::move(entry);
            }

            return result;
        }
        catch (const exception&)
        {
            return result;
        }
    }

    vector<string> enumProcedures(istream& input, const string& recordType) const
    {
        vector<string> result;
        const string expectedRecordType = uppercaseCopy(recordType);
        unordered_set<string> seen;

        string line;
        while (getline(input, line))
        {
            if (line.empty())
            {
                continue;
            }

            const auto colonPos = line.find(':');
            if (colonPos == string::npos)
            {
                continue;
            }

            const string lineType = uppercaseCopy(trimCopy(line.substr(0, colonPos)));
            if (lineType != expectedRecordType)
            {
                continue;
            }

            string payload = line.substr(colonPos + 1);
            if (!payload.empty() && payload.back() == ';')
            {
                payload.pop_back();
            }

            const vector<string> fields = splitCsv(payload);
            if (fields.size() < 3)
            {
                continue;
            }

            const string procedureName = trimCopy(fields[2]);
            if (!procedureName.empty() && seen.insert(procedureName).second)
            {
                result.push_back(procedureName);
            }
        }

        return result;
    }

private:
    static vector<RawTrack> parseProcedureTracks(
        istream& input,
        const string& recordType,
        const string& procedureName)
    {
        vector<RawTrack> tracks;
        RawTrack currentTrack;
        int currentSequence = -1;
        bool hasTrack = false;
        const string expectedRecordType = uppercaseCopy(recordType);

        string line;
        while (getline(input, line))
        {
            if (line.empty())
            {
                continue;
            }

            const auto colonPos = line.find(':');
            if (colonPos == string::npos)
            {
                continue;
            }

            const string lineType = uppercaseCopy(trimCopy(line.substr(0, colonPos)));
            if (lineType != expectedRecordType)
            {
                continue;
            }

            string payload = line.substr(colonPos + 1);
            if (!payload.empty() && payload.back() == ';')
            {
                payload.pop_back();
            }

            const vector<string> fields = splitCsv(payload);
            if (fields.size() < 5)
            {
                continue;
            }

            int sequence = -1;
            try
            {
                sequence = stoi(trimCopy(fields[0]));
            }
            catch (const exception&)
            {
                continue;
            }

            const string parsedProcedureName = trimCopy(fields[2]);
            const string branchKey = firstNonBlank(trimCopy(fields[3]), trimCopy(fields[4]));
            const string waypoint = firstNonBlank(trimCopy(fields[4]), trimCopy(fields[3]));
            const string waypointDescription = fields.size() > 8 ? trimCopy(fields[8]) : "";
            const string pathTerminator = fields.size() > 11 ? trimCopy(fields[11]) : "";
            
            // Parse waypoint coordinates from CIFP (fields 6 and 7)
            float latitude = 0.0f;
            float longitude = 0.0f;
            bool hasLocation = false;
            if (fields.size() > 7)
            {
                const string latPacked = trimCopy(fields[6]);
                const string lonPacked = trimCopy(fields[7]);
                if (!latPacked.empty() && !lonPacked.empty())
                {
                    latitude = parsePackedCoordinate(latPacked);
                    longitude = parsePackedCoordinate(lonPacked);
                    hasLocation = (latitude != 0.0f || longitude != 0.0f);
                }
            }

            // Parse altitude constraint (this CIFP row layout stores the code/value around indices 22/23)
            float altitudeConstraint = 0.0f;
            char altitudeConstraintType = 0;
            if (fields.size() > 23)
            {
                const string altCode = trimCopy(fields[22]);  // +, -, or space
                const string altValue = trimCopy(fields[23]); // e.g., "03500" or "FL100"
                if (!altValue.empty())
                {
                    altitudeConstraintType = altCode.empty() ? ' ' : altCode[0];
                    try
                    {
                        if (altValue.rfind("FL", 0) == 0 || altValue.rfind("fl", 0) == 0)
                        {
                            // Flight level: FL100 = 10000 feet
                            altitudeConstraint = stof(altValue.substr(2)) * 100.0f;
                        }
                        else
                        {
                            altitudeConstraint = stof(altValue);
                        }
                    }
                    catch (const exception&)
                    {
                        altitudeConstraint = 0.0f;
                    }
                }
            }

            // Parse speed constraint (this CIFP row layout stores the code/value around indices 26/27)
            float speedConstraint = 0.0f;
            char speedConstraintType = 0;
            if (fields.size() > 27)
            {
                const string spdCode = trimCopy(fields[26]);  // +, -, or space
                const string spdValue = trimCopy(fields[27]); // e.g., "220"
                if (!spdValue.empty())
                {
                    speedConstraintType = spdCode.empty() ? ' ' : spdCode[0];
                    try
                    {
                        speedConstraint = stof(spdValue);
                    }
                    catch (const exception&)
                    {
                        speedConstraint = 0.0f;
                    }
                }
            }

            // Check if this is a new section of the same procedure
            // (same procedure name but sequence reset - e.g., Section A followed by Section I)
            const bool sameProcedure = hasTrack && matchesToken(currentTrack.procedureName, parsedProcedureName);
            const bool newSection = sequence <= currentSequence;
            // A different non-empty branchKey means a different transition/runway within the same
            // procedure (e.g., STAR transitions OKABE vs. DISIT, or SID runway transitions RW09L
            // vs. RW27R). These must become separate selectable tracks even when sameProcedure=true.
            const bool branchKeyChanged = hasTrack &&
                !branchKey.empty() &&
                !currentTrack.branchKey.empty() &&
                branchKey != currentTrack.branchKey;

            if (!hasTrack || (newSection && !sameProcedure) || (newSection && sameProcedure && branchKeyChanged))
            {
                if (hasTrack)
                {
                    tracks.push_back(currentTrack);
                }

                currentTrack = RawTrack();
                currentTrack.procedureName = parsedProcedureName;
                currentTrack.branchKey = branchKey;
                hasTrack = true;
            }
            else if (currentTrack.procedureName.empty())
            {
                currentTrack.procedureName = parsedProcedureName;
            }
            // If same procedure and new section, continue appending to current track

            if (currentTrack.branchKey.empty())
            {
                currentTrack.branchKey = branchKey;
            }

            currentTrack.records.push_back({
                sequence,
                parsedProcedureName,
                branchKey,
                waypoint,
                waypointDescription,
                pathTerminator,
                latitude,
                longitude,
                hasLocation,
                altitudeConstraint,
                altitudeConstraintType,
                speedConstraint,
                speedConstraintType
            });
            currentSequence = sequence;
        }

        if (hasTrack)
        {
            tracks.push_back(currentTrack);
        }

        return tracks;
    }

    static const RawTrack* selectProcedureTrack(
        const vector<RawTrack>& tracks,
        const string& procedureName,
        const string& preferredRunway,
        const string& preferredTransition)
    {
        if (tracks.empty())
        {
            return {};
        }

        const string runwayNorm = normalizeRunwayToken(preferredRunway);
        const string transitionNorm = normalizeToken(preferredTransition);
        const bool hasExplicitProcedureName = !procedureName.empty();

        const RawTrack* bestTrack = nullptr;
        int bestScore = numeric_limits<int>::min();

        // First pass: determine if any track has explicit runway matching
        // This helps us prefer runway-specific procedures over generic ones
        bool hasRunwaySpecificProcedure = false;
        if (!runwayNorm.empty() && !hasExplicitProcedureName)
        {
            for (const auto& track : tracks)
            {
                if (extractRunwayToken(track.procedureName) == runwayNorm)
                {
                    hasRunwaySpecificProcedure = true;
                    break;
                }
            }
        }

        for (const auto& track : tracks)
        {
            const vector<string> waypoints = collapseWaypoints(track.records);
            // Base score: prefer procedures with more waypoints (more complete)
            int score = static_cast<int>(waypoints.size()) * 10;

            if (hasExplicitProcedureName)
            {
                if (matchesToken(track.procedureName, procedureName))
                {
                    score += 10000; // Exact procedure match is highest priority
                }
                if (!extractRunwayToken(track.procedureName).empty() &&
                    extractRunwayToken(track.procedureName) == extractRunwayToken(procedureName))
                {
                    score += 800;
                }
            }

            if (!runwayNorm.empty())
            {
                // Strong preference for exact runway match in procedure name
                if (extractRunwayToken(track.procedureName) == runwayNorm)
                {
                    score += 5000;
                }
                // Deprioritize generic procedures when runway-specific ones exist
                else if (hasRunwaySpecificProcedure && extractRunwayToken(track.procedureName).empty())
                {
                    score -= 2000;
                }
                if (matchesRunwayToken(track.branchKey, preferredRunway))
                {
                    score += 3000;
                }
                if (hasMatchingRunwayWaypoint(waypoints, preferredRunway))
                {
                    score += 2000;
                }
            }

            if (!transitionNorm.empty())
            {
                if (matchesToken(track.branchKey, preferredTransition))
                {
                    score += 600;
                }
                if (hasMatchingWaypoint(waypoints, preferredTransition))
                {
                    score += 500;
                }
            }

            if (!bestTrack || score > bestScore)
            {
                bestTrack = &track;
                bestScore = score;
            }
        }

        return bestTrack;
    }

    static vector<string> collapseWaypoints(const vector<ProcedureRecord>& records)
    {
        vector<string> waypoints;
        if (!records.empty())
        {
            const string& initialKey = records.front().branchKey;
            if (uppercaseCopy(trimCopy(initialKey)).rfind("RW", 0) != 0)
            {
                appendUnique(waypoints, initialKey);
            }
        }
        for (const auto& record : records)
        {
            appendUnique(waypoints, record.waypoint);
        }
        return waypoints;
    }

    static ProcedureTrackWithLocations collapseWaypointsWithLocations(const vector<ProcedureRecord>& records)
    {
        ProcedureTrackWithLocations result;

        // Helper to check if waypoint already exists
        const auto hasWaypoint = [&result](const string& name) {
            return any_of(result.waypoints.begin(), result.waypoints.end(),
                [&name](const WaypointWithLocation& wp) { return matchesToken(wp.name, name); });
        };

        if (!records.empty())
        {
            const auto& firstRecord = records.front();
            const string& initialKey = firstRecord.branchKey;
            if (uppercaseCopy(trimCopy(initialKey)).rfind("RW", 0) != 0)
            {
                result.waypoints.emplace_back(initialKey, 0.0f, 0.0f, false,
                    firstRecord.altitudeConstraint, firstRecord.altitudeConstraintType,
                    firstRecord.speedConstraint, firstRecord.speedConstraintType);
            }
        }

        for (const auto& record : records)
        {
            if (!hasWaypoint(record.waypoint))
            {
                result.waypoints.emplace_back(record.waypoint, record.latitude, record.longitude, record.hasLocation,
                    record.altitudeConstraint, record.altitudeConstraintType,
                    record.speedConstraint, record.speedConstraintType);
            }
        }

        return result;
    }

    static ProcedureTrackSelection splitApproachTrack(const RawTrack& track, const string& preferredRunway)
    {
        vector<ProcedureRecord> approachRecords;
        vector<ProcedureRecord> missedRecords;
        bool passedMissedApproachPoint = false;

        for (const auto& record : track.records)
        {
            if (!passedMissedApproachPoint)
            {
                approachRecords.push_back(record);
                passedMissedApproachPoint = isMissedApproachPoint(record, preferredRunway);
            }
            else
            {
                missedRecords.push_back(record);
            }
        }

        return {
            collapseWaypoints(approachRecords),
            collapseWaypoints(missedRecords)
        };
    }

    static ProcedureTrackWithLocationsSelection splitApproachTrackWithLocations(const RawTrack& track, const string& preferredRunway)
    {
        vector<ProcedureRecord> approachRecords;
        vector<ProcedureRecord> missedRecords;
        bool passedMissedApproachPoint = false;

        for (const auto& record : track.records)
        {
            if (!passedMissedApproachPoint)
            {
                approachRecords.push_back(record);
                passedMissedApproachPoint = isMissedApproachPoint(record, preferredRunway);
            }
            else
            {
                missedRecords.push_back(record);
            }
        }

        return {
            collapseWaypointsWithLocations(approachRecords),
            collapseWaypointsWithLocations(missedRecords)
        };
    }

    // Parse packed CIFP coordinates (e.g., "N43301750" -> 43.504861)
    // Format: N/S followed by DDMMSSSS (degrees, minutes, seconds with 2 decimal places)
    // Format: E/W followed by DDDMMSSSS (degrees, minutes, seconds with 2 decimal places)
    static float parsePackedCoordinate(const string& packed)
    {
        if (packed.empty() || packed.length() < 7)
        {
            return 0.0f;
        }
        
        char direction = packed[0];
        bool isNegative = (direction == 'S' || direction == 'W');
        
        try
        {
            if (direction == 'N' || direction == 'S')
            {
                // Latitude: DDMMSSSS (degrees 2 digits)
                int degrees = stoi(packed.substr(1, 2));
                int minutes = stoi(packed.substr(3, 2));
                int secondsTimes100 = stoi(packed.substr(5));
                float seconds = secondsTimes100 / 100.0f;
                float result = degrees + minutes / 60.0f + seconds / 3600.0f;
                return isNegative ? -result : result;
            }
            else if (direction == 'E' || direction == 'W')
            {
                // Longitude: DDDMMSSSS (degrees 3 digits)
                int degrees = stoi(packed.substr(1, 3));
                int minutes = stoi(packed.substr(4, 2));
                int secondsTimes100 = stoi(packed.substr(6));
                float seconds = secondsTimes100 / 100.0f;
                float result = degrees + minutes / 60.0f + seconds / 3600.0f;
                return isNegative ? -result : result;
            }
        }
        catch (const exception&)
        {
            return 0.0f;
        }
        
        return 0.0f;
    }

    static bool isMissedApproachPoint(const ProcedureRecord& record, const string& preferredRunway)
    {
        const string descriptor = normalizeToken(record.waypointDescription);
        if (descriptor.find('M') != string::npos)
        {
            return true;
        }

        if (!preferredRunway.empty() && (
            matchesRunwayToken(record.branchKey, preferredRunway) ||
            matchesRunwayToken(record.waypoint, preferredRunway)))
        {
            return true;
        }

        // Only treat waypoint as missed approach point if it exactly matches the procedure name
        // (e.g., waypoint "03" or "RW03" for procedure "ILS03" or "03").
        // Using exact match instead of matchesRunwayToken to avoid false positives
        // where waypoints like "CO03W" would incorrectly match procedure name "03".
        const string waypointNorm = normalizeToken(record.waypoint);
        const string procedureNorm = normalizeToken(record.procedureName);
        return waypointNorm == procedureNorm ||
               (waypointNorm.rfind("RW", 0) == 0 && waypointNorm.substr(2) == procedureNorm);
    }

    static bool hasMatchingWaypoint(const vector<string>& waypoints, const string& value)
    {
        return any_of(waypoints.begin(), waypoints.end(), [&value](const string& waypoint) {
            return matchesToken(waypoint, value);
        });
    }

    static bool hasMatchingRunwayWaypoint(const vector<string>& waypoints, const string& value)
    {
        return any_of(waypoints.begin(), waypoints.end(), [&value](const string& waypoint) {
            return matchesRunwayToken(waypoint, value);
        });
    }

    static void appendUnique(vector<string>& waypoints, const string& value)
    {
        if (value.empty())
        {
            return;
        }

        if (none_of(waypoints.begin(), waypoints.end(), [&value](const string& existing) {
            return matchesToken(existing, value);
        }))
        {
            waypoints.push_back(value);
        }
    }

    static vector<string> splitCsv(const string& text)
    {
        vector<string> values;
        string current;

        for (char c : text)
        {
            if (c == ',')
            {
                values.push_back(trimCopy(current));
                current.clear();
            }
            else if (c != '\r' && c != '\n')
            {
                current.push_back(c);
            }
        }

        values.push_back(trimCopy(current));
        return values;
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

    static string firstNonBlank(const string& lhs, const string& rhs)
    {
        return lhs.empty() ? rhs : lhs;
    }
};
