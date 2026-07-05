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

using namespace std;
using namespace world;

class XPMissedApproachReader
{
private:
    struct MissedApproachRecord
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
        float altitudeConstraint = 0.0f;
        char altitudeConstraintType = 0;
        float speedConstraint = 0.0f;
        char speedConstraintType = 0;
    };

    struct RawMissedTrack
    {
        string procedureName;
        string branchKey;
        vector<MissedApproachRecord> records;
    };

    shared_ptr<HostServices> m_host;

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

    static float parseFloat(const string& text, float defaultValue = 0.0f)
    {
        if (text.empty())
        {
            return defaultValue;
        }

        try
        {
            return stof(text);
        }
        catch (const exception&)
        {
            return defaultValue;
        }
    }

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
                int degrees = stoi(packed.substr(1, 2));
                int minutes = stoi(packed.substr(3, 2));
                int secondsTimes100 = stoi(packed.substr(5));
                float seconds = secondsTimes100 / 100.0f;
                float result = degrees + minutes / 60.0f + seconds / 3600.0f;
                return isNegative ? -result : result;
            }
            else if (direction == 'E' || direction == 'W')
            {
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

    static bool matchesToken(const string& lhs, const string& rhs)
    {
        return normalizeToken(lhs) == normalizeToken(rhs);
    }

    static string firstNonBlank(const string& lhs, const string& rhs)
    {
        return lhs.empty() ? rhs : lhs;
    }

    static vector<RawMissedTrack> parseMissedApproachTracks(
        istream& input,
        const string& procedureName)
    {
        vector<RawMissedTrack> tracks;
        RawMissedTrack currentTrack;
        int currentSequence = -1;
        bool hasTrack = false;
        const string expectedProcedureName = uppercaseCopy(trimCopy(procedureName));

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
            if (lineType != "APPCH")
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

            // Parse waypoint coordinates
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

            // Parse altitude constraint
            float altitudeConstraint = 0.0f;
            char altitudeConstraintType = 0;
            if (fields.size() > 23)
            {
                const string altCode = trimCopy(fields[22]);
                const string altValue = trimCopy(fields[23]);
                if (!altValue.empty())
                {
                    altitudeConstraintType = altCode.empty() ? ' ' : altCode[0];
                    try
                    {
                        if (altValue.rfind("FL", 0) == 0 || altValue.rfind("fl", 0) == 0)
                        {
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

            // Parse speed constraint
            float speedConstraint = 0.0f;
            char speedConstraintType = 0;
            if (fields.size() > 27)
            {
                const string spdCode = trimCopy(fields[26]);
                const string spdValue = trimCopy(fields[27]);
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

            // Check if this is a missed approach segment
            // Missed approach waypoints typically have 'M' in the description or
            // are part of a missed approach track
            const bool isMissedApproach = waypointDescription.find('M') != string::npos ||
                waypointDescription.find('m') != string::npos;

            if (!isMissedApproach)
            {
                continue;
            }

            // Check if this is a new section of the same procedure
            const bool sameProcedure = hasTrack && matchesToken(currentTrack.procedureName, parsedProcedureName);
            const bool newSection = sequence <= currentSequence;
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

                currentTrack = RawMissedTrack();
                currentTrack.procedureName = parsedProcedureName;
                currentTrack.branchKey = branchKey;
                hasTrack = true;
            }
            else if (currentTrack.procedureName.empty())
            {
                currentTrack.procedureName = parsedProcedureName;
            }

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

public:
    explicit XPMissedApproachReader(shared_ptr<HostServices> host) :
        m_host(std::move(host))
    {
    }

    struct MissedApproachWaypoint
    {
        string name;
        float latitude = 0.0f;
        float longitude = 0.0f;
        bool hasLocation = false;
        float altitudeConstraint = 0.0f;
        char altitudeConstraintType = 0;
        float speedConstraint = 0.0f;
        char speedConstraintType = 0;
        string pathTerminator;
        string description;

        MissedApproachWaypoint() = default;
        MissedApproachWaypoint(const string& n, float lat, float lon, bool hasLoc)
            : name(n), latitude(lat), longitude(lon), hasLocation(hasLoc) {}
        MissedApproachWaypoint(const string& n, float lat, float lon, bool hasLoc,
                               float altConstr, char altType, float spdConstr, char spdType,
                               const string& pathTerm, const string& desc)
            : name(n), latitude(lat), longitude(lon), hasLocation(hasLoc),
              altitudeConstraint(altConstr), altitudeConstraintType(altType),
              speedConstraint(spdConstr), speedConstraintType(spdType),
              pathTerminator(pathTerm), description(desc) {}
    };

    struct MissedApproachTrack
    {
        vector<MissedApproachWaypoint> waypoints;
        string procedureName;
        string branchKey;

        bool empty() const
        {
            return waypoints.empty();
        }

        size_t size() const
        {
            return waypoints.size();
        }
    };

    MissedApproachTrack readMissedApproachTrack(
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

            return readMissedApproachTrack(*input, procedureName, preferredRunway, preferredTransition);
        }
        catch (const exception&)
        {
            return {};
        }
    }

    MissedApproachTrack readMissedApproachTrack(
        istream& input,
        const string& procedureName,
        const string& preferredRunway = "",
        const string& preferredTransition = "") const
    {
        auto tracks = parseMissedApproachTracks(input, procedureName);
        if (tracks.empty())
        {
            return {};
        }

        // Select the best track based on runway and transition preferences
        const RawMissedTrack* bestTrack = selectMissedApproachTrack(
            tracks, procedureName, preferredRunway, preferredTransition);

        if (!bestTrack)
        {
            return {};
        }

        MissedApproachTrack result;
        result.procedureName = bestTrack->procedureName;
        result.branchKey = bestTrack->branchKey;

        for (const auto& record : bestTrack->records)
        {
            result.waypoints.emplace_back(
                record.waypoint,
                record.latitude,
                record.longitude,
                record.hasLocation,
                record.altitudeConstraint,
                record.altitudeConstraintType,
                record.speedConstraint,
                record.speedConstraintType,
                record.pathTerminator,
                record.waypointDescription
            );
        }

        return result;
    }

    // Check if a procedure has a missed approach segment
    bool hasMissedApproach(
        const string& airportIcao,
        const string& procedureName) const
    {
        if (!m_host)
        {
            return false;
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
                return false;
            }

            return hasMissedApproach(*input, procedureName);
        }
        catch (const exception&)
        {
            return false;
        }
    }

    bool hasMissedApproach(istream& input, const string& procedureName) const
    {
        auto tracks = parseMissedApproachTracks(input, procedureName);
        return !tracks.empty();
    }

    // Get the missed approach holding fix (if any)
    // Returns nullptr if no holding fix found, otherwise returns pointer to string
    const string* getMissedApproachHoldingFix(
        const string& airportIcao,
        const string& procedureName) const
    {
        const auto track = readMissedApproachTrack(airportIcao, procedureName);
        if (track.empty())
        {
            return nullptr;
        }

        // The last waypoint in the missed approach track is typically the holding fix
        // Look for a waypoint that appears to be a fix (not a runway)
        for (auto it = track.waypoints.rbegin(); it != track.waypoints.rend(); ++it)
        {
            const string& name = it->name;
            // Skip runway waypoints
            if (name.size() >= 2 && name.substr(0, 2) == "RW")
            {
                continue;
            }
            // Skip if it looks like a runway number
            if (name.size() <= 3 && all_of(name.begin(), name.end(), [](char c) { return isdigit(c); }))
            {
                continue;
            }
            return &name;
        }

        return nullptr;
    }

private:
    static const RawMissedTrack* selectMissedApproachTrack(
        const vector<RawMissedTrack>& tracks,
        const string& procedureName,
        const string& preferredRunway,
        const string& preferredTransition)
    {
        if (tracks.empty())
        {
            return {};
        }

        const string runwayNorm = normalizeToken(preferredRunway);
        const string transitionNorm = normalizeToken(preferredTransition);
        const bool hasExplicitProcedureName = !procedureName.empty();

        const RawMissedTrack* bestTrack = nullptr;
        int bestScore = numeric_limits<int>::min();

        for (const auto& track : tracks)
        {
            int score = static_cast<int>(track.records.size()) * 10;

            if (hasExplicitProcedureName)
            {
                if (matchesToken(track.procedureName, procedureName))
                {
                    score += 10000;
                }
            }

            if (!runwayNorm.empty())
            {
                // Check if branch key matches runway
                if (normalizeToken(track.branchKey).find(runwayNorm) != string::npos)
                {
                    score += 5000;
                }
            }

            if (!transitionNorm.empty())
            {
                if (normalizeToken(track.branchKey).find(transitionNorm) != string::npos)
                {
                    score += 600;
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
};
