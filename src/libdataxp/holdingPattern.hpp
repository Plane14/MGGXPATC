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

class XPHoldingPatternReader
{
private:
    struct HoldingPatternRecord
    {
        string ident;
        string region;
        float inboundCourse = 0.0f;      // Degrees true
        float turnDirection = 0.0f;      // 1.0 = right, 0.0 = left, 3.0-5.0 = standard rate (degrees/sec)
        float legTimeMinutes = 0.0f;     // Minutes (0 if distance-based)
        float legDistanceNm = 0.0f;      // Nautical miles (0 if time-based)
        float minimumAltitude = 0.0f;    // Feet
        float maximumAltitude = 0.0f;    // Feet (0 if no limit)
        float maximumSpeed = 0.0f;       // Knots (0 if no limit)
        bool hasLocation = false;
        float latitude = 0.0f;
        float longitude = 0.0f;
    };

    struct Cache
    {
        unordered_map<string, vector<HoldingPatternRecord>> recordsByKey;
    };

    shared_ptr<HostServices> m_host;
    mutable shared_ptr<Cache> m_cache;
    mutable mutex m_cacheMutex;

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

    void ensureLoaded() const
    {
        if (m_cache)
        {
            return;
        }

        lock_guard<mutex> lock(m_cacheMutex);
        if (m_cache)
        {
            return;
        }

        auto cache = make_shared<Cache>();
        loadHoldData(*cache);
        m_cache = cache;
    }

    void loadHoldData(Cache& cache) const
    {
        if (!m_host)
        {
            return;
        }

        // Try custom data first, then default data
        vector<vector<string>> paths = {
            { "Custom Data", "earth_hold.dat" },
            { "Resources", "default data", "earth_hold.dat" }
        };

        for (const auto& pathParts : paths)
        {
            auto input = m_host->openFileForRead(m_host->getHostFilePath(pathParts));
            if (!input)
            {
                continue;
            }

            parseHoldData(*input, cache);
            break;
        }
    }

    void parseHoldData(istream& input, Cache& cache) const
    {
        string line;
        bool inDataSection = false;

        while (getline(input, line))
        {
            if (line.empty())
            {
                continue;
            }

            // Data section starts after "I" header line and version line
            if (line[0] == 'I')
            {
                inDataSection = true;
                continue;
            }

            if (!inDataSection)
            {
                continue;
            }

            // Skip terminator
            if (line[0] == '9' || line[0] == '9' && line.size() > 1 && line[1] == '9')
            {
                break;
            }

            // Parse holding pattern record
            // Format: IDENT TYPE REGION BEARING TURN_RATE LEG_TIME LEG_DIST MIN_ALT MAX_ALT SPEED
            const vector<string> fields = splitCsv(line);
            if (fields.size() < 10)
            {
                continue;
            }

            HoldingPatternRecord record;
            record.ident = trimCopy(fields[0]);
            record.region = trimCopy(fields[2]);
            record.inboundCourse = parseFloat(trimCopy(fields[3]));
            record.turnDirection = parseFloat(trimCopy(fields[4]));
            record.legTimeMinutes = parseFloat(trimCopy(fields[5]));
            record.legDistanceNm = parseFloat(trimCopy(fields[6]));
            record.minimumAltitude = parseFloat(trimCopy(fields[7]));
            record.maximumAltitude = parseFloat(trimCopy(fields[8]));
            record.maximumSpeed = parseFloat(trimCopy(fields[9]));

            const string key = normalizeKey(record.ident);
            cache.recordsByKey[key].push_back(record);
        }
    }

public:
    explicit XPHoldingPatternReader(shared_ptr<HostServices> host) :
        m_host(std::move(host)),
        m_cache(),
        m_cacheMutex()
    {
    }

    struct HoldingPattern
    {
        string ident;
        string region;
        float inboundCourse = 0.0f;      // Degrees true
        bool isRightTurn = true;         // true = right turn, false = left turn
        float turnRate = 3.0f;           // Degrees per second (standard = 3.0)
        float legTimeMinutes = 0.0f;     // Minutes (0 if distance-based)
        float legDistanceNm = 0.0f;      // Nautical miles (0 if time-based)
        float minimumAltitude = 0.0f;    // Feet
        float maximumAltitude = 0.0f;    // Feet (0 if no limit)
        float maximumSpeed = 0.0f;       // Knots (0 if no limit)
        bool hasLocation = false;
        float latitude = 0.0f;
        float longitude = 0.0f;

        bool isTimedLeg() const
        {
            return legTimeMinutes > 0.0f;
        }

        bool isDistanceLeg() const
        {
            return legDistanceNm > 0.0f;
        }

        float outboundCourse() const
        {
            // Outbound course is reciprocal of inbound
            float result = inboundCourse + 180.0f;
            if (result >= 360.0f)
            {
                result -= 360.0f;
            }
            return result;
        }

        float turnRadiusNm(float speedKnots) const
        {
            // Standard rate turn: 3 degrees per second
            // Radius = speed / (2 * pi * turnRate) * (6076 / 3600)
            // Simplified: radius (NM) = speed (knots) / (turnRate * 10.2)
            if (speedKnots <= 0.0f || turnRate <= 0.0f)
            {
                return 0.0f;
            }

            return speedKnots / (turnRate * 10.2f);
        }
    };

    vector<HoldingPattern> queryHoldingPatterns(const string& fixIdent) const
    {
        ensureLoaded();

        vector<HoldingPattern> result;
        if (!m_cache)
        {
            return result;
        }

        const string key = normalizeKey(fixIdent);
        const auto found = m_cache->recordsByKey.find(key);
        if (found == m_cache->recordsByKey.end())
        {
            return result;
        }

        for (const auto& record : found->second)
        {
            HoldingPattern pattern;
            pattern.ident = record.ident;
            pattern.region = record.region;
            pattern.inboundCourse = record.inboundCourse;
            pattern.isRightTurn = (record.turnDirection >= 1.0f);
            pattern.turnRate = (record.turnDirection >= 3.0f) ? record.turnDirection : 3.0f;
            pattern.legTimeMinutes = record.legTimeMinutes;
            pattern.legDistanceNm = record.legDistanceNm;
            pattern.minimumAltitude = record.minimumAltitude;
            pattern.maximumAltitude = record.maximumAltitude;
            pattern.maximumSpeed = record.maximumSpeed;
            pattern.hasLocation = record.hasLocation;
            pattern.latitude = record.latitude;
            pattern.longitude = record.longitude;

            result.push_back(pattern);
        }

        return result;
    }

    bool queryHoldingPattern(const string& fixIdent, const string& region, HoldingPattern& outPattern) const
    {
        const auto patterns = queryHoldingPatterns(fixIdent);
        if (patterns.empty())
        {
            return false;
        }

        // If region specified, try to find exact match
        if (!region.empty())
        {
            const string regionNorm = normalizeKey(region);
            for (const auto& pattern : patterns)
            {
                if (normalizeKey(pattern.region) == regionNorm)
                {
                    outPattern = pattern;
                    return true;
                }
            }
        }

        // Return first pattern (most common)
        outPattern = patterns.front();
        return true;
    }

    bool hasHoldingPattern(const string& fixIdent) const
    {
        ensureLoaded();
        if (!m_cache)
        {
            return false;
        }

        const string key = normalizeKey(fixIdent);
        return m_cache->recordsByKey.find(key) != m_cache->recordsByKey.end();
    }
};
