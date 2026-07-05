//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once
#include <istream>
#include <sstream>
#include <vector>
#include <memory>
#include <string>
#include <cmath>
#include <cctype>
#include "libworld.h"

using namespace std;
using namespace world;

namespace dataxp
{
    class OpenAirAirspaceReader
    {
    public:
        struct AirspaceEntry
        {
            string classCode;
            string name;
            string lowerLimit;
            string upperLimit;
            vector<GeoPoint> polygon;
        };

    private:
        shared_ptr<HostServices> m_host;

    public:
        explicit OpenAirAirspaceReader(shared_ptr<HostServices> host) : m_host(std::move(host))
        {
        }

    public:
        vector<AirspaceEntry> read(istream& input)
        {
            vector<AirspaceEntry> entries;
            AirspaceEntry current;
            string line;
            bool hasEntry = false;

            while (getline(input, line))
            {
                line.erase(0, line.find_first_not_of(" \t\r\n"));
                line.erase(line.find_last_not_of(" \t\r\n") + 1);

                if (line.empty())
                {
                    if (hasEntry && current.polygon.size() >= 3)
                    {
                        entries.push_back(current);
                    }
                    current = AirspaceEntry{};
                    hasEntry = false;
                    continue;
                }

                if (line.size() < 3 || line[2] != ' ')
                {
                    continue;
                }

                string code = line.substr(0, 2);
                string value = (line.length() > 3) ? line.substr(3) : "";
                value.erase(0, value.find_first_not_of(" \t\r\n"));
                value.erase(value.find_last_not_of(" \t\r\n") + 1);

                if (code == "AC")
                {
                    current.classCode = value;
                    hasEntry = true;
                }
                else if (code == "AN")
                {
                    current.name = value;
                }
                else if (code == "AL")
                {
                    current.lowerLimit = value;
                }
                else if (code == "AH")
                {
                    current.upperLimit = value;
                }
                else if (code == "DP")
                {
                    GeoPoint point;
                    if (tryParseOpenAirPoint(value, point))
                    {
                        current.polygon.push_back(point);
                    }
                }
            }

            if (hasEntry && current.polygon.size() >= 3)
            {
                entries.push_back(current);
            }

            return entries;
        }

    private:
        static bool tryParseOpenAirPoint(const string& text, GeoPoint& point)
        {
            istringstream iss(text);
            string latDms, latHem, lonDms, lonHem;
            if (!(iss >> latDms >> latHem >> lonDms >> lonHem))
            {
                return false;
            }

            double lat = 0, lon = 0;
            if (!tryParseDms(latDms, latHem[0], lat) ||
                !tryParseDms(lonDms, lonHem[0], lon))
            {
                return false;
            }

            point = GeoPoint(lat, lon);
            return true;
        }

        static bool tryParseDms(const string& dmsText, char hemisphere, double& value)
        {
            vector<string> parts;
            string current;

            for (const auto ch : dmsText)
            {
                if (ch == ':')
                {
                    parts.push_back(current);
                    current.clear();
                }
                else
                {
                    current.push_back(ch);
                }
            }
            parts.push_back(current);

            if (parts.size() != 3)
            {
                return false;
            }

            try
            {
                const double degrees = stod(parts.at(0));
                const double minutes = stod(parts.at(1));
                const double seconds = stod(parts.at(2));
                value = degrees + minutes / 60.0 + seconds / 3600.0;
                const char upperHem = static_cast<char>(toupper(static_cast<unsigned char>(hemisphere)));
                if (upperHem == 'S' || upperHem == 'W')
                {
                    value *= -1.0;
                }
                return true;
            }
            catch (const exception&)
            {
                return false;
            }
        }
    };
}
