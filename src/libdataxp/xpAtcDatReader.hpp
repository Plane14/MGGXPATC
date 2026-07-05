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
    class XPAtcDatReader
    {
    public:
        struct ControllerEntry
        {
            string name;
            string facilityId;
            string role;
            vector<int> frequenciesKhz;
            struct AirspacePolygon
            {
                float floorFeet;
                float ceilingFeet;
                vector<GeoPoint> points;
            };
            vector<AirspacePolygon> airspaces;
        };

    private:
        shared_ptr<HostServices> m_host;

    public:
        explicit XPAtcDatReader(shared_ptr<HostServices> host) : m_host(std::move(host))
        {
        }

    public:
        vector<ControllerEntry> read(istream& input)
        {
            vector<ControllerEntry> entries;
            string line;
            ControllerEntry current;
            ControllerEntry::AirspacePolygon currentPolygon;
            bool inController = false;
            bool inPolygon = false;

            while (getline(input, line))
            {
                line.erase(0, line.find_first_not_of(" \t\r\n"));
                line.erase(line.find_last_not_of(" \t\r\n") + 1);

                if (line.empty())
                {
                    continue;
                }

                // Skip header lines before the first controller block, but not inside a block
                if (!inController && (line[0] == 'A' || line[0] == '1'))
                {
                    continue;
                }

                if (line == "CONTROLLER")
                {
                    if (inController && !current.name.empty())
                    {
                        if (inPolygon)
                        {
                            current.airspaces.push_back(currentPolygon);
                            inPolygon = false;
                        }
                        entries.push_back(current);
                    }
                    current = ControllerEntry{};
                    currentPolygon = ControllerEntry::AirspacePolygon{};
                    inController = true;
                    inPolygon = false;
                    continue;
                }

                if (!inController)
                {
                    continue;
                }

                if (line == "CONTROLLER_END")
                {
                    if (inPolygon)
                    {
                        current.airspaces.push_back(currentPolygon);
                        inPolygon = false;
                    }
                    if (!current.name.empty())
                    {
                        entries.push_back(current);
                    }
                    inController = false;
                    current = ControllerEntry{};
                    continue;
                }

                istringstream iss(line);
                string token;
                if (!(iss >> token))
                {
                    continue;
                }

                if (token == "NAME")
                {
                    string rest;
                    getline(iss, rest);
                    current.name = trim(rest);
                }
                else if (token == "FACILITY_ID")
                {
                    iss >> current.facilityId;
                }
                else if (token == "ROLE")
                {
                    iss >> current.role;
                }
                else if (token == "FREQ")
                {
                    int freq = 0;
                    if (iss >> freq)
                    {
                        current.frequenciesKhz.push_back(freq);
                    }
                }
                else if (token == "AIRSPACE_POLYGON_BEGIN")
                {
                    if (inPolygon)
                    {
                        current.airspaces.push_back(currentPolygon);
                    }
                    currentPolygon = ControllerEntry::AirspacePolygon{};
                    iss >> currentPolygon.floorFeet >> currentPolygon.ceilingFeet;
                    inPolygon = true;
                }
                else if (token == "AIRSPACE_POLYGON_END")
                {
                    if (inPolygon)
                    {
                        current.airspaces.push_back(currentPolygon);
                        inPolygon = false;
                    }
                }
                else if (token == "POINT")
                {
                    if (inPolygon)
                    {
                        double lat = 0, lon = 0;
                        if (iss >> lat >> lon)
                        {
                            currentPolygon.points.push_back(GeoPoint(lat, lon));
                        }
                    }
                }
            }

            if (inController && !current.name.empty())
            {
                if (inPolygon)
                {
                    current.airspaces.push_back(currentPolygon);
                }
                entries.push_back(current);
            }

            return entries;
        }

    private:
        static string trim(const string& s)
        {
            size_t start = s.find_first_not_of(" \t\r\n");
            if (start == string::npos)
            {
                return "";
            }
            size_t end = s.find_last_not_of(" \t\r\n");
            return s.substr(start, end - start + 1);
        }
    };
}
