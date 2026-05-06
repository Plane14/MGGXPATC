// Lightweight adapter to fetch airport schedules from airnavradar.com.
// It prefers the browser-facing /data/airports/search/<ICAO> JSON endpoint on
// www.airnavradar.com (with the page's tab key) and falls back to the legacy
// page/embedded-JSON heuristics when necessary.
#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <ctime>
#include <cstdio>
#include "fr24AirportScheduleSource.hpp"

using namespace std;

class AirnavradarAirportScheduleSource
{
private:
    shared_ptr<HostServices> m_host;

public:
    explicit AirnavradarAirportScheduleSource(shared_ptr<HostServices> host) : m_host(std::move(host)) {}

    bool tryLoadAirportSchedules(const string& airportIcao, vector<Fr24ScheduleEntry>& departures, vector<Fr24ScheduleEntry>& arrivals)
    {
        departures.clear();
        arrivals.clear();

        Fr24AirportScheduleSource helper(m_host);
        string code = airportIcao;
        transform(code.begin(), code.end(), code.begin(), [](unsigned char c) { return static_cast<char>(toupper(c)); });

        bool foundAny = false;
        if (tryLoadAirportSchedulesFromSearchEndpoint(helper, code, departures, arrivals))
        {
            foundAny = true;
        }
        else
        {
            string url = string("https://www.airnavradar.com/data/airports/") + code;
            string page;
            // Use the shared fetch path so Zendriver, daemon, and curl fallback can all help.
            m_host->writeLog("AIRNAV|fetching airport[%s] with shared fetch", airportIcao.c_str());
            if (!helper.fetchPage(url, page))
            {
                m_host->writeLog("AIRNAV|fetch failed for airport[%s] using shared fetch path", airportIcao.c_str());
                return false;
            }

            // Log fetched page for diagnostics (truncated)
            m_host->writeLog("AIRNAV|fetched page for airport[%s] len[%zu]", airportIcao.c_str(), page.size());

            // First try: the page may contain an embedded JSON blob (window._APP_STATE)
            string json = page;
            bool foundEmbedded = false;

            auto tryParseAndCollect = [&](const string& candidateText, const string& mode) -> bool {
                string s = candidateText;
                if (!Fr24AirportScheduleSource::extractJsonPayloadFromText(s))
                {
                    return false;
                }

                try
                {
                    SimpleJson root = SimpleJson::parse(s);

                    const SimpleJson* arrayNode = nullptr;
                    if (auto n = root.tryGet(mode)) arrayNode = n;
                    else if (auto n = root.tryGet("data")) arrayNode = n;
                    else if (auto n = root.tryGet("flights")) arrayNode = n;

                    if ((!arrayNode || !arrayNode->isArray()) && root.isObject())
                    {
                        for (const auto& kv : root.objectValue())
                        {
                            if (!kv.second.isObject()) continue;
                            if (auto n = kv.second.tryGet(mode)) { arrayNode = n; break; }
                            if (auto n = kv.second.tryGet("data")) { arrayNode = n; break; }
                            if (auto n = kv.second.tryGet("flights")) { arrayNode = n; break; }
                        }
                    }

                    if (!arrayNode || !arrayNode->isArray())
                    {
                        return false;
                    }

                    for (const auto& item : arrayNode->arrayValue())
                    {
                        if (!item.isObject()) continue;
                        try
                        {
                            Fr24ScheduleEntry e;
                            const SimpleJson* id = item.tryGet("identification");
                            if (id)
                            {
                                e.flightNumber = getStringOrEmpty(*id, { "number", "default" });
                                e.callsign = getStringOrEmpty(*id, { "callsign" });
                            }

                            e.originIcao = getStringOrEmpty(item, { "origin", "icao" });
                            e.destinationIcao = getStringOrEmpty(item, { "destination", "icao" });
                            e.airlineIcao = getStringOrEmpty(item, { "airline", "icao" });

                            long long ts = getIntegerOrZero(item, { "time", "scheduled" });
                            if (ts == 0) ts = getIntegerOrZero(item, { "scheduledTime" });
                            e.scheduledTime = (time_t)ts;

                            if (!e.flightNumber.empty())
                            {
                                if (mode == "departures") departures.push_back(e);
                                else arrivals.push_back(e);
                            }
                        }
                        catch(...) {}
                    }

                    return true;
                }
                catch (...) { return false; }
            };

            if (tryParseAndCollect(json, "departures") || tryParseAndCollect(json, "arrivals"))
            {
                foundEmbedded = true;
                foundAny = true;
            }

            string dataApiBase;
            try
            {
                string copy = page;
                if (Fr24AirportScheduleSource::extractJsonPayloadFromText(copy))
                {
                    SimpleJson state = SimpleJson::parse(copy);
                    const SimpleJson* env = state.tryGet("env");
                    if (env && env->isObject())
                    {
                        const SimpleJson* api = env->tryGet("DATA_API");
                        if (api && !api->isNull())
                        {
                            dataApiBase = api->asString();
                        }
                    }
                }
            }
            catch(...) { }

            if (!dataApiBase.empty())
            {
                if (!dataApiBase.empty() && dataApiBase.back() == '/') dataApiBase.pop_back();
                {
                    string key = string("AIRNAV_API_BASE:") + (airportIcao.size() > 8 ? airportIcao.substr(0,8) : airportIcao);
                    if (rateLimitDetailedLog(key))
                    {
                        m_host->writeLog("AIRNAV|dataApiBase detected [%s] for airport[%s]", dataApiBase.c_str(), airportIcao.c_str());
                    }
                    else
                    {
                        m_host->writeLog("AIRNAV|dataApiBase detected for airport[%s]", airportIcao.c_str());
                    }
                }
                vector<string> candidates = {
                    dataApiBase + string("/data/airports/search/") + code + "?key=mrgaporgic",
                    dataApiBase + string("/data/airports/search/") + code + "?key=mrgapdstic",
                    dataApiBase + string("/node-api/airport/") + code,
                    dataApiBase + string("/node-api/airport?code=") + code,
                    dataApiBase + string("/api/airport/") + code,
                    dataApiBase + string("/api/airport?code=") + code,
                    dataApiBase + string("/airport/") + code,
                    dataApiBase + string("/airport?code=") + code,
                };

                for (const auto& candidate : candidates)
                {
                    string resp;
                    // Use the shared fetch path for API endpoints as well.
                    if (!helper.fetchPage(candidate, resp))
                    {
                        continue;
                    }

                    if (tryParseAndCollect(resp, "departures") || tryParseAndCollect(resp, "arrivals"))
                    {
                        m_host->writeLog("AIRNAV|fetched API %s for airport[%s]", candidate.c_str(), airportIcao.c_str());
                        foundAny = true;
                    }
                    else
                    {
                        {
                            string key = string("AIRNAV_CAND_FAIL:") + (airportIcao.size() > 8 ? airportIcao.substr(0,8) : airportIcao);
                            if (rateLimitDetailedLog(key))
                            {
                                m_host->writeLog("AIRNAV|candidate[%s] fetched but parse failed for airport[%s] bodyTrunc[%s]", candidate.c_str(), airportIcao.c_str(), truncateForLog(resp, 512).c_str());
                            }
                            else
                            {
                                m_host->writeLog("AIRNAV|candidate[%s] fetched but parse failed for airport[%s]", candidate.c_str(), airportIcao.c_str());
                            }
                        }
                    }
                }
            }
        }

        m_host->writeLog("AIRNAV|airport[%s] loaded departures=[%d] arrivals=[%d]", airportIcao.c_str(), (int)departures.size(), (int)arrivals.size());
        return !departures.empty() || !arrivals.empty();
    }

private:
    static string getStringOrEmpty(const SimpleJson& node, initializer_list<string> path)
    {
        const SimpleJson* value = tryGetPath(node, vector<string>(path));
        if (value && !value->isNull())
        {
            string s = value->asString();
            if (!s.empty()) return s;
        }
        return "";
    }

    static long long getIntegerOrZero(const SimpleJson& node, initializer_list<string> path)
    {
        const SimpleJson* value = tryGetPath(node, vector<string>(path));
        if (value && !value->isNull())
        {
            long long v = value->asInteger(0);
            if (v != 0) return v;
        }
        return 0;
    }

    static string normalizeCode(string code)
    {
        code.erase(remove_if(code.begin(), code.end(), [](unsigned char c) {
            return isspace(c);
        }), code.end());

        transform(code.begin(), code.end(), code.begin(), [](unsigned char c) {
            return static_cast<char>(toupper(c));
        });

        return code;
    }

    static string extractAirNavRadarFlightNumber(const SimpleJson& node)
    {
        // Primary: fnia (flight number with airline prefix, e.g., "IB1625")
        string fnia = getStringOrEmpty(node, { "fnia" });
        if (!fnia.empty())
        {
            return normalizeCode(fnia);
        }

        // Secondary: fnic (flight number code)
        string fnic = getStringOrEmpty(node, { "fnic" });
        if (!fnic.empty())
        {
            return normalizeCode(fnic);
        }

        // Tertiary: fnumbers array or string
        if (const SimpleJson* fn = node.tryGet("fnumbers"))
        {
            if (fn->isArray() && !fn->arrayValue().empty())
            {
                const SimpleJson& first = fn->arrayValue().front();
                if (!first.isNull())
                {
                    string value = first.asString();
                    if (!value.empty()) return normalizeCode(value);
                }
            }
            else if (!fn->isNull())
            {
                string value = fn->asString();
                if (!value.empty()) return normalizeCode(value);
            }
        }

        // Fallback: aircraft registration
        string registration = getStringOrEmpty(node, { "acr" });
        if (!registration.empty())
        {
            return normalizeCode(registration);
        }

        return "";
    }

    static bool tryParseAirNavRadarDateTime(const string& dateText, const string& timeText, time_t& out)
    {
        out = 0;
        if (dateText.empty() || timeText.empty())
        {
            return false;
        }

        auto trim = [](const string& value) {
            const string whitespace = " \t\r\n";
            const auto begin = value.find_first_not_of(whitespace);
            if (begin == string::npos) return string();
            const auto end = value.find_last_not_of(whitespace);
            return value.substr(begin, end - begin + 1);
        };

        string date = trim(dateText);
        const auto commaPos = date.find(',');
        if (commaPos != string::npos)
        {
            date = trim(date.substr(commaPos + 1));
        }

        vector<string> parts;
        string current;
        for (char c : date)
        {
            if (isspace(static_cast<unsigned char>(c)))
            {
                if (!current.empty())
                {
                    parts.push_back(current);
                    current.clear();
                }
            }
            else
            {
                current.push_back(c);
            }
        }
        if (!current.empty())
        {
            parts.push_back(current);
        }

        if (parts.size() < 3)
        {
            return false;
        }

        static const vector<pair<string, int>> months = {
            { "January", 0 }, { "February", 1 }, { "March", 2 }, { "April", 3 },
            { "May", 4 }, { "June", 5 }, { "July", 6 }, { "August", 7 },
            { "September", 8 }, { "October", 9 }, { "November", 10 }, { "December", 11 }
        };

        int month = -1;
        for (const auto& kv : months)
        {
            if (kv.first == parts[0])
            {
                month = kv.second;
                break;
            }
        }

        if (month < 0)
        {
            return false;
        }

        int day = 0;
        int year = 0;
        try
        {
            day = stoi(parts[1]);
            year = stoi(parts[2]);
        }
        catch (...) { return false; }

        int hour = 0;
        int minute = 0;
        if (sscanf(timeText.c_str(), "%d:%d", &hour, &minute) != 2)
        {
            return false;
        }

        tm tmValue = {};
        tmValue.tm_year = year - 1900;
        tmValue.tm_mon = month;
        tmValue.tm_mday = day;
        tmValue.tm_hour = hour;
        tmValue.tm_min = minute;
        tmValue.tm_sec = 0;
        tmValue.tm_isdst = 0;

        out = timegm(&tmValue);
        return out != -1;
    }

    static time_t extractAirNavRadarScheduleTime(const SimpleJson& node, bool isDeparture)
    {
        // Prefer Unix timestamp fields (most reliable)
        const string tsField = isDeparture ? "depsu" : "arrsu";
        long long ts = getIntegerOrZero(node, { tsField });
        if (ts > 0)
        {
            return static_cast<time_t>(ts);
        }

        // Fallback to date/time string parsing
        const string dateField = isDeparture ? "depdate_utc" : "arrdate_utc";
        const string timeField = isDeparture ? "deps_utc" : "arrs_utc";
        string date = getStringOrEmpty(node, { dateField });
        string time = getStringOrEmpty(node, { timeField });

        if (time.empty())
        {
            time = getStringOrEmpty(node, { isDeparture ? "deps" : "arrs" });
        }

        time_t result = 0;
        if (tryParseAirNavRadarDateTime(date, time, result))
        {
            return result;
        }

        return 0;
    }

    static bool tryLoadAirportSchedulesFromSearchEndpoint(
        Fr24AirportScheduleSource& helper,
        const string& code,
        vector<Fr24ScheduleEntry>& departures,
        vector<Fr24ScheduleEntry>& arrivals)
    {
        const vector<string> searchOrigins = {
            "https://www.airnavradar.com",
            "https://live.airnavradar.com",
        };

        auto fetchAndCollectMode = [&](const string& modeKey) -> bool {
            const size_t departuresBefore = departures.size();
            const size_t arrivalsBefore = arrivals.size();

            for (const auto& origin : searchOrigins)
            {
                string url = origin + string("/data/airports/search/") + code + "?key=" + modeKey;
                string page;

                // Use the shared fetch path so daemon/curl fallback can still help.
                if (!helper.fetchPage(url, page))
                {
                    continue;
                }

                string payload = page;
                if (!Fr24AirportScheduleSource::extractJsonPayloadFromText(payload))
                {
                    continue;
                }

                try
                {
                    SimpleJson root = SimpleJson::parse(payload);

                    auto collectFromList = [&](const SimpleJson& listNode) {
                        for (const auto& item : listNode.arrayValue())
                        {
                            if (!item.isObject())
                            {
                                continue;
                            }

                            const string originIcao = normalizeCode(getStringOrEmpty(item, { "aporgic" }));
                            const string destinationIcao = normalizeCode(getStringOrEmpty(item, { "apdstic" }));
                            const bool isDeparture = originIcao == code;
                            const bool isArrival = destinationIcao == code;
                            if (!isDeparture && !isArrival)
                            {
                                continue;
                            }

                            const string flightNumber = extractAirNavRadarFlightNumber(item);
                            if (flightNumber.empty())
                            {
                                continue;
                            }

                            Fr24ScheduleEntry entry;
                            entry.flightNumber = flightNumber;
                            entry.callsign = flightNumber;
                            entry.airlineIcao = normalizeCode(getStringOrEmpty(item, { "alic" }));
                            entry.originIcao = originIcao;
                            entry.destinationIcao = destinationIcao;
                            entry.scheduledTime = extractAirNavRadarScheduleTime(item, isDeparture);

                            if (isDeparture)
                            {
                                departures.push_back(entry);
                            }
                            else
                            {
                                arrivals.push_back(entry);
                            }
                        }
                    };

                    if (root.isArray())
                    {
                        for (const auto& chunk : root.arrayValue())
                        {
                            if (!chunk.isObject())
                            {
                                continue;
                            }

                            const SimpleJson* listNode = chunk.tryGet("list");
                            if (!listNode || !listNode->isArray())
                            {
                                continue;
                            }

                            collectFromList(*listNode);
                        }
                    }
                    else if (root.isObject())
                    {
                        const SimpleJson* listNode = root.tryGet("list");
                        if (listNode && listNode->isArray())
                        {
                            collectFromList(*listNode);
                        }
                    }

                    if (departures.size() > departuresBefore || arrivals.size() > arrivalsBefore)
                    {
                        return true;
                    }
                }
                catch (...) {
                    continue;
                }
            }

            return false;
        };

        bool foundAnyList = false;
        foundAnyList = fetchAndCollectMode("mrgaporgic") || foundAnyList;
        foundAnyList = fetchAndCollectMode("mrgapdstic") || foundAnyList;
        return foundAnyList;
    }

    static const SimpleJson* tryGetPath(const SimpleJson& node, const vector<string>& path)
    {
        const SimpleJson* current = &node;
        for (const auto& key : path)
        {
            if (!current || !current->isObject())
            {
                return nullptr;
            }
            current = current->tryGet(key);
        }
        return current;
    }
};
