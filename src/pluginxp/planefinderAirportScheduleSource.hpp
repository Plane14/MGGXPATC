// Lightweight adapter for planefinder.net airport schedules.
// The page renders the visible schedule cards directly, so we parse the DOM
// structure instead of guessing at hidden JSON endpoints.
#pragma once

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <ctime>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "fr24AirportScheduleSource.hpp"

using namespace std;

class PlanefinderAirportScheduleSource
{
private:
    shared_ptr<HostServices> m_host;

public:
    explicit PlanefinderAirportScheduleSource(shared_ptr<HostServices> host) : m_host(std::move(host)) {}

    bool tryLoadAirportSchedules(const string& airportIcao, vector<Fr24ScheduleEntry>& departures, vector<Fr24ScheduleEntry>& arrivals)
    {
        return tryLoadAirportSchedules(airportIcao, "", departures, arrivals);
    }

    bool tryLoadAirportSchedules(const string& airportIcao, const string& airportIata, vector<Fr24ScheduleEntry>& departures, vector<Fr24ScheduleEntry>& arrivals)
    {
        departures.clear();
        arrivals.clear();

        Fr24AirportScheduleSource helper(m_host);

        vector<string> candidateCodes;
        const auto addCandidate = [&](const string& rawCode) {
            string code = rawCode;
            code.erase(remove_if(code.begin(), code.end(), [](unsigned char c) {
                return isspace(c);
            }), code.end());
            transform(code.begin(), code.end(), code.begin(), [](unsigned char c) {
                return static_cast<char>(toupper(c));
            });
            if (code.empty())
            {
                return;
            }
            if (find(candidateCodes.begin(), candidateCodes.end(), code) == candidateCodes.end())
            {
                candidateCodes.push_back(code);
            }
        };

        addCandidate(airportIata);
        addCandidate(airportIcao);
        if (airportIata.empty() && airportIcao.size() == 4)
        {
            addCandidate(airportIcao.substr(1));
        }

        for (const auto& code : candidateCodes)
        {
            const string url = string("https://planefinder.net/airport/") + code;
            string page;

            m_host->writeLog("PLF|fetching airport[%s] code[%s] with shared fetch", airportIcao.c_str(), code.c_str());
            if (!helper.fetchPage(url, page))
            {
                continue;
            }

            if (containsIgnoreCase(page, "sorry, you have been blocked") || containsIgnoreCase(page, "just a moment"))
            {
                m_host->writeLog("PLF|airport[%s] code[%s] fetch returned a challenge page", airportIcao.c_str(), code.c_str());
                continue;
            }

            if (page.size() < 256 || !containsIgnoreCase(page, "class=\"airport-card"))
            {
                m_host->writeLog(
                    "PLF|airport[%s] code[%s] fetch returned a suspiciously small/non-card page pageLen[%zu]",
                    airportIcao.c_str(),
                    code.c_str(),
                    page.size());
                continue;
            }

            const string pageIata = extractAirportIataFromOpts(page);
            const string selectedTab = extractSelectedTabLabel(page);
            const bool arrivalsTab = containsIgnoreCase(selectedTab, "arrival");

            vector<Fr24ScheduleEntry> parsedEntries;
            const size_t parsedCards = parseAirportCards(page, parsedEntries);
            if (parsedCards == 0)
            {
                m_host->writeLog(
                    "PLF|airport[%s] code[%s] pageIATA[%s] tab[%s] had no parseable cards pageLen[%zu]",
                    airportIcao.c_str(),
                    code.c_str(),
                    pageIata.c_str(),
                    selectedTab.empty() ? "unknown" : selectedTab.c_str(),
                    page.size());
                continue;
            }

            sortEntries(parsedEntries);
            if (arrivalsTab)
            {
                arrivals = std::move(parsedEntries);
            }
            else
            {
                departures = std::move(parsedEntries);
            }

            m_host->writeLog(
                "PLF|airport[%s] code[%s] pageIATA[%s] tab[%s] cards[%zu] departures=[%d] arrivals=[%d]",
                airportIcao.c_str(),
                code.c_str(),
                pageIata.c_str(),
                selectedTab.empty() ? "unknown" : selectedTab.c_str(),
                parsedCards,
                (int)departures.size(),
                (int)arrivals.size());

            return !departures.empty() || !arrivals.empty();
        }

        m_host->writeLog("PLF|fetch failed for airport[%s] after trying shared fetch candidates", airportIcao.c_str());
        return false;
    }

private:
    static string trimCopy(string value)
    {
        const auto first = find_if(value.begin(), value.end(), [](unsigned char c) {
            return !isspace(c);
        });
        if (first == value.end())
        {
            return "";
        }

        const auto last = find_if(value.rbegin(), value.rend(), [](unsigned char c) {
            return !isspace(c);
        }).base();

        return string(first, last);
    }

    static string toLowerCopy(string value)
    {
        transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
            return static_cast<char>(tolower(c));
        });
        return value;
    }

    static bool containsIgnoreCase(const string& text, const string& needle)
    {
        if (text.empty() || needle.empty())
        {
            return false;
        }

        return toLowerCopy(text).find(toLowerCopy(needle)) != string::npos;
    }

    static void replaceAll(string& value, const string& from, const string& to)
    {
        if (from.empty())
        {
            return;
        }

        size_t pos = 0;
        while ((pos = value.find(from, pos)) != string::npos)
        {
            value.replace(pos, from.length(), to);
            pos += to.length();
        }
    }

    static string htmlEntityDecodeBasic(string value)
    {
        replaceAll(value, "&quot;", "\"");
        replaceAll(value, "&#34;", "\"");
        replaceAll(value, "&#x22;", "\"");
        replaceAll(value, "&#39;", "'");
        replaceAll(value, "&#x27;", "'");
        replaceAll(value, "&lt;", "<");
        replaceAll(value, "&gt;", ">");
        replaceAll(value, "&amp;", "&");
        replaceAll(value, "&nbsp;", " ");
        replaceAll(value, "&#160;", " ");
        return value;
    }

    static string stripTagsCopy(string value)
    {
        string stripped;
        stripped.reserve(value.size());

        bool inTag = false;
        for (char ch : value)
        {
            if (ch == '<')
            {
                inTag = true;
                continue;
            }

            if (ch == '>')
            {
                inTag = false;
                continue;
            }

            if (!inTag)
            {
                stripped.push_back(ch);
            }
        }

        return trimCopy(htmlEntityDecodeBasic(stripped));
    }

    static string extractTextBetween(const string& text, const string& startMarker, const string& endMarker, size_t searchFrom = 0)
    {
        const size_t start = text.find(startMarker, searchFrom);
        if (start == string::npos)
        {
            return "";
        }

        const size_t contentStart = start + startMarker.size();
        const size_t contentEnd = text.find(endMarker, contentStart);
        if (contentEnd == string::npos || contentEnd < contentStart)
        {
            return "";
        }

        return stripTagsCopy(text.substr(contentStart, contentEnd - contentStart));
    }

    static string extractCardValueAfterLabel(const string& block, const string& labelText)
    {
        const size_t labelPos = block.find(labelText);
        if (labelPos == string::npos)
        {
            return "";
        }

        const string valueMarker = "class=\"card-value\">";
        size_t valuePos = block.find(valueMarker, labelPos);
        if (valuePos == string::npos)
        {
            return "";
        }

        valuePos += valueMarker.size();
        size_t end = block.find("</span>", valuePos);
        if (end == string::npos)
        {
            end = block.find('<', valuePos);
        }
        if (end == string::npos)
        {
            end = block.size();
        }

        return stripTagsCopy(block.substr(valuePos, end - valuePos));
    }

    static bool tryParseClockTimeToUtcDay(const string& text, time_t& resolvedTime)
    {
        const string trimmed = trimCopy(text);
        int hour = -1;
        int minute = -1;
        if (sscanf(trimmed.c_str(), "%d:%d", &hour, &minute) != 2)
        {
            return false;
        }

        if (hour < 0 || hour > 23 || minute < 0 || minute > 59)
        {
            return false;
        }

        time_t now = time(nullptr);
        tm utcNow{};
#if IBM
        if (gmtime_s(&utcNow, &now) != 0)
        {
            return false;
        }
#else
        if (!gmtime_r(&now, &utcNow))
        {
            return false;
        }
#endif

        utcNow.tm_hour = hour;
        utcNow.tm_min = minute;
        utcNow.tm_sec = 0;

        time_t candidate = timegm(&utcNow);
        if (candidate == static_cast<time_t>(-1))
        {
            return false;
        }

        const time_t twelveHours = 12 * 60 * 60;
        if (now - candidate > twelveHours)
        {
            candidate += 24 * 60 * 60;
        }

        resolvedTime = candidate;
        return true;
    }

    static bool shouldSkipStatus(const string& statusText)
    {
        const string normalized = toLowerCopy(statusText);
        return (
            normalized.find("cancel") != string::npos ||
            normalized.find("divert") != string::npos ||
            normalized.find("arrived") != string::npos ||
            normalized.find("departed") != string::npos ||
            normalized.find("landed") != string::npos);
    }

    static string extractAirportIataFromOpts(const string& page)
    {
        const string marker = "var opts = ";
        const size_t start = page.find(marker);
        if (start == string::npos)
        {
            return "";
        }

        const size_t jsonStart = page.find('{', start);
        if (jsonStart == string::npos)
        {
            return "";
        }

        int depth = 0;
        size_t jsonEnd = string::npos;
        for (size_t i = jsonStart; i < page.size(); ++i)
        {
            if (page[i] == '{')
            {
                ++depth;
            }
            else if (page[i] == '}')
            {
                --depth;
                if (depth == 0)
                {
                    jsonEnd = i + 1;
                    break;
                }
            }
        }

        if (jsonEnd == string::npos)
        {
            return "";
        }

        try
        {
            SimpleJson opts = SimpleJson::parse(page.substr(jsonStart, jsonEnd - jsonStart));
            const SimpleJson* airportIata = opts.tryGet("airportIATA");
            if (airportIata && !airportIata->isNull())
            {
                return trimCopy(airportIata->asString());
            }
        }
        catch (...)
        {
        }

        return "";
    }

    static string extractSelectedTabLabel(const string& page)
    {
        const string marker = "class=\"selected tab\">";
        const size_t start = page.find(marker);
        if (start == string::npos)
        {
            return "";
        }

        const size_t contentStart = start + marker.size();
        const size_t contentEnd = page.find("<div", contentStart);
        if (contentEnd == string::npos || contentEnd < contentStart)
        {
            return "";
        }

        return stripTagsCopy(page.substr(contentStart, contentEnd - contentStart));
    }

    static bool parseAirportCard(const string& cardBlock, Fr24ScheduleEntry& entry)
    {
        const string flightNumber = extractCardValueAfterLabel(cardBlock, "Flight No.");
        if (flightNumber.empty())
        {
            return false;
        }

        const string statusText = extractTextBetween(cardBlock, "class=\"flight-status\">", "</div>");
        if (shouldSkipStatus(statusText))
        {
            return false;
        }

        const string scheduledText = extractCardValueAfterLabel(cardBlock, "Scheduled");
        const string actualText = extractTextBetween(cardBlock, "class=\"time-actual\">", "</div>");

        entry = Fr24ScheduleEntry{};
        entry.flightNumber = flightNumber;
        entry.callsign = flightNumber;

        time_t resolvedTime = 0;
        if (!scheduledText.empty())
        {
            tryParseClockTimeToUtcDay(scheduledText, resolvedTime);
        }
        if (resolvedTime == 0 && !actualText.empty())
        {
            tryParseClockTimeToUtcDay(actualText, resolvedTime);
        }

        entry.scheduledTime = resolvedTime;
        return true;
    }

    static void sortEntries(vector<Fr24ScheduleEntry>& entries)
    {
        sort(entries.begin(), entries.end(), [](const Fr24ScheduleEntry& a, const Fr24ScheduleEntry& b) {
            if (a.scheduledTime != b.scheduledTime)
            {
                if (a.scheduledTime == 0)
                {
                    return false;
                }
                if (b.scheduledTime == 0)
                {
                    return true;
                }
                return a.scheduledTime < b.scheduledTime;
            }

            if (a.flightNumber != b.flightNumber)
            {
                return a.flightNumber < b.flightNumber;
            }

            if (a.callsign != b.callsign)
            {
                return a.callsign < b.callsign;
            }

            return a.airlineIcao < b.airlineIcao;
        });
    }

    static size_t parseAirportCards(const string& page, vector<Fr24ScheduleEntry>& entries)
    {
        const string cardMarker = "class=\"airport-card";
        size_t parsedCards = 0;

        size_t searchPos = page.find("class=\"card-container\"");
        if (searchPos == string::npos)
        {
            searchPos = 0;
        }

        while (true)
        {
            const size_t cardStart = page.find(cardMarker, searchPos);
            if (cardStart == string::npos)
            {
                break;
            }

            const size_t nextCardStart = page.find(cardMarker, cardStart + cardMarker.size());
            const string cardBlock = (nextCardStart == string::npos)
                ? page.substr(cardStart)
                : page.substr(cardStart, nextCardStart - cardStart);

            Fr24ScheduleEntry entry;
            if (parseAirportCard(cardBlock, entry))
            {
                entries.push_back(std::move(entry));
                ++parsedCards;
            }

            if (nextCardStart == string::npos)
            {
                break;
            }

            searchPos = nextCardStart;
        }

        return parsedCards;
    }
};