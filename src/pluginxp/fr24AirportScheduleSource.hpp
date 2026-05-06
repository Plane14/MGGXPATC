//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <thread>
#include <regex>

#include "libworld.h"
#include "airlineReferenceTable.hpp"
#include "aircraftTypeReferenceTable.hpp"
#include "simpleJson.hpp"

using namespace std;
using namespace world;

static string truncateForLog(const string& text, size_t maxLen = 256)
{
    if (text.size() <= maxLen) return text;
    return text.substr(0, maxLen) + string("... [truncated ") + to_string(text.size() - maxLen) + string(" bytes]");
}

static bool rateLimitDetailedLog(const string& key, int seconds = 60)
{
    static unordered_map<string, chrono::steady_clock::time_point> s_last;
    const auto now = chrono::steady_clock::now();
    auto it = s_last.find(key);
    if (it == s_last.end())
    {
        s_last[key] = now;
        return true;
    }

    const auto elapsed = chrono::duration_cast<chrono::seconds>(now - it->second).count();
    if (elapsed >= seconds)
    {
        it->second = now;
        return true;
    }
    return false;
}

static string stripOuterDoubleQuotesCopy(string value)
{
    auto first = find_if(value.begin(), value.end(), [](unsigned char c) {
        return !isspace(c);
    });
    if (first == value.end())
    {
        return "";
    }

    auto last = find_if(value.rbegin(), value.rend(), [](unsigned char c) {
        return !isspace(c);
    }).base();

    value = string(first, last);
    while (value.size() >= 2 && value.front() == '"' && value.back() == '"')
    {
        value = value.substr(1, value.size() - 2);
    }
    return value;
}

static string quoteCommandArgument(const string& value)
{
    const string cleaned = stripOuterDoubleQuotesCopy(value);
    string quoted = "\"";
    for (const char c : cleaned)
    {
        if (c == '"')
        {
            quoted += "\\\"";
        }
        else
        {
            quoted += c;
        }
    }
    quoted += "\"";
    return quoted;
}

static string buildPythonScriptCommand(const string& pythonBinary, const string& scriptPath)
{
    return quoteCommandArgument(pythonBinary) + " " + quoteCommandArgument(scriptPath) + " 2>&1";
}

static bool fileExistsForRead(const string& path)
{
    if (path.empty())
    {
        return false;
    }

    ifstream file(path, ios::in | ios::binary);
    return file.good();
}

// Shared browser pool for Zendriver to avoid launching a new browser per request.
// This is a process-wide singleton that persists across multiple schedule source fetches.
class ZendriverBrowserPool
{
public:
    struct Session
    {
        string scriptPath;
        string pythonBinary;
        chrono::steady_clock::time_point lastUsed;
    };

private:
    inline static mutex s_mutex;
    inline static shared_ptr<ZendriverBrowserPool> s_instance;
    vector<Session> m_sessions;
    shared_ptr<HostServices> m_host;
    bool m_pythonChecked = false;
    string m_workingPython;

    ZendriverBrowserPool(shared_ptr<HostServices> host) : m_host(host) {}

public:
    static shared_ptr<ZendriverBrowserPool> getInstance(shared_ptr<HostServices> host)
    {
        lock_guard<mutex> lock(s_mutex);
        if (!s_instance)
        {
            s_instance = shared_ptr<ZendriverBrowserPool>(new ZendriverBrowserPool(host));
        }
        return s_instance;
    }

    static void resetInstance()
    {
        lock_guard<mutex> lock(s_mutex);
        s_instance.reset();
    }

    // Check which Python binary works with zendriver (cached after first check)
    string getWorkingPythonBinary()
    {
        if (m_pythonChecked)
        {
            return m_workingPython;
        }

        // Try common installation paths and PATH-based names
        vector<string> candidates = {
            "python", "python3", "py",
            "C:\\Python314\\python.exe",
            "C:\\Python313\\python.exe",
            "C:\\Python312\\python.exe",
            "C:\\Python311\\python.exe",
            "C:\\Python310\\python.exe",
            "C:\\Users\\%USERNAME%\\AppData\\Local\\Programs\\Python\\Python314\\python.exe",
            "C:\\Users\\%USERNAME%\\AppData\\Local\\Programs\\Python\\Python313\\python.exe",
            "C:\\Users\\%USERNAME%\\AppData\\Local\\Programs\\Python\\Python312\\python.exe",
            "C:\\Users\\%USERNAME%\\AppData\\Local\\Programs\\Python\\Python311\\python.exe",
            "C:\\Users\\%USERNAME%\\AppData\\Local\\Programs\\Python\\Python310\\python.exe",
            "C:\\Program Files\\Python314\\python.exe",
            "C:\\Program Files\\Python313\\python.exe",
            "C:\\Program Files\\Python312\\python.exe",
            "C:\\Program Files\\Python311\\python.exe",
            "C:\\Program Files\\Python310\\python.exe"
        };

        string allOutputs;
        bool anyPythonFound = false;

        for (auto py : candidates)
        {
            py = stripOuterDoubleQuotesCopy(py);

            // Expand %USERNAME% if present
            if (py.find("%USERNAME%") != string::npos)
            {
                const char* username = getenv("USERNAME");
                if (username)
                {
                    size_t pos = py.find("%USERNAME%");
                    py.replace(pos, 10, username);
                }
            }

            string testScript =
                "try:\n"
                "    import zendriver\n"
                "    print('OK')\n"
                "except Exception as e:\n"
                "    print('FAIL:', e)\n";

            string testPath;
            if (const char* tempDir = getenv("TEMP"))
            {
                testPath = string(tempDir) + "\\mggxpatc_zd_test.py";
            }
            else
            {
                testPath = "mggxpatc_zd_test.py";
            }

            {
                ofstream f(testPath, ios::out | ios::trunc | ios::binary);
                if (f) f << testScript;
            }

            string command = buildPythonScriptCommand(py, testPath);
            string output;
#if IBM
            FILE* pipe = _popen(command.c_str(), "r");
#else
            FILE* pipe = popen(command.c_str(), "r");
#endif
            if (pipe)
            {
                anyPythonFound = true;
                char buffer[1024];
                while (fgets(buffer, sizeof(buffer), pipe))
                {
                    output.append(buffer);
                }
#if IBM
                _pclose(pipe);
#else
                pclose(pipe);
#endif
            }
            else
            {
                allOutputs += "[" + py + "]:popen_failed;";
            }
            remove(testPath.c_str());

            if (!output.empty())
            {
                allOutputs += "[" + py + "]:" + output + ";";
            }

            if (output.find("OK") != string::npos)
            {
                m_workingPython = py;
                m_pythonChecked = true;
                m_host->writeLog("FR24|Browser pool using Python binary: %s", py.c_str());
                return m_workingPython;
            }
        }

        m_pythonChecked = true;
        m_host->writeLog("FR24|Browser pool failed to find working Python with zendriver. Tried: %s", allOutputs.c_str());
        if (!anyPythonFound)
        {
            m_host->writeLog("FR24|Browser pool: No Python executable found. Install Python, add to PATH, and run: pip install zendriver");
        }
        return "";
    }

    // Fetch URL using a persistent browser session (reuses browser process)
    bool fetchUrl(const string& url, string& responseText, int maxRetries = 2)
    {
        responseText.clear();
        string pythonBinary = getWorkingPythonBinary();
        if (pythonBinary.empty())
        {
            return false;
        }

        // Create script that fetches URL using an existing browser if possible
        const auto now = chrono::steady_clock::now().time_since_epoch().count();
        string scriptPath;
        if (const char* tempDir = getenv("TEMP"))
        {
            scriptPath = string(tempDir) + "\\mggxpatc_fr24_zd_" + to_string(static_cast<long long>(now)) + ".py";
        }
        else
        {
            scriptPath = "mggxpatc_fr24_zd_" + to_string(static_cast<long long>(now)) + ".py";
        }

        // Escape URL for Python triple-quoted string
        string escapedUrl = url;
        size_t pos = 0;
        while ((pos = escapedUrl.find("'''", pos)) != string::npos)
        {
            escapedUrl.replace(pos, 3, "\\'\\'\\'");
            pos += 6;
        }

        // Python script that uses Zendriver as a real browser context.  Direct
        // navigation to FR24's API can land on a Cloudflare challenge page, so
        // wait for the browser to settle and then fetch the URL from inside the
        // page context with the browser's cookies.
        const string script =
            "import asyncio\n"
            "import sys\n"
            "import html\n"
            "import re\n"
            "import json\n"
            "import time\n"
            "\n"
            "url = '''" + escapedUrl + "'''\n"
            "\n"
            "try:\n"
            "    import zendriver as zd\n"
            "    from zendriver import cdp\n"
            "except Exception as e:\n"
            "    sys.stderr.write('IMPORT_ERROR:' + str(e) + '\\n')\n"
            "    sys.exit(2)\n"
            "\n"
            "def looks_like_json(text):\n"
            "    stripped = (text or '').lstrip()\n"
            "    return stripped.startswith('{') or stripped.startswith('[')\n"
            "\n"
            "def is_cf_challenge(text):\n"
            "    lowered = (text or '').lower()\n"
            "    return ('just a moment' in lowered or '_cf_chl_opt' in lowered or\n"
            "            'enable javascript and cookies' in lowered or 'challenge-platform' in lowered)\n"
            "\n"
            "def content_text(content):\n"
            "    if not content:\n"
            "        return ''\n"
            "    m = re.search(r'<pre[^>]*>(.*?)</pre>', content, re.IGNORECASE | re.DOTALL)\n"
            "    if m:\n"
            "        return html.unescape(m.group(1)).strip()\n"
            "    return content.strip()\n"
            "\n"
            "async def navigate(page, target):\n"
            "    try:\n"
            "        await page.send(cdp.page.navigate(target))\n"
            "    except Exception:\n"
            "        try:\n"
            "            await asyncio.wait_for(page.get(target), timeout=20)\n"
            "        except Exception:\n"
            "            pass\n"
            "\n"
            "async def browser_fetch(page, target):\n"
            "    script = \"\"\"\n"
            "        (async (target) => {\n"
            "          const response = await fetch(target, {\n"
            "            method: 'GET',\n"
            "            credentials: 'include',\n"
            "            cache: 'no-store',\n"
            "            headers: {\n"
            "              'Accept': 'application/json,text/plain,*/*',\n"
            "              'X-Requested-With': 'XMLHttpRequest'\n"
            "            }\n"
            "          });\n"
            "          const text = await response.text();\n"
            "          return JSON.stringify({\n"
            "            status: response.status,\n"
            "            contentType: response.headers.get('content-type') || '',\n"
            "            text: text\n"
            "          });\n"
            "        })(%s)\n"
            "    \"\"\" % json.dumps(target)\n"
            "    result = await asyncio.wait_for(page.evaluate(script, await_promise=True, return_by_value=True), timeout=30)\n"
            "    if not isinstance(result, str):\n"
            "        return ''\n"
            "    data = json.loads(result)\n"
            "    return (data.get('text') or '').strip()\n"
            "\n"
            "async def main():\n"
            "    browser = None\n"
            "    try:\n"
            "        try:\n"
            "            import asyncio as _asyncio\n"
            "            import sys as _sys\n"
            "            if _sys.platform == 'win32':\n"
            "                try:\n"
            "                    _asyncio.set_event_loop_policy(_asyncio.WindowsSelectorEventLoopPolicy())\n"
            "                except Exception:\n"
            "                    pass\n"
            "        except Exception:\n"
            "            pass\n"
            "\n"
            "        browser = await zd.start()\n"
            "        try:\n"
            "            page = browser.tabs[0]\n"
            "        except Exception:\n"
            "            page = await browser.get('about:blank')\n"
            "\n"
            "        await navigate(page, url)\n"
            "        last_text = ''\n"
            "        deadline = time.monotonic() + 55\n"
            "        while time.monotonic() < deadline:\n"
            "            try:\n"
            "                try:\n"
            "                    await page.wait_for_ready_state('complete', timeout=5)\n"
            "                except Exception:\n"
            "                    pass\n"
            "\n"
            "                text = content_text(await page.get_content())\n"
            "                if text:\n"
            "                    last_text = text\n"
            "                if looks_like_json(text):\n"
            "                    sys.stdout.write(text)\n"
            "                    return\n"
            "\n"
            "                if is_cf_challenge(text):\n"
            "                    try:\n"
            "                        await asyncio.wait_for(page.verify_cf(timeout=10), timeout=15)\n"
            "                    except Exception:\n"
            "                        pass\n"
            "                    await asyncio.sleep(1)\n"
            "                    continue\n"
            "\n"
            "                fetched = await browser_fetch(page, url)\n"
            "                if fetched:\n"
            "                    sys.stdout.write(fetched)\n"
            "                    return\n"
            "            except Exception:\n"
            "                await asyncio.sleep(1)\n"
            "\n"
            "        try:\n"
            "            fetched = await browser_fetch(page, url)\n"
            "            if fetched:\n"
            "                sys.stdout.write(fetched)\n"
            "                return\n"
            "        except Exception:\n"
            "            pass\n"
            "\n"
            "        if not last_text:\n"
            "            sys.stderr.write('EMPTY_RESPONSE\\n')\n"
            "            sys.exit(3)\n"
            "\n"
            "        sys.stdout.write(last_text)\n"
            "    except Exception as e:\n"
            "        sys.stderr.write('FETCH_ERROR:' + str(e) + '\\n')\n"
            "        sys.exit(1)\n"
            "    finally:\n"
            "        try:\n"
            "            if browser:\n"
            "                await asyncio.wait_for(browser.stop(), timeout=10)\n"
            "        except Exception:\n"
            "            pass\n"
            "\n"
            "if __name__ == '__main__':\n"
            "    asyncio.run(main())\n";

        {
            ofstream scriptFile(scriptPath, ios::out | ios::trunc | ios::binary);
            if (!scriptFile)
            {
                return false;
            }
            scriptFile << script;
        }

        string command = buildPythonScriptCommand(pythonBinary, scriptPath);

#if IBM
        FILE* pipe = _popen(command.c_str(), "r");
#else
        FILE* pipe = popen(command.c_str(), "r");
#endif
        if (!pipe)
        {
            remove(scriptPath.c_str());
            m_host->writeLog("FR24|ZENDRIVER|popen failed for command[%s]", command.c_str());
            return false;
        }

        char buffer[4096];
        responseText.clear();
        while (fgets(buffer, sizeof(buffer), pipe))
        {
            responseText.append(buffer);
        }

#if IBM
        int exitCode = _pclose(pipe);
#else
        int exitCode = pclose(pipe);
#endif
        remove(scriptPath.c_str());

        bool ok = (exitCode == 0 && !responseText.empty());
        if (!ok)
        {
            string truncated = truncateForLog(responseText, 1024);
            m_host->writeLog("FR24|ZENDRIVER|command exitCode[%d] url[%s] output[%s]", exitCode, url.c_str(), truncated.c_str());
        }

        return ok;
    }
};

struct Fr24ScheduleEntry
{
    string airlineIcao;
    string flightNumber;
    string callsign;
    string aircraftIcao;
    string originIcao;
    string destinationIcao;
    bool diverted = false;
    string divertedDestinationIcao;
    time_t scheduledTime = 0;
};

class Fr24AirportScheduleSource
{
private:
    shared_ptr<HostServices> m_host;

public:
    explicit Fr24AirportScheduleSource(shared_ptr<HostServices> host) : m_host(std::move(host))
    {
    }

    // Public helpers for other schedule sources that also want to use
    // Zendriver-based fetching and JSON extraction.
    bool fetchPageWithZendriver(const string& url, string& responseText)
    {
        return fetchUrlTextWithZendriver(url, responseText);
    }

    // Safe fetch that attempts Zendriver and falls back to curl when needed.
    bool fetchPage(const string& url, string& responseText)
    {
        return fetchUrlText(url, responseText);
    }

    static bool extractJsonPayloadFromText(string& text)
    {
        return extractJsonPayload(text);
    }

    bool tryLoadAirportSchedules(
        const string& airportIcao,
        vector<Fr24ScheduleEntry>& departures,
        vector<Fr24ScheduleEntry>& arrivals)
    {
        departures.clear();
        arrivals.clear();

        bool loadedDepartures = loadAirportScheduleMode(airportIcao, "departures", departures);
        bool loadedArrivals = loadAirportScheduleMode(airportIcao, "arrivals", arrivals);

        if (loadedDepartures)
        {
            sortEntries(departures);
        }
        if (loadedArrivals)
        {
            sortEntries(arrivals);
        }

        return loadedDepartures || loadedArrivals;
    }

private:
    static void sortEntries(vector<Fr24ScheduleEntry>& entries)
    {
        sort(entries.begin(), entries.end(), [](const Fr24ScheduleEntry& a, const Fr24ScheduleEntry& b) {
            if (a.scheduledTime != b.scheduledTime)
            {
                if (a.scheduledTime == 0) return false;
                if (b.scheduledTime == 0) return true;
                return a.scheduledTime < b.scheduledTime;
            }

            if (a.airlineIcao != b.airlineIcao)
            {
                return a.airlineIcao < b.airlineIcao;
            }

            return a.flightNumber < b.flightNumber;
        });
    }

    bool loadAirportScheduleMode(const string& airportIcao, const string& mode, vector<Fr24ScheduleEntry>& entries)
    {
        const string normalizedAirportIcao = normalizeCode(airportIcao);
        string url = buildAirportScheduleUrl(normalizedAirportIcao, mode);
        string responseText;

        if (!fetchUrlText(url, responseText))
        {
            m_host->writeLog("FR24|schedule[%s] airport[%s] fetch failed", mode.c_str(), normalizedAirportIcao.c_str());
            return false;
        }

        try
        {
            SimpleJson root = SimpleJson::parse(responseText);
            const SimpleJson* scheduleData = tryGetPath(root, {
                "result",
                "response",
                "airport",
                "pluginData",
                "schedule",
                mode,
                "data"
            });

            if (!scheduleData || !scheduleData->isArray())
            {
                m_host->writeLog("FR24|schedule[%s] airport[%s] did not contain a data array", mode.c_str(), normalizedAirportIcao.c_str());
                return false;
            }

            collectEntries(*scheduleData, normalizedAirportIcao, mode, entries);

            m_host->writeLog(
                "FR24|schedule[%s] airport[%s] loaded [%d] entries",
                mode.c_str(),
                normalizedAirportIcao.c_str(),
                (int)entries.size());
            return !entries.empty();
        }
        catch(const exception& e)
        {
            m_host->writeLog(
                "FR24|schedule[%s] airport[%s] parse failed: %s",
                mode.c_str(),
                normalizedAirportIcao.c_str(),
                e.what());
            return false;
        }
    }

    static string buildAirportScheduleUrl(const string& airportIcao, const string& mode)
    {
        string url = "https://api.flightradar24.com/common/v1/airport.json?code=";
        url += normalizeCode(airportIcao);
        url += "&limit=100&page=1&plugin%5B%5D=&plugin-setting%5Bschedule%5D%5Bmode%5D=";
        url += mode;

        const time_t now = time(nullptr);
        if (now > 0)
        {
            const long long dayNoonUtc = (static_cast<long long>(now) / 86400LL) * 86400LL + 43200LL;
            url += "&plugin-setting%5Bschedule%5D%5Btimestamp%5D=";
            url += to_string(dayNoonUtc);
        }

        return url;
    }

    string findZendriverDaemonScriptPath()
    {
        vector<string> candidates = {
            "tools\\zendriver_daemon.py",
            "tools/zendriver_daemon.py"
        };

        if (m_host)
        {
            candidates.push_back(m_host->getResourceFilePath({ "tools", "zendriver_daemon.py" }));
            candidates.push_back(m_host->getResourceFilePath({ "Resources", "tools", "zendriver_daemon.py" }));
            candidates.push_back(m_host->getHostFilePath({ "tools", "zendriver_daemon.py" }));
            candidates.push_back(m_host->getHostFilePath({ "Resources", "plugins", "airTrafficAndControl", "tools", "zendriver_daemon.py" }));
        }

        set<string> seen;
        string attempted;
        for (const string& candidate : candidates)
        {
            if (candidate.empty() || seen.find(candidate) != seen.end())
            {
                continue;
            }
            seen.insert(candidate);
            if (!attempted.empty())
            {
                attempted += ";";
            }
            attempted += candidate;

            if (fileExistsForRead(candidate))
            {
                return candidate;
            }
        }

        if (rateLimitDetailedLog("ZENDRIVER_DAEMON_SCRIPT_MISSING", 300))
        {
            m_host->writeLog("FR24|ZENDRIVER_DAEMON|script not found; tried[%s]", truncateForLog(attempted, 768).c_str());
        }
        return "";
    }

    bool fetchUrlText(const string& url, string& responseText)
    {
        responseText.clear();

        // First, try a local persistent Zendriver daemon (faster than spawning python per-request)
        const int daemonPort = 37337;

        auto isDaemonUp = [&](int timeoutMs = 2000) -> bool {
            const string healthCmd = string("curl -sS -m ") + to_string(max(1, timeoutMs/1000)) + " -o NUL -w \"%{http_code}\" \"http://127.0.0.1:" + to_string(daemonPort) + "/health\" 2>&1";

#if IBM
            FILE* pipe = _popen(healthCmd.c_str(), "r");
#else
            FILE* pipe = popen(healthCmd.c_str(), "r");
#endif
            if (!pipe) return false;
            char buf[128];
            string out;
            while (fgets(buf, sizeof(buf), pipe)) out.append(buf);
#if IBM
            _pclose(pipe);
#else
            pclose(pipe);
#endif
            // parse any leading digits
            auto firstDigit = out.find_first_of("0123456789");
            if (firstDigit == string::npos) return false;
            try { int code = stoi(out); return code == 200; } catch(...) { return false; }
        };

        auto tryDaemonFetch = [&](string& out) -> bool {
            const string cmd = string("curl -g -sS -G --data-urlencode \"url=") + url + "\" \"http://127.0.0.1:" + to_string(daemonPort) + "/fetch\" 2>&1";

#if IBM
            FILE* pipe = _popen(cmd.c_str(), "r");
#else
            FILE* pipe = popen(cmd.c_str(), "r");
#endif
            if (!pipe) return false;
            char buffer[4096];
            string responseCombined;
            while (fgets(buffer, sizeof(buffer), pipe))
            {
                responseCombined.append(buffer);
            }

#if IBM
            const int exitCode = _pclose(pipe);
#else
            const int exitCode = pclose(pipe);
#endif
            if (exitCode != 0 || responseCombined.empty()) return false;
            if (responseCombined.rfind("ERROR:", 0) == 0) return false;
            out = responseCombined;
            return true;
        };

        if (!isDaemonUp())
        {
            auto pool = ZendriverBrowserPool::getInstance(m_host);
            string pythonBinary = pool ? pool->getWorkingPythonBinary() : string();
            if (!pythonBinary.empty())
            {
                string scriptPath = findZendriverDaemonScriptPath();
                if (!scriptPath.empty())
                {
#ifdef _WIN32
                    string launch = "cmd /c start \"\" " + quoteCommandArgument(pythonBinary) + " " + quoteCommandArgument(scriptPath) + " --port " + to_string(daemonPort);
#else
                    string launch = quoteCommandArgument(pythonBinary) + " " + quoteCommandArgument(scriptPath) + " --port " + to_string(daemonPort) + " &";
#endif
                    // Detached launch; start should return quickly.
                    int launchResult = system(launch.c_str());
                    if (launchResult != 0 && rateLimitDetailedLog("ZENDRIVER_DAEMON_LAUNCH_FAIL", 60))
                    {
                        m_host->writeLog("FR24|ZENDRIVER_DAEMON|launch command failed result[%d] script[%s]", launchResult, scriptPath.c_str());
                    }
                }

                // Wait briefly for the daemon to appear
                for (int i = 0; i < 20; ++i)
                {
                    if (isDaemonUp(500)) break;
                    this_thread::sleep_for(chrono::milliseconds(250));
                }
            }
        }

        string daemonResp;
        if (isDaemonUp() && tryDaemonFetch(daemonResp))
        {
            string tmp = daemonResp;
            if (extractJsonPayload(tmp))
            {
                responseText = tmp;
                return true;
            }
            else
            {
                string key = string("ZENDRIVER_DAEMON_NONJSON:") + (url.size() > 128 ? url.substr(0, 128) : url);
                if (rateLimitDetailedLog(key))
                {
                    m_host->writeLog("FR24|ZENDRIVER_DAEMON|fetched non-JSON response for url[%s] truncated[%s]", url.c_str(), truncateForLog(daemonResp, 256).c_str());
                }
                else
                {
                    m_host->writeLog("FR24|ZENDRIVER_DAEMON|non-JSON url[%s]", truncateForLog(url, 128).c_str());
                }
            }
        }

        // Prefer Zendriver for FR24 to reduce anti-bot blocks (per-request zendriver fallback)
        if (fetchUrlTextWithZendriver(url, responseText))
        {
            string tmp = responseText;
            if (extractJsonPayload(tmp))
            {
                return true;
            }
            else
            {
                {
                    string key = string("ZENDRIVER_NONJSON:") + (url.size() > 128 ? url.substr(0, 128) : url);
                    if (rateLimitDetailedLog(key))
                    {
                        m_host->writeLog("FR24|ZENDRIVER|fetched non-JSON response for url[%s] truncated[%s]", url.c_str(), truncateForLog(responseText, 256).c_str());
                    }
                    else
                    {
                        m_host->writeLog("FR24|ZENDRIVER|non-JSON url[%s]", truncateForLog(url, 128).c_str());
                    }
                }
            }
        }

        // Fallback to curl for environments without Python/Zendriver/chrome.
        // Capture both stdout and stderr and include HTTP status for diagnostics.
        const string writeMarker = "__HTTP_STATUS_CODE__:";
        const int maxAttempts = 3;
        bool ok = false;
        int finalExitCode = -1;
        int finalHttpStatus = 0;
        string finalBody;
        string finalFull;

        for (int attempt = 1; attempt <= maxAttempts; ++attempt)
        {
            const string command = "curl -g -L -sS -H \"User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64)\" -H \"Accept: text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8\" -H \"Accept-Language: en-US,en;q=0.5\" -H \"Referer: https://www.google.com/\" -w \"\\n" + writeMarker + "%{http_code}\" \"" + url + "\" 2>&1";

#if IBM
            FILE* pipe = _popen(command.c_str(), "r");
#else
            FILE* pipe = popen(command.c_str(), "r");
#endif
            if (!pipe)
            {
                m_host->writeLog("FR24|curl|popen failed for command[%s]", command.c_str());
                break;
            }

            char buffer[4096];
            string responseCombined;
            while (fgets(buffer, sizeof(buffer), pipe))
            {
                responseCombined.append(buffer);
            }

#if IBM
            const int exitCode = _pclose(pipe);
#else
            const int exitCode = pclose(pipe);
#endif

            // Extract HTTP status if present and separate body
            int httpStatus = 0;
            string body = responseCombined;
            size_t markerPos = responseCombined.rfind(writeMarker);
            if (markerPos != string::npos)
            {
                size_t codePos = markerPos + writeMarker.length();
                string codeStr = responseCombined.substr(codePos);
                // Trim whitespace
                auto firstDigit = codeStr.find_first_of("0123456789");
                auto lastDigit = codeStr.find_last_of("0123456789");
                if (firstDigit != string::npos && lastDigit != string::npos && lastDigit >= firstDigit)
                {
                    try { httpStatus = stoi(codeStr.substr(firstDigit, lastDigit - firstDigit + 1)); } catch(...) { httpStatus = 0; }
                }
                // Body is everything before the marker (strip trailing newline)
                if (markerPos > 0 && responseCombined[markerPos - 1] == '\n')
                    body = responseCombined.substr(0, markerPos - 1);
                else
                    body = responseCombined.substr(0, markerPos);
            }

            finalExitCode = exitCode;
            finalHttpStatus = httpStatus;
            finalBody = body;
            finalFull = responseCombined;

            string tmp = body;
            if (extractJsonPayload(tmp))
            {
                // successful JSON extraction
                responseText = tmp;
                ok = true;
                break;
            }

            // If we got a 429 (rate limit), retry with backoff. Also retry on transient curl failures (non-zero exit code)
            if ((httpStatus == 429 || exitCode != 0) && attempt < maxAttempts)
            {
                int backoffMs = 250 * (1 << (attempt - 1)); // 250ms, 500ms, 1000ms
                if (rateLimitDetailedLog(string("FR24_RETRY:") + to_string(httpStatus) + ":" + (url.size() > 64 ? url.substr(0,64) : url)))
                {
                    m_host->writeLog("FR24|curl|attempt[%d] httpStatus[%d] exitCode[%d] url[%s] - retrying after %dms", attempt, httpStatus, exitCode, truncateForLog(url, 128).c_str(), backoffMs);
                }
                this_thread::sleep_for(chrono::milliseconds(backoffMs));
                continue;
            }

            // Non-retriable or out of attempts
            break;
        }

        string truncatedBody = truncateForLog(finalBody, 512);
        string truncatedFull = truncateForLog(finalFull, 512);

        if (!ok)
        {
            string key = string("CURL_FAIL:") + (url.size() > 128 ? url.substr(0, 128) : url);
            if (rateLimitDetailedLog(key))
            {
                m_host->writeLog("FR24|curl|finalExit[%d] httpStatus[%d] url[%s] bodyTrunc[%s] fullTrunc[%s]", finalExitCode, finalHttpStatus, url.c_str(), truncatedBody.c_str(), truncatedFull.c_str());
            }
            else
            {
                m_host->writeLog("FR24|curl|finalExit[%d] httpStatus[%d] url[%s]", finalExitCode, finalHttpStatus, truncateForLog(url, 128).c_str());
            }
        }

        return ok;
    }

    static string escapeForPythonTripleQuoted(const string& value)
    {
        string escaped = value;
        size_t pos = 0;
        while ((pos = escaped.find("'''", pos)) != string::npos)
        {
            escaped.replace(pos, 3, "\\'\\'\\'");
            pos += 6;
        }
        return escaped;
    }

    bool runPythonScriptAndCapture(const string& scriptPath, const string& pythonBinary, string& responseText)
    {
        // Capture stderr as well so callers can log useful diagnostics when zendriver/python fails.
        const string command = buildPythonScriptCommand(pythonBinary, scriptPath);
        m_host->writeLog("FR24|executing command[%s]", command.c_str());

#if IBM
        FILE* pipe = _popen(command.c_str(), "r");
#else
        FILE* pipe = popen(command.c_str(), "r");
#endif

        if (!pipe)
        {
            m_host->writeLog("FR24|popen failed for command[%s]", command.c_str());
            return false;
        }

        char buffer[4096];
        responseText.clear();
        while (fgets(buffer, sizeof(buffer), pipe))
        {
            responseText.append(buffer);
        }

#if IBM
        const int exitCode = _pclose(pipe);
#else
        const int exitCode = pclose(pipe);
#endif

        m_host->writeLog("FR24|command exitCode[%d] output[%s]", exitCode, responseText.c_str());
        return exitCode == 0;
    }

    bool fetchUrlTextWithZendriver(const string& url, string& responseText)
    {
        // Use the shared browser pool to avoid launching a new browser per request
        auto pool = ZendriverBrowserPool::getInstance(m_host);
        return pool->fetchUrl(url, responseText);
    }

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
        return value;
    }

    static string extractPreText(const string& text)
    {
        string lowered = text;
        transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) {
            return static_cast<char>(tolower(c));
        });

        const size_t preStart = lowered.find("<pre");
        if (preStart == string::npos)
        {
            return "";
        }

        const size_t contentStart = lowered.find('>', preStart);
        if (contentStart == string::npos)
        {
            return "";
        }

        const size_t preEnd = lowered.find("</pre>", contentStart + 1);
        if (preEnd == string::npos || preEnd <= contentStart)
        {
            return "";
        }

        return htmlEntityDecodeBasic(text.substr(contentStart + 1, preEnd - contentStart - 1));
    }

    static bool extractBalancedJsonCandidate(const string& source, size_t start, string& candidate)
    {
        if (start >= source.size() || (source[start] != '{' && source[start] != '['))
        {
            return false;
        }

        vector<char> expectedClosers;
        bool inString = false;
        bool escaped = false;

        for (size_t i = start; i < source.size(); ++i)
        {
            const char ch = source[i];

            if (inString)
            {
                if (escaped)
                {
                    escaped = false;
                }
                else if (ch == '\\')
                {
                    escaped = true;
                }
                else if (ch == '"')
                {
                    inString = false;
                }
                continue;
            }

            if (ch == '"')
            {
                inString = true;
                continue;
            }

            if (ch == '{')
            {
                expectedClosers.push_back('}');
                continue;
            }
            if (ch == '[')
            {
                expectedClosers.push_back(']');
                continue;
            }
            if (ch == '}' || ch == ']')
            {
                if (expectedClosers.empty() || expectedClosers.back() != ch)
                {
                    return false;
                }

                expectedClosers.pop_back();
                if (expectedClosers.empty())
                {
                    candidate = source.substr(start, i - start + 1);
                    return true;
                }
            }
        }

        return false;
    }

    static bool acceptJsonCandidate(const string& candidate, string& text)
    {
        string trimmed = trimCopy(candidate);
        if (trimmed.empty())
        {
            return false;
        }

        try
        {
            SimpleJson::parse(trimmed);
            text = trimmed;
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    static bool extractJsonPayload(string& text)
    {
        if (text.empty())
        {
            return false;
        }

        string candidate;
        string trimmed = trimCopy(text);
        if (!trimmed.empty() && (trimmed.front() == '{' || trimmed.front() == '['))
        {
            if (extractBalancedJsonCandidate(trimmed, 0, candidate) && acceptJsonCandidate(candidate, text))
            {
                return true;
            }
        }

        string preText = extractPreText(text);
        if (!preText.empty() && extractJsonPayload(preText))
        {
            text = preText;
            return true;
        }

        const vector<string> markers = {
            "window._APP_STATE",
            "var opts =",
            "\"result\"",
            "\"response\""
        };

        for (const auto& marker : markers)
        {
            size_t markerPos = text.find(marker);
            while (markerPos != string::npos)
            {
                size_t start = text.find_first_of("{[", markerPos);
                if (start == string::npos)
                {
                    break;
                }
                if (extractBalancedJsonCandidate(text, start, candidate) && acceptJsonCandidate(candidate, text))
                {
                    return true;
                }

                markerPos = text.find(marker, markerPos + marker.length());
            }
        }

        for (size_t start = text.find_first_of("{[");
             start != string::npos;
             start = text.find_first_of("{[", start + 1))
        {
            if (extractBalancedJsonCandidate(text, start, candidate) && acceptJsonCandidate(candidate, text))
            {
                return true;
            }
        }

        return false;
    }

    static string normalizeCode(string value)
    {
        value.erase(remove_if(value.begin(), value.end(), [](unsigned char c) {
            return isspace(c);
        }), value.end());

        for (char& c : value)
        {
            c = static_cast<char>(toupper(static_cast<unsigned char>(c)));
        }

        return value;
    }

    // FR24 and some other live sources frequently use shorthand aircraft
    // family codes instead of the full ICAO type designator. Normalize the
    // common ones so they don't fall through to the generic fallback aircraft.
    static string canonicalizeAircraftIcao(string aircraftIcao)
    {
        aircraftIcao = normalizeCode(aircraftIcao);
        if (aircraftIcao.empty())
        {
            return "";
        }

        if (aircraftIcao == "318")
        {
            return "A318";
        }
        if (aircraftIcao == "319")
        {
            return "A319";
        }
        if (aircraftIcao == "320")
        {
            return "A320";
        }
        if (aircraftIcao == "321")
        {
            return "A321";
        }
        if (aircraftIcao == "32N")
        {
            return "A20N";
        }
        if (aircraftIcao.rfind("32", 0) == 0)
        {
            return "A320";
        }

        if (aircraftIcao == "737" || aircraftIcao == "73G" || aircraftIcao == "73W")
        {
            return "B737";
        }
        if (aircraftIcao == "738" || aircraftIcao == "73H")
        {
            return "B738";
        }
        if (aircraftIcao == "739")
        {
            return "B739";
        }
        if (aircraftIcao == "7M8")
        {
            return "B38M";
        }
        if (aircraftIcao == "7M9")
        {
            return "B39M";
        }

        return aircraftIcao;
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

    static string getStringPath(const SimpleJson& node, initializer_list<vector<string>> paths)
    {
        for (const auto& path : paths)
        {
            const SimpleJson* value = tryGetPath(node, path);
            if (value && !value->isNull())
            {
                const string text = value->asString();
                if (!text.empty())
                {
                    return text;
                }
            }
        }

        return "";
    }

    static long long getIntegerPath(const SimpleJson& node, initializer_list<vector<string>> paths)
    {
        for (const auto& path : paths)
        {
            const SimpleJson* value = tryGetPath(node, path);
            if (value && !value->isNull())
            {
                const long long integerValue = value->asInteger(0);
                if (integerValue != 0)
                {
                    return integerValue;
                }
            }
        }

        return 0;
    }

    static string firstNonEmpty(const string& a, const string& b)
    {
        return !a.empty() ? a : b;
    }

    static string stripLeadingFlightDesignator(const string& value)
    {
        size_t index = 0;
        while (index < value.size() && !isdigit(static_cast<unsigned char>(value.at(index))))
        {
            ++index;
        }

        return index < value.size()
            ? value.substr(index)
            : value;
    }

    static string extractFlightNumberRaw(const SimpleJson& node)
    {
        string flightNumber = getStringPath(node, {
            { "flight", "identification", "number", "default" },
            { "flight", "identification", "number", "alternative" },
            { "identification", "number", "default" },
            { "identification", "number", "alternative" }
        });

        flightNumber = normalizeCode(flightNumber);
        if (!flightNumber.empty())
        {
            return flightNumber;
        }

        string callsign = getStringPath(node, {
            { "flight", "identification", "callsign" },
            { "identification", "callsign" },
            { "flight", "callsign" }
        });

        callsign = normalizeCode(callsign);
        if (!callsign.empty())
        {
            return callsign;
        }

        return "";
    }

    static string extractFlightNumber(const SimpleJson& node)
    {
        const string flightNumber = extractFlightNumberRaw(node);
        if (!flightNumber.empty())
        {
            return stripLeadingFlightDesignator(flightNumber);
        }

        return "";
    }

    static string extractAirlineLabel(const SimpleJson& node)
    {
        return getStringPath(node, {
            { "flight", "airline", "name" },
            { "flight", "airline", "callsign" },
            { "flight", "owner", "name" },
            { "flight", "owner", "callsign" },
            { "airline", "name" },
            { "airline", "callsign" },
            { "owner", "name" },
            { "owner", "callsign" }
        });
    }

    static string extractAirlineIcao(const SimpleJson& node, const string& rawFlightNumber, const string& flightNumber)
    {
        string airlineIcao = getStringPath(node, {
            { "flight", "airline", "code", "icao" },
            { "flight", "airline", "icao" },
            { "flight", "airline", "code" },
            { "flight", "owner", "code", "icao" },
            { "flight", "owner", "icao" },
            { "airline", "code", "icao" },
            { "airline", "icao" },
            { "airline", "code" },
            { "owner", "code", "icao" },
            { "owner", "icao" }
        });

        airlineIcao = normalizeCode(airlineIcao);
        if (airlineIcao.length() == 3)
        {
            AirlineReferenceTable::Entry airline;
            if (AirlineReferenceTable::tryFindByIcao(airlineIcao, airline))
            {
                return airlineIcao;
            }
        }

        const string airlineLabel = extractAirlineLabel(node);
        if (!airlineLabel.empty())
        {
            AirlineReferenceTable::Entry airline;
            if (AirlineReferenceTable::tryFindByName(airlineLabel, airline))
            {
                return airline.icao;
            }
        }

        if (!rawFlightNumber.empty())
        {
            AirlineReferenceTable::Entry airline;
            string flightCallsign;
            if (AirlineReferenceTable::tryFindByFlightNumber(rawFlightNumber, airline, flightCallsign))
            {
                return airline.icao;
            }
        }

        if (!flightNumber.empty())
        {
            AirlineReferenceTable::Entry airline;
            string flightCallsign;
            if (AirlineReferenceTable::tryFindByFlightNumber(flightNumber, airline, flightCallsign))
            {
                return airline.icao;
            }
        }

        return "";
    }

    static string extractAircraftIcao(const SimpleJson& node)
    {
        string aircraftIcao = getStringPath(node, {
            { "flight", "aircraft", "model", "code" },
            { "flight", "aircraft", "model", "icao" },
            { "flight", "aircraft", "code", "icao" },
            { "flight", "aircraft", "icao" },
            { "flight", "aircraft", "type", "icao" },
            { "flight", "aircraft", "type" },
            { "aircraft", "model", "code" },
            { "aircraft", "model", "icao" },
            { "aircraft", "code", "icao" },
            { "aircraft", "icao" },
            { "aircraft", "type", "icao" },
            { "aircraft", "type" }
        });

        aircraftIcao = canonicalizeAircraftIcao(aircraftIcao);
        if (aircraftIcao.empty())
        {
            return "";
        }

        AircraftTypeReferenceTable::Entry aircraft;
        if (AircraftTypeReferenceTable::tryFindByIcao(aircraftIcao, aircraft))
        {
            return aircraft.icao;
        }

        if (aircraftIcao.length() > 4)
        {
            const string aircraftPrefix = aircraftIcao.substr(0, 4);
            if (AircraftTypeReferenceTable::tryFindByIcao(aircraftPrefix, aircraft))
            {
                return aircraft.icao;
            }
        }

        return aircraftIcao.length() == 4 ? aircraftIcao : "";
    }

    static string resolveCallsign(const string& airlineIcao, const string& flightNumber)
    {
        AirlineReferenceTable::Entry airline;
        if (!airlineIcao.empty() && AirlineReferenceTable::tryFindByIcao(airlineIcao, airline) && !airline.callsign.empty())
        {
            return flightNumber.empty() ? airline.callsign : airline.callsign + " " + flightNumber;
        }

        return !flightNumber.empty() ? flightNumber : airlineIcao;
    }

    static time_t extractDepartureScheduleTime(const SimpleJson& node)
    {
        const long long value = getIntegerPath(node, {
            { "flight", "time", "estimated", "departure" },
            { "flight", "time", "real", "departure" },
            { "flight", "time", "other", "etd" },
            { "flight", "time", "scheduled", "departure" },
            { "flight", "time", "departure" },
            { "time", "estimated", "departure" },
            { "time", "real", "departure" },
            { "time", "other", "etd" },
            { "time", "scheduled", "departure" },
            { "time", "departure" },
        });

        return value > 0 ? static_cast<time_t>(value) : 0;
    }

    static time_t extractArrivalScheduleTime(const SimpleJson& node)
    {
        const long long value = getIntegerPath(node, {
            { "flight", "time", "estimated", "arrival" },
            { "flight", "time", "other", "eta" },
            { "flight", "time", "real", "arrival" },
            { "flight", "time", "scheduled", "arrival" },
            { "flight", "time", "arrival" },
            { "time", "estimated", "arrival" },
            { "time", "other", "eta" },
            { "time", "real", "arrival" },
            { "time", "scheduled", "arrival" },
            { "time", "arrival" }
        });

        return value > 0 ? static_cast<time_t>(value) : 0;
    }

    static time_t extractScheduleTime(const SimpleJson& node, const string& mode)
    {
        return mode == "arrivals"
            ? extractArrivalScheduleTime(node)
            : extractDepartureScheduleTime(node);
    }

    static bool shouldSkipByStatus(const string& normalizedStatusText)
    {
        if (normalizedStatusText.empty())
        {
            return false;
        }

        // Landed/arrived/completed legs are stale for schedule-based spawning.
        if (normalizedStatusText.rfind("LANDED", 0) == 0 ||
            normalizedStatusText.rfind("ARRIVED", 0) == 0 ||
            normalizedStatusText.rfind("DEPARTED", 0) == 0)
        {
            return true;
        }

        // Explicitly non-flyable statuses should be ignored as well.
        // NOTE: do not skip diverted flights here; we want to keep them and
        // apply diversion destination handling in the loader.
        return normalizedStatusText.find("CANCEL") != string::npos;
    }

    static string extractDiversionDestinationIcao(const SimpleJson& node, const string& normalizedStatusText)
    {
        string divertedIcao = normalizeCode(getStringPath(node, {
            { "flight", "airport", "destination", "alternate", "code", "icao" },
            { "flight", "airport", "destination", "alternative", "code", "icao" },
            { "flight", "airport", "destination", "diverted", "code", "icao" },
            { "flight", "airport", "destination", "real", "code", "icao" },
            { "airport", "destination", "alternate", "code", "icao" },
            { "airport", "destination", "alternative", "code", "icao" },
            { "airport", "destination", "diverted", "code", "icao" },
            { "airport", "destination", "real", "code", "icao" },
            { "destination", "alternate", "code", "icao" },
            { "destination", "alternative", "code", "icao" },
            { "destination", "diverted", "code", "icao" },
            { "destination", "real", "code", "icao" }
        }));

        if (divertedIcao.length() == 4)
        {
            return divertedIcao;
        }

        if (!normalizedStatusText.empty())
        {
            // Examples seen in live feeds: "DIVERTED", "DIVERTED TO LEST"
            // Keep it strict to 4-letter ICAO tokens.
            static const regex divertedToPattern(R"(DIVERT(?:ED)?(?:\s+TO)?\s+([A-Z]{4}))", regex_constants::icase);
            smatch match;
            if (regex_search(normalizedStatusText, match, divertedToPattern) && match.size() > 1)
            {
                const string parsed = normalizeCode(match[1].str());
                if (parsed.length() == 4)
                {
                    return parsed;
                }
            }
        }

        return "";
    }

    void collectEntries(const SimpleJson& dataArray, const string& airportIcao, const string& mode, vector<Fr24ScheduleEntry>& entries)
    {
        unordered_set<string> seen;
        const bool isDepartureMode = (mode == "departures");
        const string normalizedAirportIcao = normalizeCode(airportIcao);

        if (!dataArray.isArray())
        {
            return;
        }

        for (const auto& item : dataArray.arrayValue())
        {
            if (!item.isObject())
            {
                continue;
            }

            const string statusText = normalizeCode(getStringPath(item, {
                { "flight", "status", "generic", "status", "text" },
                { "flight", "status", "text" },
                { "status", "generic", "status", "text" },
                { "status", "text" }
            }));
            if (shouldSkipByStatus(statusText))
            {
                continue;
            }

            const string rawFlightNumber = extractFlightNumberRaw(item);
            const string flightNumber = stripLeadingFlightDesignator(rawFlightNumber);
            if (flightNumber.empty())
            {
                continue;
            }

            string originIcao = normalizeCode(getStringPath(item, {
                { "flight", "airport", "origin", "code", "icao" },
                { "flight", "airport", "origin", "icao" },
                { "flight", "origin", "code", "icao" },
                { "flight", "origin", "icao" },
                { "airport", "origin", "code", "icao" },
                { "airport", "origin", "icao" },
                { "origin", "code", "icao" },
                { "origin", "icao" }
            }));

            string destinationIcao = normalizeCode(getStringPath(item, {
                { "flight", "airport", "destination", "code", "icao" },
                { "flight", "airport", "destination", "icao" },
                { "flight", "destination", "code", "icao" },
                { "flight", "destination", "icao" },
                { "airport", "destination", "code", "icao" },
                { "airport", "destination", "icao" },
                { "destination", "code", "icao" },
                { "destination", "icao" }
            }));

            const string divertedDestinationIcao = extractDiversionDestinationIcao(item, statusText);
            const bool diverted = statusText.find("DIVERT") != string::npos ||
                (!divertedDestinationIcao.empty() && divertedDestinationIcao != destinationIcao);
            if (!divertedDestinationIcao.empty())
            {
                destinationIcao = divertedDestinationIcao;
            }

            if (isDepartureMode)
            {
                if (originIcao.empty())
                {
                    originIcao = normalizedAirportIcao;
                }
            }
            else
            {
                if (destinationIcao.empty())
                {
                    destinationIcao = normalizedAirportIcao;
                }
            }

            if (originIcao.empty() && destinationIcao.empty())
            {
                continue;
            }

            const string airlineIcao = extractAirlineIcao(item, rawFlightNumber, flightNumber);
            const string aircraftIcao = extractAircraftIcao(item);
            const string callsign = resolveCallsign(airlineIcao, flightNumber);
            const time_t scheduleTime = extractScheduleTime(item, mode);

            const string dedupeKey =
                mode + "|" + flightNumber + "|" + originIcao + "|" + destinationIcao + "|" +
                airlineIcao + "|" + aircraftIcao + "|" + callsign + "|" + to_string(static_cast<long long>(scheduleTime));
            if (!seen.insert(dedupeKey).second)
            {
                continue;
            }

            Fr24ScheduleEntry entry;
            entry.airlineIcao = airlineIcao;
            entry.flightNumber = flightNumber;
            entry.callsign = callsign;
            entry.aircraftIcao = aircraftIcao;
            entry.originIcao = originIcao;
            entry.destinationIcao = destinationIcao;
            entry.diverted = diverted;
            entry.divertedDestinationIcao = divertedDestinationIcao;
            entry.scheduledTime = scheduleTime;
            entries.push_back(entry);
        }
    }
};
