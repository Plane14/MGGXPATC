//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "libworld.h"
#include "aircraftTypeReferenceTable.hpp"

using namespace std;
using namespace world;

namespace ai
{
    struct AircraftPerformanceProfile
    {
        float approachSpeedKt = 145.0f;
        float descentRateFpm = 1000.0f;
        float minutesToThreshold = 4.0f;
        float flareBufferFeet = 40.0f;
        float landingTouchdownDistanceMeters = 380.0f;
        float landingTouchdownSpeedKt = 132.0f;
        float landingExitSpeedKt = 30.0f;
        float landingRolloutDecelerationKtPerSecond = 5.2f;
        float takeoffRotateSpeedKt = 120.0f;
        float takeoffLiftOffSpeedKt = 130.0f;
        float takeoffInitialClimbSpeedKt = 150.0f;
        float takeoffAccelerationKtPerSecond = 7.0f;
        float takeoffRotatePitchDegrees = 12.0f;
        float initialClimbRocFpm = 1500.0f;  // Rate of Climb in initial climb (to 5000ft)
        float rangeNm = 500.0f;              // Aircraft range in nautical miles (from web performance data or default)
        int ceilingFl = 450;                 // Maximum operating ceiling in flight levels (from web performance data or default)
        string classification;
        string wakeCategory;
        bool hasEurocontrolData = false;     // Loaded from a web performance page (Eurocontrol or Doc8643 fallback)

        // Takeoff and landing distances from Eurocontrol aircraft performance database
        // These are aircraft-specific required distances (TODR/LDR), not runway declared distances
        float takeoffDistanceMeters = 0.0f;  // Take-off Distance Required (TODR) at MTOW, sea level, standard conditions
        float landingDistanceMeters = 0.0f;    // Landing Distance Required (LDR) at MLW, sea level, standard conditions

        // Determines if this aircraft can perform unrestricted vertical climbouts
        // Based on Eurocontrol data: requires >= 20,000 ft/min initial climb rate
        bool canPerformUnrestrictedClimbout() const
        {
            return initialClimbRocFpm >= 20000.0f;
        }
    };

    class AircraftPerformanceTable
    {
    private:
        static bool containsAnyToken(const string& value, const vector<const char*>& tokens)
        {
            for (const char* token : tokens)
            {
                if (token && *token && value.find(token) != string::npos)
                {
                    return true;
                }
            }

            return false;
        }

        static map<string, AircraftPerformanceProfile>& cache()
        {
            static map<string, AircraftPerformanceProfile> s_cache;
            return s_cache;
        }

        static bool& loaded()
        {
            static bool s_loaded = false;
            return s_loaded;
        }

        static bool applyLocalModelHeuristics(const string& normalizedIcao, world::Aircraft::Category category, AircraftPerformanceProfile& profile)
        {
            const auto apply = [&](float approachSpeedKt,
                                   float descentRateFpm,
                                   float minutesToThreshold,
                                   float flareBufferFeet,
                                   float landingTouchdownDistanceMeters,
                                   float landingTouchdownSpeedKt,
                                   float landingExitSpeedKt,
                                   float landingRolloutDecelerationKtPerSecond,
                                   float takeoffRotateSpeedKt,
                                   float takeoffLiftOffSpeedKt,
                                   float takeoffInitialClimbSpeedKt,
                                   float takeoffAccelerationKtPerSecond,
                                   float takeoffRotatePitchDegrees,
                                   float initialClimbRocFpm,
                                   float rangeNm,
                                   int ceilingFl) {
                profile.approachSpeedKt = approachSpeedKt;
                profile.descentRateFpm = descentRateFpm;
                profile.minutesToThreshold = minutesToThreshold;
                profile.flareBufferFeet = flareBufferFeet;
                profile.landingTouchdownDistanceMeters = landingTouchdownDistanceMeters;
                profile.landingTouchdownSpeedKt = landingTouchdownSpeedKt;
                profile.landingExitSpeedKt = landingExitSpeedKt;
                profile.landingRolloutDecelerationKtPerSecond = landingRolloutDecelerationKtPerSecond;
                profile.takeoffRotateSpeedKt = takeoffRotateSpeedKt;
                profile.takeoffLiftOffSpeedKt = takeoffLiftOffSpeedKt;
                profile.takeoffInitialClimbSpeedKt = takeoffInitialClimbSpeedKt;
                profile.takeoffAccelerationKtPerSecond = takeoffAccelerationKtPerSecond;
                profile.takeoffRotatePitchDegrees = takeoffRotatePitchDegrees;
                profile.initialClimbRocFpm = initialClimbRocFpm;
                profile.rangeNm = rangeNm;
                profile.ceilingFl = ceilingFl;
                return true;
            };

            if (normalizedIcao.empty())
            {
                return false;
            }

            if (category == world::Aircraft::Category::Jet)
            {
                if (normalizedIcao == "B738" || normalizedIcao == "B737" || normalizedIcao == "B739" ||
                    normalizedIcao == "B38M" || normalizedIcao == "B39M" || normalizedIcao == "B37M" ||
                    normalizedIcao == "A318" || normalizedIcao == "A319" || normalizedIcao == "A320" ||
                    normalizedIcao == "A20N" || normalizedIcao == "A321" || normalizedIcao == "A21N" ||
                    normalizedIcao.rfind("B73", 0) == 0 || normalizedIcao.rfind("A32", 0) == 0)
                {
                    return apply(145.0f, 1100.0f, 5.0f, 40.0f, 420.0f, 138.0f, 35.0f, 5.0f, 145.0f, 156.0f, 178.0f, 3.0f, 13.0f, 2600.0f, 3200.0f, 410);
                }

                if (normalizedIcao == "CRJ2" || normalizedIcao == "CRJ7" || normalizedIcao == "CRJ9" ||
                    normalizedIcao == "E170" || normalizedIcao == "E175" || normalizedIcao == "E190" ||
                    normalizedIcao == "E195" || normalizedIcao.rfind("CRJ", 0) == 0)
                {
                    return apply(140.0f, 900.0f, 5.0f, 35.0f, 300.0f, 118.0f, 30.0f, 5.4f, 132.0f, 142.0f, 160.0f, 3.5f, 12.0f, 2200.0f, 1700.0f, 410);
                }
            }

            if (category == world::Aircraft::Category::Turboprop)
            {
                if (normalizedIcao == "AT72" || normalizedIcao == "DH8A" || normalizedIcao == "DH8B" ||
                    normalizedIcao == "DH8C" || normalizedIcao == "DH8D" || normalizedIcao == "SF34")
                {
                    return apply(115.0f, 800.0f, 5.0f, 35.0f, 250.0f, 100.0f, 25.0f, 5.8f, 108.0f, 118.0f, 138.0f, 2.5f, 11.0f, 1800.0f, 1200.0f, 270);
                }
            }

            if (category == world::Aircraft::Category::LightProp || category == world::Aircraft::Category::Prop)
            {
                if (normalizedIcao == "C152" || normalizedIcao == "C172")
                {
                    return apply(68.0f, 500.0f, 4.0f, 30.0f, 120.0f, 58.0f, 15.0f, 4.2f, 55.0f, 62.0f, 82.0f, 1.5f, 9.0f, 900.0f, 430.0f, 130);
                }
            }

            if (category == world::Aircraft::Category::Fighter)
            {
                if (normalizedIcao == "F22")
                {
                    return apply(176.0f, 1500.0f, 6.0f, 50.0f, 680.0f, 155.0f, 45.0f, 7.0f, 155.0f, 175.0f, 260.0f, 18.0f, 14.0f, 30000.0f, 1500.0f, 555);
                }

                if (normalizedIcao == "F16" || normalizedIcao == "F18" || normalizedIcao == "FA18" ||
                    normalizedIcao == "F35")
                {
                    return apply(175.0f, 1500.0f, 6.0f, 50.0f, 620.0f, 148.0f, 40.0f, 6.8f, 150.0f, 170.0f, 240.0f, 16.0f, 14.0f, 25000.0f, 1500.0f, 500);
                }
            }

            return false;
        }

        static bool shouldSkipNetworkFetch(shared_ptr<HostServices> host)
        {
            if (!host)
            {
                return true;
            }

            try
            {
                const string resourcePath = host->getResourceFilePath({ "Resources", "aircraft-performance.csv" });
                return resourcePath.rfind("PLUGIN_DIR/", 0) == 0 || resourcePath.rfind("PLUGIN_DIR\\", 0) == 0;
            }
            catch (const exception&)
            {
                return false;
            }
        }

        enum class PerformanceDataSource
        {
            None,
            Eurocontrol,
            Doc8643
        };

        static const char* performanceSourceName(PerformanceDataSource source)
        {
            switch (source)
            {
            case PerformanceDataSource::Eurocontrol:
                return "Eurocontrol";
            case PerformanceDataSource::Doc8643:
                return "Doc8643";
            default:
                return "default";
            }
        }

    public:
        static AircraftPerformanceProfile lookup(shared_ptr<HostServices> host, const string& modelIcao, world::Aircraft::Category category)
        {
            ensureLoaded(host);

            const string key = normalizeIcao(modelIcao);
            if (key.empty())
            {
                return defaultProfile(category);
            }

            auto it = cache().find(key);
            if (it != cache().end())
            {
                return it->second;
            }

            AircraftPerformanceProfile profile = defaultProfile(category);

            applyLocalModelHeuristics(key, category, profile);

            if (host && !shouldSkipNetworkFetch(host) && loadPerformanceDetailsPage(host, key, profile) != PerformanceDataSource::None)
            {
                cache()[key] = profile;
                return profile;
            }

            cache()[key] = profile;
            return profile;
        }

        static float minimumRunwayLengthMeters(world::Aircraft::Category category)
        {
            switch (category)
            {
            case world::Aircraft::Category::Helicopter:
                return 0.0f;
            case world::Aircraft::Category::LightProp:
                return 550.0f;
            case world::Aircraft::Category::Prop:
                return 800.0f;
            case world::Aircraft::Category::Turboprop:
                return 1200.0f;
            case world::Aircraft::Category::Fighter:
                return 1400.0f;
            case world::Aircraft::Category::Heavy:
                return 2400.0f;
            case world::Aircraft::Category::Jet:
            default:
                return 1800.0f;
            }
        }

        static bool canOperateFromRunwayLengthMeters(world::Aircraft::Category category, float runwayLengthMeters)
        {
            return runwayLengthMeters >= minimumRunwayLengthMeters(category);
        }

        static bool isRegionalAirlinerTurbopropType(const string& icaoType)
        {
            static const unordered_set<string> regionalAirlinerTurboprops = {
                "AT45", "AT72", "DH8A", "DH8B", "DH8C", "DH8D",
                "E110", "E120", "F50", "JS31", "JS32", "JS41",
                "L410", "SF34", "SW4", "ATP", "AN26", "AN28", "AN38"
            };

            const string key = normalizeIcao(icaoType);
            return regionalAirlinerTurboprops.find(key) != regionalAirlinerTurboprops.end();
        }

        static bool isGeneralAviationTrafficType(const string& icaoType, world::Aircraft::Category category)
        {
            const string key = normalizeIcao(icaoType);
            if (key.empty())
            {
                return false;
            }

            if (category == world::Aircraft::Category::LightProp || category == world::Aircraft::Category::Prop)
            {
                return true;
            }

            if (category != world::Aircraft::Category::Turboprop)
            {
                return false;
            }

            if (isRegionalAirlinerTurbopropType(key))
            {
                return false;
            }

            static const unordered_set<string> gaTurbopropTypes = {
                "C208", "C208B", "DHC2", "DHC3", "DHC6", "PC12", "PC6",
                "TBM7", "TBM8", "TBM9", "TBMX", "BE10", "BE20", "BE30",
                "C90", "C90B", "C90GT", "F406", "P68T"
            };

            if (gaTurbopropTypes.find(key) != gaTurbopropTypes.end())
            {
                return true;
            }

            return key.rfind("TBM", 0) == 0 ||
                key.rfind("PC", 0) == 0 ||
                key.rfind("C20", 0) == 0 ||
                key.rfind("BE2", 0) == 0 ||
                key.rfind("C90", 0) == 0;
        }

    public:
        /// @brief Discovers installed fighter CSL models and loads their Eurocontrol performance data
        /// @details Scans CSL directories for xsb_aircraft.txt files, extracts ICAO codes for fighter aircraft,
        ///          and fetches their performance data from Eurocontrol database.
        /// @param host Host services for file access
        /// @param cslBasePath Base path to CSL models (e.g., "Resources/CSL" or X-Plane CSL path)
        /// @return Number of fighter CSL models discovered and loaded
        static int discoverFighterCSLModels(shared_ptr<HostServices> host, const string& cslBasePath)
        {
            if (!host)
                return 0;

            int fighterCount = 0;
            vector<string> icaoTypes;

            // Scan CSL directories for xsb_aircraft.txt files
            scanCSLDirectory(host, cslBasePath, icaoTypes);

            // Look up each unique ICAO type in Eurocontrol
            for (const auto& icao : icaoTypes)
            {
                string key = normalizeIcao(icao);
                if (key.empty() || cache().find(key) != cache().end())
                    continue; // Already cached

                AircraftPerformanceProfile profile = defaultProfile(world::Aircraft::Category::Fighter);

                // Try to fetch from Eurocontrol first, then Doc8643 as a fallback
                const PerformanceDataSource source = loadPerformanceDetailsPage(host, key, profile);
                if (source != PerformanceDataSource::None)
                {
                    // Check if this is actually a high-performance fighter (ROC >= 20000)
                    if (profile.canPerformUnrestrictedClimbout())
                    {
                        cache()[key] = profile;
                        host->writeLog("PERF|Loaded fighter CSL model[%s] ROC=%.0f fpm from %s",
                            key.c_str(), profile.initialClimbRocFpm, performanceSourceName(source));
                        fighterCount++;
                    }
                    else
                    {
                        host->writeLog("PERF|CSL model[%s] ROC=%.0f fpm - not a high-performance fighter",
                            key.c_str(), profile.initialClimbRocFpm);
                    }
                }
            }

            if (fighterCount > 0)
            {
                host->writeLog("PERF|Discovered %d high-performance fighter CSL models", fighterCount);
            }

            return fighterCount;
        }

        /// @brief Discovers installed GA (General Aviation) CSL models and loads their Eurocontrol performance data
        /// @details Scans CSL directories for xsb_aircraft.txt files, extracts ICAO codes for GA aircraft
        ///          (LightProp, Prop, Turboprop), and fetches their performance data from Eurocontrol database.
        /// @param host Host services for file access
        /// @param cslBasePath Base path to CSL models (e.g., "Resources/CSL" or X-Plane CSL path)
        /// @return Number of GA CSL models discovered and loaded
        static int discoverGaCSLModels(shared_ptr<HostServices> host, const string& cslBasePath)
        {
            if (!host)
                return 0;

            int gaCount = 0;
            vector<string> icaoTypes;

            // Scan CSL directories for xsb_aircraft.txt files
            scanCSLDirectory(host, cslBasePath, icaoTypes);

            // Look up each unique ICAO type in Eurocontrol
            for (const auto& icao : icaoTypes)
            {
                string key = normalizeIcao(icao);
                if (key.empty() || cache().find(key) != cache().end())
                    continue; // Already cached

                // Determine likely category from ICAO code via Doc8643 classification lookup
                world::Aircraft::Category category = classifyFromIcao(key);
                if (!isGeneralAviationTrafficType(key, category))
                {
                    continue; // Not a GA aircraft type
                }

                AircraftPerformanceProfile profile = defaultProfile(category);

                // Try to fetch from Eurocontrol first, then Doc8643 as a fallback
                const PerformanceDataSource source = loadPerformanceDetailsPage(host, key, profile);
                if (source != PerformanceDataSource::None)
                {
                    cache()[key] = profile;
                    host->writeLog("PERF|Loaded GA CSL model[%s] category=%d approach=%.0f kts descent=%.0f fpm from %s",
                        key.c_str(), category, profile.approachSpeedKt, profile.descentRateFpm, performanceSourceName(source));
                    gaCount++;
                }
            }

            if (gaCount > 0)
            {
                host->writeLog("PERF|Discovered %d GA CSL models with Eurocontrol data", gaCount);
            }

            return gaCount;
        }

        /// @brief Discovers installed Helicopter CSL models and loads their Eurocontrol performance data
        /// @details Scans CSL directories for xsb_aircraft.txt files, extracts ICAO codes for helicopters,
        ///          and fetches their performance data from Eurocontrol database.
        /// @param host Host services for file access
        /// @param cslBasePath Base path to CSL models (e.g., "Resources/CSL" or X-Plane CSL path)
        /// @return Number of helicopter CSL models discovered and loaded
        static int discoverHelicopterCSLModels(shared_ptr<HostServices> host, const string& cslBasePath)
        {
            if (!host)
                return 0;

            int heliCount = 0;
            vector<string> icaoTypes;

            // Scan CSL directories for xsb_aircraft.txt files
            scanCSLDirectory(host, cslBasePath, icaoTypes);

            // Look up each unique ICAO type in Eurocontrol
            for (const auto& icao : icaoTypes)
            {
                string key = normalizeIcao(icao);
                if (key.empty() || cache().find(key) != cache().end())
                    continue; // Already cached

                // Determine likely category from ICAO code via Doc8643 classification lookup
                world::Aircraft::Category category = classifyFromIcao(key);
                if (category != world::Aircraft::Category::Helicopter)
                {
                    continue; // Not a helicopter type
                }

                AircraftPerformanceProfile profile = defaultProfile(world::Aircraft::Category::Helicopter);

                // Try to fetch from Eurocontrol first, then Doc8643 as a fallback
                const PerformanceDataSource source = loadPerformanceDetailsPage(host, key, profile);
                if (source != PerformanceDataSource::None)
                {
                    cache()[key] = profile;
                    host->writeLog("PERF|Loaded Helicopter CSL model[%s] approach=%.0f kts descent=%.0f fpm from %s",
                        key.c_str(), profile.approachSpeedKt, profile.descentRateFpm, performanceSourceName(source));
                    heliCount++;
                }
            }

            if (heliCount > 0)
            {
                host->writeLog("PERF|Discovered %d helicopter CSL models with Eurocontrol data", heliCount);
            }

            return heliCount;
        }

        /// @brief Classifies an ICAO aircraft type code to a world::Aircraft::Category based on Doc8643 patterns
        /// @param icaoType The ICAO aircraft type code (e.g., "C172", "B738", "EC35")
        /// @return The detected aircraft category, or world::Aircraft::Category::Jet as default
        static world::Aircraft::Category classifyFromIcao(const string& icaoType)
        {
            // Common GA aircraft ICAO codes for quick classification
            // Light Prop (L1P/L2P classification typically)
            static const unordered_set<string> lightPropTypes = {
                "C172", "C182", "C152", "PA28", "PA38", "SR20", "SR22",
                "DA40", "DA42", "DV20", "M20P", "BE23", "BE24", "BE36",
                "C77R", "C82R", "C85R", "C150", "C175", "C180", "C185",
                "C206", "PA18", "PA22", "PA24", "PA30", "PA32", "PA46",
                "GC1", "CH7A", "CH7B", "BL8", "BL8M", "G109", "G115",
                "NG5", "RV6", "RV7", "RV8", "RV9", "RV10", "S10S"
            };

            // Prop (multi-engine props, typically L2P/L4P)
            static const unordered_set<string> propTypes = {
                "BE58", "BE55", "BE56", "PA31", "PA34", "PA44", "PA60",
                "BE76", "BE95", "BE99", "BE10", "BE20", "C310", "C320",
                "C335", "C340", "C402", "C404", "C414", "C421", "C425",
                "C441", "C90", "C90B", "C90GT", "E50P", "E50R", "E55P",
                "F406", "P68", "P68T", "PA27", "PA30", "PA39", "PA42"
            };

            // Turboprop (L1T/L2T classification typically)
            static const unordered_set<string> turbopropTypes = {
                "C208", "C208B", "DHC6", "DHC2", "DHC3", "PC12", "PC6",
                "PC24", "TBM7", "TBM8", "TBM9", "TBMX", "BE30", "BE33",
                "BE35", "BE40", "BE60", "BE99", "BE10", "BE20", "BE30",
                "AT45", "AT72", "SF34", "DH8A", "DH8B", "DH8C", "DH8D",
                "E110", "E120", "F50", "JS31", "JS32", "JS41", "L410",
                "SW4", "ATP", "AN26", "AN28", "AN38", "A30B", "A400"
            };

            // Helicopter (H classification in Doc8643)
            static const unordered_set<string> helicopterTypes = {
                "AS32", "AS50", "AS55", "AS65",
                "B06", "B105", "B206", "B212", "B214", "B222", "B230", "B407", "B412", "B427", "B429", "B505",
                "EC20", "EC25", "EC30", "EC35", "EC45", "EC55", "EC75",
                "MI8", "MI24",
                "R22", "R44", "R66",
                "S300", "S58T", "S61", "S65C", "S76", "S92",
                "AW09", "AW11", "AW109", "AW119", "AW139", "AW149",
                "H125", "H130", "H135", "H145", "H155", "H160", "H175", "H225",
                "BK17", "MD52", "MD60", "MD90", "MD500", "MD600", "MD902",
                "GAZL", "W3", "AN2", "CLON", "EN48", "K126", "KMAX",
                "S51", "SC7", "A109", "A119", "A139", "A149", "A189"
            };

            static const vector<const char*> helicopterPrefixes = {
                "B06", "UH1", "UH60", "AH64", "OH58", "CH47", "CH53",
                "HH60", "MH60", "SH3", "SH60", "S70", "MD5", "MD6", "MD9"
            };

            static const vector<const char*> helicopterNameTokens = {
                "HELI", "ROTOR", "ROTORCRAFT", "SIKORSKY", "EUROCOPTER",
                "AIRBUSHELICOPTERS", "AGUSTA", "AEROSPATIALE", "DAUPHIN",
                "COLIBRI", "ECUREUIL", "JETRANGER", "LONGRANGER", "HUEY",
                "PUMA", "SUPERPUMA", "BLACKHAWK", "SEAHAWK", "CHINOOK",
                "STALLION", "COBRA", "APACHE", "ROBINSON", "BOEINGVERTOL",
                "VERTOL", "MIL", "MD500", "MD600", "MD902"
            };

            // Fighter aircraft (known military types)
            static const unordered_set<string> fighterTypes = {
                "F16", "F18", "FA18", "F15", "F22", "F35", "A10", "MIR2",
                "MIR4", "MIR5", "MIR6", "MG29", "MG31", "MG35", "S27",
                "S30", "S35", "S37", "EUFI", "EF35", "RAF", "HAR",
                "T2", "T38", "C101", "F104", "F4", "F5", "F14"
            };

            string key = normalizeIcao(icaoType);

            if (fighterTypes.find(key) != fighterTypes.end())
                return world::Aircraft::Category::Fighter;
            if (helicopterTypes.find(key) != helicopterTypes.end())
                return world::Aircraft::Category::Helicopter;
            for (const char* prefix : helicopterPrefixes)
            {
                if (prefix && *prefix && key.rfind(prefix, 0) == 0)
                {
                    return world::Aircraft::Category::Helicopter;
                }
            }

            AircraftTypeReferenceTable::Entry refEntry;
            if (AircraftTypeReferenceTable::tryFindByIcao(key, refEntry))
            {
                const string normalizedName = normalizeIcao(refEntry.name);
                if (containsAnyToken(normalizedName, helicopterNameTokens))
                {
                    return world::Aircraft::Category::Helicopter;
                }
            }

            if (lightPropTypes.find(key) != lightPropTypes.end())
                return world::Aircraft::Category::LightProp;
            if (propTypes.find(key) != propTypes.end())
                return world::Aircraft::Category::Prop;
            if (turbopropTypes.find(key) != turbopropTypes.end())
                return world::Aircraft::Category::Turboprop;

            // Default to Jet for commercial aircraft (A320, B737, etc.)
            return world::Aircraft::Category::Jet;
        }

    private:
        /// @brief Recursively scans CSL directories for xsb_aircraft.txt files and extracts ICAO types
        static void scanCSLDirectory(shared_ptr<HostServices> host, const string& path, vector<string>& icaoTypes)
        {
            try
            {
                // Try to list files in this directory
                vector<string> files = host->findFilesInHostDirectory({path});
                
                // Check if xsb_aircraft.txt exists here
                auto it = find(files.begin(), files.end(), "xsb_aircraft.txt");
                if (it != files.end())
                {
                    parseXsbAircraftFile(host, path, icaoTypes);
                }

                // Recurse into subdirectories
                for (const auto& file : files)
                {
                    string fullPath = path + "/" + file;
                    // Try to list it as a directory - if it succeeds, it's a directory
                    vector<string> subFiles = host->findFilesInHostDirectory({fullPath});
                    if (!subFiles.empty())
                    {
                        scanCSLDirectory(host, fullPath, icaoTypes);
                    }
                }
            }
            catch(const exception&)
            {
                // Directory doesn't exist or can't be read
            }
        }

        /// @brief Parses xsb_aircraft.txt file to extract ICAO aircraft types
        static void parseXsbAircraftFile(shared_ptr<HostServices> host, const string& dirPath, vector<string>& icaoTypes)
        {
            try
            {
                string filePath = dirPath + "/xsb_aircraft.txt";
                auto file = host->openFileForRead(filePath);

                string line;
                while (std::getline(*file, line))
                {
                    line = trim(line);
                    
                    // Look for OBJ8_AIRCRAFT line which defines a new aircraft
                    if (line.find("OBJ8_AIRCRAFT") == 0 || line.find("OBJ8_AIRCRAFT ") == 0)
                    {
                        // Next lines may contain ICAO definition
                        continue;
                    }
                    
                    // Look for ICAO line
                    if (line.find("ICAO ") == 0 || line.find("ICAO\t") == 0)
                    {
                        istringstream ss(line);
                        string keyword, icaoToken;
                        ss >> keyword >> icaoToken;
                        if (!icaoToken.empty())
                        {
                            string icao = normalizeIcao(icaoToken);
                            if (!icao.empty())
                            {
                                icaoTypes.push_back(icao);
                            }
                        }
                    }
                }
            }
            catch(const exception&)
            {
                // File couldn't be read
            }
        }

        static void ensureLoaded(shared_ptr<HostServices> host)
        {
            if (loaded())
            {
                return;
            }

            loaded() = true;
            if (host)
            {
                loadFromCsvResource(host, cache());

                // Discover CSL models from X-Plane installation and load Eurocontrol performance data
                // Try common CSL installation paths
                const vector<string> cslPaths = {
                    "Resources/CSL",
                    "../Resources/plugins/LiveTraffic/Resources/CSL",
                    "../Resources/plugins/XPMP2/Resources/CSL",
                    "../Resources/plugins/Bluebell/Resources/CSL"
                };

                for (const auto& path : cslPaths)
                {
                    discoverFighterCSLModels(host, path);
                    discoverGaCSLModels(host, path);
                    discoverHelicopterCSLModels(host, path);
                }
            }
        }

        static bool loadFromCsvResource(shared_ptr<HostServices> host, map<string, AircraftPerformanceProfile>& out)
        {
            try
            {
                const string resourcePath = host->getResourceFilePath({ "Resources", "aircraft-performance.csv" });
                auto file = host->openFileForRead(resourcePath);

                string line;
                int lineNo = 0;
                while (std::getline(*file, line))
                {
                    ++lineNo;
                    line = trim(line);
                    if (line.empty() || line[0] == '#')
                    {
                        continue;
                    }

                    vector<string> columns;
                    splitCsvLine(line, columns);
                    if (columns.size() < 2)
                    {
                        continue;
                    }

                    AircraftPerformanceProfile profile = defaultProfile(world::Aircraft::Category::Jet);
                    const string modelIcao = normalizeIcao(columns.at(0));
                    profile.approachSpeedKt = static_cast<float>(parseFloat(columns, 1, profile.approachSpeedKt));
                    profile.descentRateFpm = static_cast<float>(parseFloat(columns, 2, profile.descentRateFpm));
                    profile.minutesToThreshold = static_cast<float>(parseFloat(columns, 3, profile.minutesToThreshold));
                    profile.flareBufferFeet = static_cast<float>(parseFloat(columns, 4, profile.flareBufferFeet));
                    profile.hasEurocontrolData = true;
                    out[modelIcao] = profile;
                }

                if (!out.empty())
                {
                    host->writeLog("PERF|Loaded [%d] aircraft performance profiles from resource CSV", (int)out.size());
                    return true;
                }
            }
            catch(const exception&)
            {
            }

            return false;
        }

        static PerformanceDataSource loadPerformanceDetailsPage(shared_ptr<HostServices> host, const string& modelIcao, AircraftPerformanceProfile& out)
        {
            if (!host)
            {
                return PerformanceDataSource::None;
            }

            string html;
            if (!fetchUrlText(buildEurocontrolDetailsUrl(modelIcao), html))
            {
                host->writeLog("PERF|Eurocontrol details page fetch failed for [%s]", modelIcao.c_str());
            }
            else
            {
                AircraftPerformanceProfile profile = out;
                string text = stripMarkup(std::move(html));
                normalizeWhitespace(text);

                if (parseEurocontrolDetailsText(text, profile))
                {
                    profile.hasEurocontrolData = true;
                    out = profile;
                    return PerformanceDataSource::Eurocontrol;
                }

                host->writeLog("PERF|Eurocontrol details page did not yield parseable data for [%s]", modelIcao.c_str());
            }

            html.clear();
            if (!fetchUrlText(buildDoc8643DetailsUrl(modelIcao), html))
            {
                host->writeLog("PERF|Doc8643 details page fetch failed for [%s]", modelIcao.c_str());
                return PerformanceDataSource::None;
            }

            AircraftPerformanceProfile profile = out;
            string text = stripMarkup(std::move(html));
            normalizeWhitespace(text);

            if (!parseDoc8643DetailsText(text, profile))
            {
                host->writeLog("PERF|Doc8643 details page did not yield parseable data for [%s]", modelIcao.c_str());
                return PerformanceDataSource::None;
            }

            profile.hasEurocontrolData = true;
            out = profile;
            return PerformanceDataSource::Doc8643;
        }

        static string buildEurocontrolDetailsUrl(const string& modelIcao)
        {
            return "https://learningzone.eurocontrol.int/ilp/customs/ATCPFDB/details.aspx?ICAO=" + normalizeIcao(modelIcao);
        }

        static string buildDoc8643DetailsUrl(const string& modelIcao)
        {
            return "https://www.doc8643.com/aircraft/" + normalizeIcao(modelIcao);
        }

        static bool parseEurocontrolDetailsText(const string& text, AircraftPerformanceProfile& profile)
        {
            // Hard cap: do not run std::regex on more than 64 KB of text.
            // GCC's recursive NFA backtracker can overflow the 1 MB Windows thread stack
            // on large inputs when lazy wildcards (.*?) fail to match.
            const string& safeText = (text.size() > 65536) ? text.substr(0, 65536) : text;

            smatch match;
            bool parsedAny = false;

            const regex typePattern(
                R"(Type\s+([A-Z0-9]+)\s+APC\s+([A-Z])\s+WTC\s+([A-Z])\s+RECAT-EU\s+([A-Za-z0-9\- ]+))",
                regex_constants::icase);
            if (regex_search(safeText, match, typePattern))
            {
                profile.classification = trim(match[1].str());
                profile.wakeCategory = trim(match[3].str());
                parsedAny = true;
            }

            const regex approachSpeedPattern(R"(Approach\s+IAS:?[ ]*([0-9]+)\s*kts?)", regex_constants::icase);
            if (regex_search(safeText, match, approachSpeedPattern))
            {
                profile.approachSpeedKt = static_cast<float>(stoi(match[1].str()));
                parsedAny = true;
            }

            // Bounded span [^\n]{0,400}: prevents GCC's recursive NFA from stack-overflowing
            // on pages where the anchor text exists but the rest of the pattern does not follow.
            const regex approachDescentPattern(R"(Approach\s+IAS:?[ ]*[0-9]+\s*kts?[^\n]{0,400}ROD:?[ ]*([0-9]+)\s*ft/min)", regex_constants::icase);
            if (regex_search(safeText, match, approachDescentPattern))
            {
                profile.descentRateFpm = static_cast<float>(stoi(match[1].str()));
                parsedAny = true;
            }

            // Parse Initial Climb ROC (Rate of Climb) - key for unrestricted climbout capability
            const regex initialClimbRocPattern(R"(Initial\s+climb[^\n]{0,400}ROC\s*([0-9]+)\s*ft/min)", regex_constants::icase);
            if (regex_search(safeText, match, initialClimbRocPattern))
            {
                profile.initialClimbRocFpm = static_cast<float>(stoi(match[1].str()));
                parsedAny = true;
            }

            // Parse Range from Cruise section (e.g., "Range 430 NM")
            const regex rangePattern(R"(Range\s+([0-9]+)\s*NM)", regex_constants::icase);
            if (regex_search(safeText, match, rangePattern))
            {
                profile.rangeNm = static_cast<float>(stoi(match[1].str()));
                parsedAny = true;
            }

            // Parse Ceiling from Cruise section (e.g., "Ceiling FL 130")
            const regex ceilingPattern(R"(Ceiling\s+FL\s+([0-9]+))", regex_constants::icase);
            if (regex_search(safeText, match, ceilingPattern))
            {
                profile.ceilingFl = stoi(match[1].str());
                parsedAny = true;
            }

            const regex landingSpeedPattern(R"(Vat\s*\(IAS\):?\s*([0-9]+)\s*kts?)", regex_constants::icase);
            if (regex_search(safeText, match, landingSpeedPattern))
            {
                profile.flareBufferFeet = max(profile.flareBufferFeet, static_cast<float>(max(20, stoi(match[1].str()) / 4)));
                parsedAny = true;
            }

            // Parse Take-off Distance Required (TODR) - Eurocontrol provides this as "Distance XXXX m" in Take-off section
            // Bounded span prevents catastrophic NFA backtracking on large pages.
            const regex takeoffDistancePattern(
                R"(Take-off[^\n]{0,400}V2\s*\(IAS\)\s*[0-9]+\s*kts?\s*Distance\s*([0-9]+)\s*m)",
                regex_constants::icase);
            if (regex_search(safeText, match, takeoffDistancePattern))
            {
                profile.takeoffDistanceMeters = static_cast<float>(stoi(match[1].str()));
                parsedAny = true;
            }

            // Parse Landing Distance Required (LDR) - Eurocontrol provides this as "Distance XXXX m" in Landing section
            // Bounded span prevents catastrophic NFA backtracking on large pages.
            const regex landingDistancePattern(
                R"(Landing[^\n]{0,400}Vat\s*\(IAS\)\s*[0-9]+\s*kts?\s*Distance\s*([0-9]+)\s*m)",
                regex_constants::icase);
            if (regex_search(safeText, match, landingDistancePattern))
            {
                profile.landingDistanceMeters = static_cast<float>(stoi(match[1].str()));
                parsedAny = true;
            }

            return parsedAny;
        }

        static bool parseDoc8643DetailsText(const string& text, AircraftPerformanceProfile& profile)
        {
            // Hard cap: do not run std::regex on more than 64 KB of text.
            const string& safeText = (text.size() > 65536) ? text.substr(0, 65536) : text;

            smatch match;
            bool parsedAny = false;

            const regex classificationPattern(R"(\b([A-Z][0-9][A-Z])\s+([A-Z]/[A-Z])\b)", regex_constants::icase);
            if (regex_search(safeText, match, classificationPattern))
            {
                profile.classification = trim(match[1].str());
                profile.wakeCategory = trim(match[2].str());
                parsedAny = true;
            }

            const regex maxSpeedPattern(R"(Maximum\s+Speed\s*\(kts/M\)\s*([0-9]+)(?:\s*/\s*[0-9.]+)?)", regex_constants::icase);
            if (regex_search(safeText, match, maxSpeedPattern))
            {
                const float maxSpeedKt = static_cast<float>(stoi(match[1].str()));
                profile.approachSpeedKt = max(profile.approachSpeedKt, min(200.0f, maxSpeedKt * 0.22f));
                parsedAny = true;
            }

            const regex rangePattern(R"(Maximum\s+Range\s*\(Nm\)\s*([0-9]+))", regex_constants::icase);
            if (regex_search(safeText, match, rangePattern))
            {
                profile.rangeNm = static_cast<float>(stoi(match[1].str()));
                parsedAny = true;
            }

            const regex absoluteCeilingPattern(R"(Absolute\s+Ceiling\s*\(x100ft\)\s*([0-9]+))", regex_constants::icase);
            if (regex_search(safeText, match, absoluteCeilingPattern))
            {
                profile.ceilingFl = stoi(match[1].str());
                parsedAny = true;
            }
            else
            {
                const regex optimumCeilingPattern(R"(Optimum\s+Ceiling\s*\(x100ft\)\s*([0-9]+))", regex_constants::icase);
                if (regex_search(safeText, match, optimumCeilingPattern))
                {
                    profile.ceilingFl = stoi(match[1].str());
                    parsedAny = true;
                }
            }

            const regex maxClimbPattern(R"(Maximum\s+Climb\s+Rate\s*\(ft/min\)\s*([0-9]+))", regex_constants::icase);
            if (regex_search(safeText, match, maxClimbPattern))
            {
                profile.initialClimbRocFpm = static_cast<float>(stoi(match[1].str()));
                parsedAny = true;
            }

            return parsedAny;
        }

        static AircraftPerformanceProfile defaultProfile(world::Aircraft::Category category)
        {
            AircraftPerformanceProfile profile;

            switch (category)
            {
            case world::Aircraft::Category::Heavy:
                profile.approachSpeedKt = 160.0f;
                profile.descentRateFpm = 1200.0f;
                profile.minutesToThreshold = 15.0f;  // Increased from 4.5 for realistic descent from cruise
                profile.flareBufferFeet = 50.0f;
                profile.landingTouchdownDistanceMeters = 540.0f;
                profile.landingTouchdownSpeedKt = 148.0f;
                profile.landingExitSpeedKt = 35.0f;
                profile.landingRolloutDecelerationKtPerSecond = 4.8f;
                profile.takeoffRotateSpeedKt = 165.0f;
                profile.takeoffLiftOffSpeedKt = 178.0f;
                profile.takeoffInitialClimbSpeedKt = 205.0f;
                profile.takeoffAccelerationKtPerSecond = 6.0f;
                profile.takeoffRotatePitchDegrees = 12.0f;
                profile.rangeNm = 5500.0f;           // Long-haul aircraft (B777, A380, etc.)
                profile.ceilingFl = 430;             // Heavy jets typically FL 410-450
                break;
            case world::Aircraft::Category::Jet:
                profile.approachSpeedKt = 145.0f;
                profile.descentRateFpm = 1000.0f;
                profile.minutesToThreshold = 12.0f;  // Increased from 4.0 for realistic descent from cruise
                profile.flareBufferFeet = 40.0f;
                profile.landingTouchdownDistanceMeters = 380.0f;
                profile.landingTouchdownSpeedKt = 132.0f;
                profile.landingExitSpeedKt = 30.0f;
                profile.landingRolloutDecelerationKtPerSecond = 5.2f;
                profile.takeoffRotateSpeedKt = 140.0f;
                profile.takeoffLiftOffSpeedKt = 152.0f;
                profile.takeoffInitialClimbSpeedKt = 175.0f;
                profile.takeoffAccelerationKtPerSecond = 7.0f;
                profile.takeoffRotatePitchDegrees = 13.0f;
                profile.rangeNm = 3200.0f;           // Medium-haul jets (B737, A320 family)
                profile.ceilingFl = 410;             // Commercial jets typically FL 410
                break;
            case world::Aircraft::Category::Turboprop:
                profile.approachSpeedKt = 125.0f;
                profile.descentRateFpm = 700.0f;
                profile.minutesToThreshold = 10.0f;  // Increased from 3.5
                profile.flareBufferFeet = 35.0f;
                profile.landingTouchdownDistanceMeters = 250.0f;
                profile.landingTouchdownSpeedKt = 96.0f;
                profile.landingExitSpeedKt = 24.0f;
                profile.landingRolloutDecelerationKtPerSecond = 5.8f;
                profile.takeoffRotateSpeedKt = 102.0f;
                profile.takeoffLiftOffSpeedKt = 114.0f;
                profile.takeoffInitialClimbSpeedKt = 136.0f;
                profile.takeoffAccelerationKtPerSecond = 8.0f;
                profile.takeoffRotatePitchDegrees = 11.0f;
                profile.rangeNm = 1200.0f;           // Regional turboprops (ATR72, Dash8, etc.)
                profile.ceilingFl = 250;             // Turboprops typically FL 250
                break;
            case world::Aircraft::Category::Prop:
            case world::Aircraft::Category::LightProp:
                profile.approachSpeedKt = 95.0f;
                profile.descentRateFpm = 500.0f;
                profile.minutesToThreshold = 8.0f;  // Increased from 3.0
                profile.flareBufferFeet = 30.0f;
                profile.landingTouchdownDistanceMeters = 120.0f;
                profile.landingTouchdownSpeedKt = 60.0f;
                profile.landingExitSpeedKt = 15.0f;
                profile.landingRolloutDecelerationKtPerSecond = 4.0f;
                profile.takeoffRotateSpeedKt = 60.0f;
                profile.takeoffLiftOffSpeedKt = 70.0f;
                profile.takeoffInitialClimbSpeedKt = 88.0f;
                profile.takeoffAccelerationKtPerSecond = 5.0f;
                profile.takeoffRotatePitchDegrees = 9.0f;
                profile.rangeNm = 500.0f;            // GA aircraft (C172: ~430 NM, PA28: ~450 NM)
                profile.ceilingFl = 130;             // Light GA typically FL 100-150 (C172: FL 130)
                break;
            case world::Aircraft::Category::Helicopter:
                profile.approachSpeedKt = 70.0f;
                profile.descentRateFpm = 300.0f;
                profile.minutesToThreshold = 5.0f;  // Increased from 2.0
                profile.flareBufferFeet = 20.0f;
                profile.landingTouchdownDistanceMeters = 20.0f;
                profile.landingTouchdownSpeedKt = 18.0f;
                profile.landingExitSpeedKt = 0.0f;
                profile.landingRolloutDecelerationKtPerSecond = 4.0f;
                profile.takeoffRotateSpeedKt = 35.0f;
                profile.takeoffLiftOffSpeedKt = 45.0f;
                profile.takeoffInitialClimbSpeedKt = 70.0f;
                profile.takeoffAccelerationKtPerSecond = 4.0f;
                profile.takeoffRotatePitchDegrees = 6.0f;
                profile.rangeNm = 350.0f;            // Helicopters (EC35: ~340 NM, B412: ~400 NM)
                profile.ceilingFl = 200;             // Helicopters typically FL 150-200 (EC35: FL 200)
                break;
            case world::Aircraft::Category::Fighter:
                profile.approachSpeedKt = 170.0f;
                profile.descentRateFpm = 1500.0f;
                profile.minutesToThreshold = 6.0f;  // Increased from 3.0 (fighters descend fast but need more time)
                profile.flareBufferFeet = 50.0f;
                profile.landingTouchdownDistanceMeters = 640.0f;
                profile.landingTouchdownSpeedKt = 150.0f;
                profile.landingExitSpeedKt = 40.0f;
                profile.landingRolloutDecelerationKtPerSecond = 6.5f;
                profile.takeoffRotateSpeedKt = 150.0f;
                profile.takeoffLiftOffSpeedKt = 170.0f;
                profile.takeoffInitialClimbSpeedKt = 240.0f;
                profile.takeoffAccelerationKtPerSecond = 16.0f;
                profile.takeoffRotatePitchDegrees = 14.0f;
                profile.initialClimbRocFpm = 20000.0f;  // Default for fighters (F-16: 55000, F-18: 30000, A-10: 6000)
                profile.rangeNm = 1500.0f;           // Fighters (F-16: ~2000 NM, F-18: ~1500 NM)
                profile.ceilingFl = 500;             // Fighters very high ceiling (F-16: FL 500)
                break;
            default:
                profile.rangeNm = 500.0f;            // Default conservative range
                profile.ceilingFl = 250;             // Default conservative ceiling
                break;
            }

            return profile;
        }

        static AircraftPerformanceProfile defaultProfileFromClassification(const string& classification)
        {
            AircraftPerformanceProfile profile = defaultProfile(world::Aircraft::Category::Jet);
            if (classification.find('H') != string::npos)
            {
                profile = defaultProfile(world::Aircraft::Category::Helicopter);
            }
            else if (classification.find('T') != string::npos)
            {
                profile = defaultProfile(world::Aircraft::Category::Turboprop);
            }
            else if (classification.find('P') != string::npos)
            {
                profile = defaultProfile(world::Aircraft::Category::Prop);
            }

            profile.classification = classification;
            return profile;
        }

        static string normalizeIcao(string value)
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

        static string trim(const string& value)
        {
            size_t begin = 0;
            while (begin < value.size() && isspace(static_cast<unsigned char>(value.at(begin))))
            {
                ++begin;
            }

            size_t end = value.size();
            while (end > begin && isspace(static_cast<unsigned char>(value.at(end - 1))))
            {
                --end;
            }

            return value.substr(begin, end - begin);
        }

        static void splitCsvLine(const string& line, vector<string>& columns)
        {
            columns.clear();
            string current;
            bool inQuotes = false;

            for (size_t i = 0 ; i < line.size() ; ++i)
            {
                const char ch = line.at(i);
                if (ch == '"')
                {
                    inQuotes = !inQuotes;
                    continue;
                }

                if (ch == ',' && !inQuotes)
                {
                    columns.push_back(trim(current));
                    current.clear();
                    continue;
                }

                current.push_back(ch);
            }

            columns.push_back(trim(current));
        }

        static double parseFloat(const vector<string>& columns, size_t index, double fallback)
        {
            if (index >= columns.size())
            {
                return fallback;
            }

            const string& value = columns.at(index);
            if (value.empty())
            {
                return fallback;
            }

            char* endPtr = nullptr;
            double parsedValue = strtod(value.c_str(), &endPtr);
            return endPtr && *endPtr == '\0' ? parsedValue : fallback;
        }

        static string stripMarkup(string html)
        {
            replaceAll(html, "</tr>", "\n");
            replaceAll(html, "</p>", "\n");
            replaceAll(html, "<br>", "\n");
            replaceAll(html, "<br/>", "\n");
            replaceAll(html, "<br />", "\n");
            replaceAll(html, "&nbsp;", " ");
            replaceAll(html, "&amp;", "&");

            string text;
            text.reserve(html.size());
            bool inTag = false;

            for (size_t i = 0 ; i < html.size() ; ++i)
            {
                const char ch = html.at(i);
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
                    text.push_back(ch);
                }
            }

            return text;
        }

        static void normalizeWhitespace(string& text)
        {
            string result;
            result.reserve(text.size());
            bool previousWasSpace = false;

            for (char ch : text)
            {
                if (isspace(static_cast<unsigned char>(ch)))
                {
                    if (!previousWasSpace)
                    {
                        result.push_back(' ');
                        previousWasSpace = true;
                    }
                }
                else
                {
                    result.push_back(ch);
                    previousWasSpace = false;
                }
            }

            text.swap(result);
        }

        static void replaceAll(string& value, const string& from, const string& to)
        {
            if (from.empty())
            {
                return;
            }

            size_t start = 0;
            while ((start = value.find(from, start)) != string::npos)
            {
                value.replace(start, from.length(), to);
                start += to.length();
            }
        }

        static bool fetchUrlText(const string& url, string& responseText)
        {
            responseText.clear();
            const string command =
                "curl -g -L --fail --silent --show-error --compressed "
                "--connect-timeout 8 --max-time 20 --retry 2 --retry-delay 1 "
                "--user-agent \"Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
                "(KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36\" "
                "\"" + url + "\"";

#if IBM
            FILE* pipe = _popen(command.c_str(), "r");
#else
            FILE* pipe = popen(command.c_str(), "r");
#endif

            if (!pipe)
            {
                return false;
            }

            char buffer[4096];
            while (fgets(buffer, sizeof(buffer), pipe))
            {
                responseText.append(buffer);
            }

#if IBM
            const int exitCode = _pclose(pipe);
#else
            const int exitCode = pclose(pipe);
#endif

            return exitCode == 0 && !responseText.empty();
        }
    };
}