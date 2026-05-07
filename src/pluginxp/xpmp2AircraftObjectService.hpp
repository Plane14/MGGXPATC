// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <cmath>
#include <array>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <functional>
#include <mutex>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <utility>

// SDK
#include "XPLMPlugin.h"
#include "XPLMUtilities.h"

// XPMP2
#include "XPCAircraft.h"
#include "XPMPAircraft.h"
#include "XPMPMultiplayer.h"

// PPL 
#include "log.h"
#include "logwriter.h"
#include "menuitem.h"
#include "action.h"
#include "pluginpath.h"

// tnc
#include "utils.h"
#include "libworld.h"
#include "configuration.hpp"

using namespace std;
using namespace PPL;
using namespace XPMP2;
using namespace world;

namespace
{
    enum class AircraftSoundFamily
    {
        SingleA,
        SingleB,
        TwinA,
        TwinB,
        TurbopropB,
        TurbopropC,
        JetB,
        JetC,
        JetD,
        JetE,
        JetF,
        Fighter,
        Helicopter
    };

    enum class AircraftSoundSituation
    {
        Start,
        Tow,
        Taxi,
        Toga,
        Max,
        Back
    };

    struct Doc8643SoundInfo
    {
        std::string typeClass;
        std::string wakeCategory;
    };

    struct AircraftSoundCatalogState
    {
        bool tablesLoaded = false;
        bool relatedLoaded = false;
        bool soundsRegistered = false;
        std::mutex mutex;
        std::unordered_map<std::string, Doc8643SoundInfo> doc8643ByIcao;
        std::unordered_set<std::string> jet4;
        std::unordered_set<std::string> jet3;
        std::unordered_set<std::string> jet2;
        std::unordered_set<std::string> regionalJets;
        std::unordered_set<std::string> businessJets;
        std::unordered_set<std::string> commuters;
        std::unordered_set<std::string> twinProps;
        std::unordered_set<std::string> smallProps;
        std::unordered_set<std::string> tailDraggers;
        std::unordered_set<std::string> biplanes;
        std::unordered_set<std::string> gliders;
        std::unordered_set<std::string> helicopters;
        std::set<AircraftSoundFamily> availableFamilies;
        bool specialSoundsRegistered = false;
        bool fighterSpecialRegistered = false;
    };

    AircraftSoundCatalogState& aircraftSoundCatalogState()
    {
        static AircraftSoundCatalogState state;
        return state;
    }

    std::string trimCopy(std::string value)
    {
        auto notSpace = [](int ch) { return !std::isspace(ch); };
        value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
        value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
        return value;
    }

    std::string upperCopy(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
            return static_cast<char>(std::toupper(ch));
        });
        return value;
    }

    bool startsWith(const std::string& value, const std::string& prefix)
    {
        return value.rfind(prefix, 0) == 0;
    }

    bool containsCode(const std::unordered_set<std::string>& values, const std::string& code)
    {
        return values.find(upperCopy(code)) != values.end();
    }

    std::unordered_set<std::string>* relatedSetForHeader(std::string header)
    {
        auto& state = aircraftSoundCatalogState();
        header = upperCopy(trimCopy(std::move(header)));

        if (header.find("JET 4 ENGINE - L4J") != std::string::npos)
        {
            return &state.jet4;
        }
        if (header.find("JET 3 ENGINE - L3J") != std::string::npos)
        {
            return &state.jet3;
        }
        if (header.find("JET 2 ENGINE - L2J") != std::string::npos)
        {
            return &state.jet2;
        }
        if (header.find("REGIONAL JETS - L2J") != std::string::npos)
        {
            return &state.regionalJets;
        }
        if (header.find("BUSINESS JETS - L2J") != std::string::npos)
        {
            return &state.businessJets;
        }
        if (header.find("COMMUTERS - L2T") != std::string::npos)
        {
            return &state.commuters;
        }
        if (header.find("TWINPROPS - L2P") != std::string::npos)
        {
            return &state.twinProps;
        }
        if (header.find("SMALL PROPS / GA") != std::string::npos)
        {
            return &state.smallProps;
        }
        if (header.find("GA 1-MOT TAIL DRAGGER") != std::string::npos)
        {
            return &state.tailDraggers;
        }
        if (header.find("BIPLANE") != std::string::npos)
        {
            return &state.biplanes;
        }
        if (header.find("MOTORIZED SAILPLANES / GLIDERS") != std::string::npos)
        {
            return &state.gliders;
        }
        if (header.find("HELICOPTERS") != std::string::npos)
        {
            return &state.helicopters;
        }

        return nullptr;
    }

    const char* familyName(AircraftSoundFamily family)
    {
        switch (family)
        {
        case AircraftSoundFamily::SingleA:      return "SingleA";
        case AircraftSoundFamily::SingleB:      return "SingleB";
        case AircraftSoundFamily::TwinA:        return "TwinA";
        case AircraftSoundFamily::TwinB:        return "TwinB";
        case AircraftSoundFamily::TurbopropB:   return "TurbopropB";
        case AircraftSoundFamily::TurbopropC:   return "TurbopropC";
        case AircraftSoundFamily::JetB:         return "JetB";
        case AircraftSoundFamily::JetC:         return "JetC";
        case AircraftSoundFamily::JetD:         return "JetD";
        case AircraftSoundFamily::JetE:         return "JetE";
        case AircraftSoundFamily::JetF:         return "JetF";
        case AircraftSoundFamily::Fighter:      return "Fighter";
        case AircraftSoundFamily::Helicopter:   return "Helicopter";
        default:                                return "";
        }
    }

    const char* situationName(AircraftSoundSituation situation)
    {
        switch (situation)
        {
        case AircraftSoundSituation::Start:  return "start";
        case AircraftSoundSituation::Tow:    return "tow";
        case AircraftSoundSituation::Taxi:   return "taxi";
        case AircraftSoundSituation::Toga:   return "toga";
        case AircraftSoundSituation::Max:    return "max";
        case AircraftSoundSituation::Back:   return "back";
        default:                            return "";
        }
    }

    std::vector<std::string> makeSoundPathParts(const char* family, const char* fileName)
    {
        return { "sounds", family, fileName };
    }

    bool registerSoundFile(
        const std::shared_ptr<HostServices>& host,
        const std::string& soundName,
        const std::vector<std::string>& relativePathParts,
        bool loop,
        bool jetCone = false)
    {
        const std::string filePath = host->getResourceFilePath(relativePathParts);
        const char* error = jetCone
            ? XPMPSoundAdd(soundName.c_str(), filePath.c_str(), loop, 180.0f, 0.0f, 30.0f, 60.0f, 0.5f)
            : XPMPSoundAdd(soundName.c_str(), filePath.c_str(), loop);

        if (error && error[0])
        {
            host->writeLog(
                "XPMSND|sound[%s] file[%s] unavailable: %s",
                soundName.c_str(),
                filePath.c_str(),
                error);
            return false;
        }

        return true;
    }

    const Doc8643SoundInfo* lookupDoc8643Info(const std::string& modelIcao)
    {
        auto& state = aircraftSoundCatalogState();
        const auto found = state.doc8643ByIcao.find(upperCopy(modelIcao));
        if (found == state.doc8643ByIcao.end())
        {
            return nullptr;
        }
        return &found->second;
    }

    void loadRelatedTables(const std::shared_ptr<HostServices>& host)
    {
        auto& state = aircraftSoundCatalogState();
        std::lock_guard<std::mutex> guard(state.mutex);
        if (state.relatedLoaded)
        {
            return;
        }

        try
        {
            auto file = host->openFileForRead(host->getResourceFilePath({ "Resources", "related.txt" }));
            std::string line;
            std::unordered_set<std::string>* currentSet = nullptr;

            while (std::getline(*file, line))
            {
                const std::string trimmed = trimCopy(line);
                if (trimmed.empty())
                {
                    continue;
                }

                if (trimmed.front() == ';')
                {
                    currentSet = relatedSetForHeader(trimmed);
                    continue;
                }

                if (!currentSet)
                {
                    continue;
                }

                std::stringstream row(trimmed);
                std::string code;
                while (row >> code)
                {
                    currentSet->insert(upperCopy(trimCopy(code)));
                }
            }

            state.relatedLoaded = true;
            host->writeLog("XPMSND|loaded aircraft grouping table related.txt");
        }
        catch (const std::exception& e)
        {
            host->writeLog("XPMSND|failed to load related.txt: %s", e.what());
        }
    }

    bool hasCustomSoundPackForFamily(AircraftSoundFamily family)
    {
        const auto& state = aircraftSoundCatalogState();
        return state.availableFamilies.find(family) != state.availableFamilies.end();
    }

    bool hasFighterSpecialSounds()
    {
        return aircraftSoundCatalogState().fighterSpecialRegistered;
    }

    std::vector<AircraftSoundSituation> situationsForFamily(AircraftSoundFamily family)
    {
        switch (family)
        {
        case AircraftSoundFamily::SingleA:
        case AircraftSoundFamily::SingleB:
        case AircraftSoundFamily::TwinA:
        case AircraftSoundFamily::TwinB:
        case AircraftSoundFamily::TurbopropB:
        case AircraftSoundFamily::TurbopropC:
            return { AircraftSoundSituation::Start, AircraftSoundSituation::Tow, AircraftSoundSituation::Taxi };
        case AircraftSoundFamily::JetB:
        case AircraftSoundFamily::JetC:
        case AircraftSoundFamily::JetD:
        case AircraftSoundFamily::JetE:
        case AircraftSoundFamily::JetF:
            return { AircraftSoundSituation::Start, AircraftSoundSituation::Tow, AircraftSoundSituation::Taxi,
                     AircraftSoundSituation::Toga, AircraftSoundSituation::Max, AircraftSoundSituation::Back };
        default:
            return {};
        }
    }

    AircraftSoundFamily chooseSoundFamily(const std::shared_ptr<world::Aircraft>& source)
    {
        if (!source)
        {
            return AircraftSoundFamily::JetC;
        }

        const std::string modelIcao = upperCopy(source->modelIcao());
        const Doc8643SoundInfo* docInfo = lookupDoc8643Info(modelIcao);
        const std::string typeClass = docInfo ? docInfo->typeClass : std::string();
        const std::string wakeCategory = docInfo ? docInfo->wakeCategory : std::string();
        const bool wakeLight = wakeCategory == "L" || wakeCategory == "-";
        const bool wakeMedium = wakeCategory == "M";
        const auto isIn = [&](const std::unordered_set<std::string>& values) {
            return containsCode(values, modelIcao);
        };

        if (source->category() == world::Aircraft::Category::Fighter)
        {
            return AircraftSoundFamily::Fighter;
        }

        if (startsWith(typeClass, "H"))
        {
            return AircraftSoundFamily::Helicopter;
        }

        if (startsWith(typeClass, "L1T") || isIn(aircraftSoundCatalogState().commuters))
        {
            return wakeLight ? AircraftSoundFamily::SingleB : AircraftSoundFamily::TurbopropB;
        }

        if (startsWith(typeClass, "L2T"))
        {
            if (isIn(aircraftSoundCatalogState().commuters) || wakeMedium)
            {
                return AircraftSoundFamily::TurbopropC;
            }
            return wakeLight ? AircraftSoundFamily::TwinB : AircraftSoundFamily::TurbopropB;
        }

        if (startsWith(typeClass, "L2P") || startsWith(typeClass, "A2P") || isIn(aircraftSoundCatalogState().twinProps))
        {
            return wakeLight ? AircraftSoundFamily::TwinA : AircraftSoundFamily::TwinB;
        }

        if (startsWith(typeClass, "L1P") || startsWith(typeClass, "A1P") || startsWith(typeClass, "G1P") || isIn(aircraftSoundCatalogState().smallProps))
        {
            if (isIn(aircraftSoundCatalogState().tailDraggers) || isIn(aircraftSoundCatalogState().biplanes) || isIn(aircraftSoundCatalogState().gliders))
            {
                return AircraftSoundFamily::SingleA;
            }

            return wakeLight ? AircraftSoundFamily::SingleA : AircraftSoundFamily::SingleB;
        }

        if (startsWith(typeClass, "L1J"))
        {
            return AircraftSoundFamily::JetB;
        }

        if (startsWith(typeClass, "L2J"))
        {
            if (isIn(aircraftSoundCatalogState().businessJets))
            {
                return AircraftSoundFamily::JetB;
            }
            if (isIn(aircraftSoundCatalogState().regionalJets) || isIn(aircraftSoundCatalogState().jet2))
            {
                return AircraftSoundFamily::JetC;
            }
            return wakeLight ? AircraftSoundFamily::JetB : AircraftSoundFamily::JetC;
        }

        if (startsWith(typeClass, "L3J"))
        {
            return AircraftSoundFamily::JetD;
        }

        if (startsWith(typeClass, "L4J"))
        {
            return AircraftSoundFamily::JetE;
        }

        if (isIn(aircraftSoundCatalogState().jet4))
        {
            return AircraftSoundFamily::JetE;
        }
        if (isIn(aircraftSoundCatalogState().jet3))
        {
            return AircraftSoundFamily::JetD;
        }
        if (isIn(aircraftSoundCatalogState().jet2) || isIn(aircraftSoundCatalogState().regionalJets) || isIn(aircraftSoundCatalogState().businessJets))
        {
            return wakeLight ? AircraftSoundFamily::JetB : AircraftSoundFamily::JetC;
        }

        return wakeMedium ? AircraftSoundFamily::JetC : AircraftSoundFamily::JetB;
    }

    void loadDoc8643Tables(const std::shared_ptr<HostServices>& host)
    {
        auto& state = aircraftSoundCatalogState();
        std::lock_guard<std::mutex> guard(state.mutex);
        if (state.tablesLoaded)
        {
            return;
        }

        try
        {
            auto file = host->openFileForRead(host->getResourceFilePath({ "Resources", "Doc8643.txt" }));
            std::string line;
            while (std::getline(*file, line))
            {
                const std::string trimmed = trimCopy(line);
                if (trimmed.empty() || trimmed.front() == ';')
                {
                    continue;
                }

                std::vector<std::string> columns;
                std::stringstream row(trimmed);
                std::string column;
                while (std::getline(row, column, '\t'))
                {
                    columns.push_back(trimCopy(column));
                }

                if (columns.size() < 5)
                {
                    continue;
                }

                const std::string icao = upperCopy(columns[2]);
                if (icao.empty())
                {
                    continue;
                }

                state.doc8643ByIcao[icao] = { upperCopy(columns[3]), upperCopy(columns[4]) };
            }

            state.tablesLoaded = true;
            host->writeLog("XPMSND|loaded %zu Doc8643 type rows", state.doc8643ByIcao.size());
        }
        catch (const std::exception& e)
        {
            host->writeLog("XPMSND|failed to load Doc8643 type tables: %s", e.what());
        }
    }

    bool registerFamilySounds(
        const std::shared_ptr<HostServices>& host,
        AircraftSoundFamily family,
        bool jetCone)
    {
        bool allLoaded = true;
        const char* familyDir = familyName(family);
        const auto situations = situationsForFamily(family);

        for (const auto situation : situations)
        {
            const std::string soundName = std::string(familyDir) + "." + situationName(situation);
            const std::string fileName = std::string(situationName(situation)) + ".wav";
            const bool isLoop = situation == AircraftSoundSituation::Taxi;
            allLoaded = registerSoundFile(
                host,
                soundName,
                makeSoundPathParts(familyDir, fileName.c_str()),
                isLoop,
                jetCone) && allLoaded;
        }

        if (allLoaded)
        {
            aircraftSoundCatalogState().availableFamilies.insert(family);
        }

        return allLoaded;
    }

    bool registerSpecialSounds(const std::shared_ptr<HostServices>& host)
    {
        bool allLoaded = true;

        // Register the absolute-path-based fallbacks used when the more specific family packs are unavailable.
        allLoaded = registerSoundFile(host, "jet.small",        { "sounds", "turbofan_engineMsmall.wav" }, true, true) && allLoaded;
        allLoaded = registerSoundFile(host, "jet.mid",          { "sounds", "turbofan_engineM.wav" }, true, true) && allLoaded;
        allLoaded = registerSoundFile(host, "jet.big",          { "sounds", "turbofan_engineMbig.wav" }, true, true) && allLoaded;
        allLoaded = registerSoundFile(host, "jet.high",         { "sounds", "turbofan_engineH.wav" }, true, true) && allLoaded;
        allLoaded = registerSoundFile(host, "prop.single",      { "sounds", "prop.wav" }, true, false) && allLoaded;
        allLoaded = registerSoundFile(host, "prop.double",      { "sounds", "prop2.wav" }, true, false) && allLoaded;
        allLoaded = registerSoundFile(host, "turboprop.prop",   { "sounds", "turboprop_prop.wav" }, true, false) && allLoaded;
        allLoaded = registerSoundFile(host, "turboprop.turbine", { "sounds", "turboprop_turbine.wav" }, true, false) && allLoaded;
        allLoaded = registerSoundFile(host, "helicopter.engine", { "sounds", "helo.wav" }, true, false) && allLoaded;
        allLoaded = registerSoundFile(host, "fighter.engine.low",  { "sounds", "fighter_engineL.wav" }, true, true) && allLoaded;
        allLoaded = registerSoundFile(host, "fighter.engine.mid",  { "sounds", "fighter_engineM.wav" }, true, true) && allLoaded;
        allLoaded = registerSoundFile(host, "fighter.engine.high", { "sounds", "fighter_engineH.wav" }, true, true) && allLoaded;
        allLoaded = registerSoundFile(host, "fighter.burner",      { "sounds", "burner.wav" }, true, true) && allLoaded;
        allLoaded = registerSoundFile(host, "fighter.burner.ext",  { "sounds", "burner_ext.wav" }, true, true) && allLoaded;
        allLoaded = registerSoundFile(host, "fighter.sonicboom",   { "sounds", "sonicboom2.wav" }, false, false) && allLoaded;

        if (allLoaded)
        {
            aircraftSoundCatalogState().specialSoundsRegistered = true;
            aircraftSoundCatalogState().fighterSpecialRegistered = true;
        }

        return allLoaded;
    }

    void ensureAircraftSoundCatalog(const std::shared_ptr<HostServices>& host)
    {
        if (!host)
        {
            return;
        }

        loadDoc8643Tables(host);
        loadRelatedTables(host);

        if (!XPMPSoundIsEnabled())
        {
            return;
        }

        auto& state = aircraftSoundCatalogState();
        std::lock_guard<std::mutex> guard(state.mutex);
        if (state.soundsRegistered)
        {
            return;
        }

        bool anyLoaded = false;
        anyLoaded = registerFamilySounds(host, AircraftSoundFamily::SingleA, false) || anyLoaded;
        anyLoaded = registerFamilySounds(host, AircraftSoundFamily::SingleB, false) || anyLoaded;
        anyLoaded = registerFamilySounds(host, AircraftSoundFamily::TwinA, false) || anyLoaded;
        anyLoaded = registerFamilySounds(host, AircraftSoundFamily::TwinB, false) || anyLoaded;
        anyLoaded = registerFamilySounds(host, AircraftSoundFamily::TurbopropB, false) || anyLoaded;
        anyLoaded = registerFamilySounds(host, AircraftSoundFamily::TurbopropC, false) || anyLoaded;
        anyLoaded = registerFamilySounds(host, AircraftSoundFamily::JetB, true) || anyLoaded;
        anyLoaded = registerFamilySounds(host, AircraftSoundFamily::JetC, true) || anyLoaded;
        anyLoaded = registerFamilySounds(host, AircraftSoundFamily::JetD, true) || anyLoaded;
        anyLoaded = registerFamilySounds(host, AircraftSoundFamily::JetE, true) || anyLoaded;
        anyLoaded = registerFamilySounds(host, AircraftSoundFamily::JetF, true) || anyLoaded;
        anyLoaded = registerSpecialSounds(host) || anyLoaded;

        if (anyLoaded)
        {
            state.soundsRegistered = true;
            host->writeLog("XPMSND|registered custom aircraft sound packs");
        }
    }
}

class Xpmp2AircraftObjectService;

class Xpmp2AircraftObject : public XPMP2::Aircraft
{
private:
    shared_ptr<HostServices> m_host;
    shared_ptr<Flight> m_flight;
    shared_ptr<PluginConfiguration> m_config;
    World::OnChangesCallback m_onQueryChanges;
    int m_frameCount;
    float m_departurePhaseElapsedSec;
    Flight::Phase m_lastObservedPhase;
    bool m_afterburnerActive;
    uint64_t m_taxiLoopId;
    bool m_startCuePlayed;
    bool m_towCuePlayed;
    bool m_togaCuePlayed;
    bool m_maxCuePlayed;
    bool m_backCuePlayed;
    bool m_burnerCuePlayed;
    bool m_sonicBoomCuePlayed;
    bool m_visualStateInitialized;
    bool m_engineStateInitialized;
    float m_smoothedHeading;
    float m_smoothedPitch;
    float m_smoothedRoll;
    float m_smoothedThrustRatio;
    float m_smoothedReverseRatio;
    float m_smoothedEngineRpm;
    float m_smoothedPropRpm;
public:
    Xpmp2AircraftObject(
        shared_ptr<HostServices> _host,
        const shared_ptr<Flight>& _flight
    ) : Aircraft(
            _flight->aircraft()->modelIcao(),
            _flight->aircraft()->airlineIcao(),
            "",
            4333 + _flight->id(), // mode-S id
            ""
        ),
        m_host(std::move(_host)),
        m_flight(_flight),
        m_onQueryChanges(World::onChangesUnassigned),
        m_frameCount(0),
        m_departurePhaseElapsedSec(0.0f),
        m_lastObservedPhase(_flight->phase()),
        m_afterburnerActive(false),
        m_taxiLoopId(0),
        m_startCuePlayed(false),
        m_towCuePlayed(false),
        m_togaCuePlayed(false),
        m_maxCuePlayed(false),
        m_backCuePlayed(false),
        m_burnerCuePlayed(false),
        m_sonicBoomCuePlayed(false),
        m_visualStateInitialized(false),
        m_engineStateInitialized(false),
        m_smoothedHeading(0.0f),
        m_smoothedPitch(0.0f),
        m_smoothedRoll(0.0f),
        m_smoothedThrustRatio(0.0f),
        m_smoothedReverseRatio(0.0f),
        m_smoothedEngineRpm(0.0f),
        m_smoothedPropRpm(0.0f)
    {
        m_config = m_host->services().get<PluginConfiguration>();
        ensureAircraftSoundCatalog(m_host);

        auto source = m_flight->aircraft();
        auto location = source->location();

        // Label
        label = getLabelText(source);
        colLabel[0] = 0.0f;             // green
        colLabel[1] = 1.0f;
        colLabel[2] = 0.0f;

        // Radar
        acRadar.code = 4333 + _flight->id();
        acRadar.mode = xpmpTransponderMode_ModeC;

        // informational texts
        strcpy(acInfoTexts.icaoAcType, source->modelIcao().c_str());
        strcpy(acInfoTexts.icaoAirline, source->airlineIcao().c_str());
        strcpy(acInfoTexts.tailNum, m_flight->callSign().c_str());

        m_smoothedHeading = static_cast<float>(source->attitude().heading());
        m_smoothedPitch = static_cast<float>(source->attitude().pitch());
        m_smoothedRoll = static_cast<float>(source->attitude().roll());

        SetLocation(location.latitude, location.longitude, 0.0f);
        SetHeading(source->attitude().heading());
        SetPitch(0.0f);
        SetRoll(0.0f);
        SetGearRatio(1.0f);
        SetFlapRatio(0.0f);
        SetSlatRatio(0.0f);
        SetSpoilerRatio(0);
        SetSpeedbrakeRatio(0);
        SetWingSweepRatio(0.0f);
        ensureEngineRunning(source, false);
        SetYokePitchRatio(0.0f);
        SetYokeHeadingRatio(0.0f);
        SetYokeRollRatio(0.0f);
        SetLightsBeacon(false);
        SetLightsTaxi(false);
        SetLightsStrobe(false);
        SetLightsLanding(false);
        SetLightsNav(false);
        SetTouchDown(false);

        float groundPitch = 0.0f;
        safeClampToGround(groundPitch);
        SetPitch(groundPitch);
        m_smoothedPitch = groundPitch;
        m_visualStateInitialized = true;
    }

    void UpdatePosition(float elapsedSinceLastCall, int) override
    {
        //m_host->writeLog("Xpmp2AircraftObject::UpdatePosition - enter");
        try
        {
            safeUpdatePosition(elapsedSinceLastCall);
        }
        catch (const exception& e)
        {
            m_host->writeLog("Xpmp2AircraftObject::UpdatePosition CRASHED!!! %s", e.what());
        }
        //m_host->writeLog("Xpmp2AircraftObject::UpdatePosition - exit");
    }

    float GetAoA() const override
    {
        auto source = m_flight ? m_flight->aircraft() : nullptr;
        if (!source)
        {
            return XPMP2::Aircraft::GetAoA();
        }

        const double groundSpeedFeetPerSecond = max(45.0, abs(source->groundSpeedKt()) * 1.6878098571);
        const double verticalSpeedFeetPerSecond = source->verticalSpeedFpm() / 60.0;
        const double flightPathAngleDegrees = GeoMath::radiansToDegrees(atan2(verticalSpeedFeetPerSecond, groundSpeedFeetPerSecond));
        double aoaDegrees = source->attitude().pitch() - flightPathAngleDegrees;

        if (source->altitude().isGroundBased() && abs(source->groundSpeedKt()) < 35.0)
        {
            aoaDegrees = 0.0;
        }

        return static_cast<float>(max(-3.0, min(18.0, aoaDegrees)));
    }

    float GetLift() const override
    {
        auto source = m_flight ? m_flight->aircraft() : nullptr;
        const float aircraftWeightNewton = GetMass() * G_EARTH;
        if (!source || aircraftWeightNewton <= 0.0f)
        {
            return XPMP2::Aircraft::GetLift();
        }

        const float groundSpeedKt = static_cast<float>(abs(source->groundSpeedKt()));
        const float verticalSpeedFpm = static_cast<float>(source->verticalSpeedFpm());
        const float spoilerState = source->spoilerState();
        const float gearState = source->gearState();
        const float flapState = source->flapState();
        float liftFactor = 1.0f;

        if (source->altitude().isGroundBased())
        {
            if (groundSpeedKt < 35.0f)
            {
                return 0.0f;
            }

            liftFactor = clampRatio((groundSpeedKt - 35.0f) / 85.0f + flapState * 0.30f);
            if (spoilerState > 0.10f)
            {
                liftFactor *= max(0.0f, 1.0f - spoilerState * 0.80f);
            }

            return aircraftWeightNewton * min(1.02f, liftFactor);
        }

        const float aglFeet = altitudeAglFeet(source);
        liftFactor *= 1.0f - gearState * 0.08f;
        liftFactor *= 1.0f - spoilerState * 0.45f;
        liftFactor *= 1.0f + min(0.16f, max(-0.10f, verticalSpeedFpm / 18000.0f));

        if (aglFeet < 250.0f && verticalSpeedFpm < -200.0f)
        {
            liftFactor *= 0.90f;
        }
        else if (aglFeet < 600.0f && verticalSpeedFpm > 600.0f)
        {
            liftFactor *= 1.05f;
        }

        if (groundSpeedKt < 110.0f)
        {
            liftFactor *= max(0.45f, groundSpeedKt / 110.0f);
        }

        return aircraftWeightNewton * max(0.20f, min(1.22f, liftFactor));
    }

    void onQueryChanges(World::OnChangesCallback callback)
    {
        m_onQueryChanges = callback;
    }

    std::string SoundGetName(SoundEventsTy sndEvent, float& volAdj) const override
    {
        ensureAircraftSoundCatalog(m_host);

        auto source = m_flight->aircraft();
        if (!source)
        {
            return XPMP2::Aircraft::SoundGetName(sndEvent, volAdj);
        }

        const auto family = chooseSoundFamily(source);

        if (sndEvent == SND_ENG)
        {
            volAdj = max(volAdj, familyEngineVolumeFloor(family));

            if (family == AircraftSoundFamily::Helicopter)
            {
                return aircraftSoundCatalogState().specialSoundsRegistered ? std::string("helicopter.engine") : std::string(XP_SOUND_PROP_HELI);
            }

            if (family == AircraftSoundFamily::Fighter)
            {
                if (!hasFighterSpecialSounds())
                {
                    return XP_SOUND_LOBYPASSJET;
                }

                if (m_afterburnerActive)
                {
                    volAdj = max(volAdj, 1.25f);
                    return "fighter.engine.high"; // Afterburner active
                }

                return "fighter.engine.high";
            }

            if (aircraftSoundCatalogState().specialSoundsRegistered)
            {
                switch (family)
                {
                case AircraftSoundFamily::SingleA:
                    return "prop.single";
                case AircraftSoundFamily::SingleB:
                    return "prop.double";
                case AircraftSoundFamily::TwinA:
                    return "prop.double";
                case AircraftSoundFamily::TwinB:
                    return "turboprop.prop";
                case AircraftSoundFamily::TurbopropB:
                    return "turboprop.prop";
                case AircraftSoundFamily::TurbopropC:
                    return "turboprop.turbine";
                case AircraftSoundFamily::JetB:
                    return "jet.small";
                case AircraftSoundFamily::JetC:
                    return "jet.mid";
                case AircraftSoundFamily::JetD:
                    return "jet.big";
                case AircraftSoundFamily::JetE:
                case AircraftSoundFamily::JetF:
                    return "jet.high";
                default:
                    break;
                }
            }
        }
        else if (sndEvent == SND_REVERSE_THRUST)
        {
            if (family == AircraftSoundFamily::JetB || family == AircraftSoundFamily::JetC ||
                family == AircraftSoundFamily::JetD || family == AircraftSoundFamily::JetE ||
                family == AircraftSoundFamily::JetF)
            {
                if (hasCustomSoundPackForFamily(family))
                {
                    return std::string(familyName(family)) + ".back";
                }
            }
        }

        return XPMP2::Aircraft::SoundGetName(sndEvent, volAdj);
    }

private:

    std::string familySoundName(AircraftSoundFamily family, AircraftSoundSituation situation) const
    {
        const char* familyDir = familyName(family);
        const char* situationDir = situationName(situation);
        if (!familyDir[0] || !situationDir[0])
        {
            return "";
        }

        return std::string(familyDir) + "." + situationDir;
    }

    void playFamilyCue(AircraftSoundFamily family, AircraftSoundSituation situation, float volume = 1.0f)
    {
        if (!hasCustomSoundPackForFamily(family))
        {
            return;
        }

        const std::string soundName = familySoundName(family, situation);
        if (!soundName.empty())
        {
            SoundPlay(soundName, volume);
        }
    }

    void updateTaxiLoop(const shared_ptr<world::Aircraft>& source, AircraftSoundFamily family)
    {
        if (!hasCustomSoundPackForFamily(family))
        {
            if (m_taxiLoopId)
            {
                SoundStop(m_taxiLoopId);
                m_taxiLoopId = 0;
            }
            return;
        }

        const bool isGroundBased = source->altitude().isGroundBased();
        const float groundSpeedKt = static_cast<float>(std::fabs(source->groundSpeedKt()));
        const bool shouldTaxi = isGroundBased && groundSpeedKt > 1.5f && groundSpeedKt < 50.0f;

        if (shouldTaxi)
        {
            const float baseVolume = familyTaxiLoopBaseVolume(family);
            if (!m_taxiLoopId)
            {
                m_taxiLoopId = SoundPlay(familySoundName(family, AircraftSoundSituation::Taxi), baseVolume);
            }

            if (m_taxiLoopId)
            {
                const float taxiVolume = max(baseVolume, min(1.0f, baseVolume + (groundSpeedKt / 65.0f)));
                SoundVolume(m_taxiLoopId, taxiVolume);
            }
        }
        else if (m_taxiLoopId)
        {
            SoundStop(m_taxiLoopId);
            m_taxiLoopId = 0;
        }
    }

    void triggerSituationalSounds(const shared_ptr<world::Aircraft>& source, bool touchedDown)
    {
        if (!source)
        {
            return;
        }

        const auto family = chooseSoundFamily(source);
        const bool isGroundBased = source->altitude().isGroundBased();
        const float groundSpeedKt = static_cast<float>(std::fabs(source->groundSpeedKt()));
        const float reverseRatio = GetThrustReversRatio();
        const float thrustRatio = GetThrustRatio();

        updateTaxiLoop(source, family);

        if (family == AircraftSoundFamily::Fighter)
        {
            const float machNumber = estimateMachNumber(source);
            const bool airborne = !source->altitude().isGroundBased();

            if (m_afterburnerActive && !m_burnerCuePlayed)
            {
                if (hasFighterSpecialSounds())
                {
                    SoundPlay("fighter.burner", 1.0f);
                }
                m_burnerCuePlayed = true;
            }

            if (airborne && machNumber > 1.0f && !m_sonicBoomCuePlayed)
            {
                if (hasFighterSpecialSounds())
                {
                    SoundPlay("fighter.sonicboom", 1.0f);
                }
                m_sonicBoomCuePlayed = true;
            }
            else if (machNumber < 0.98f)
            {
                m_sonicBoomCuePlayed = false;
            }

            if (!m_afterburnerActive)
            {
                m_burnerCuePlayed = false;
            }
            return;
        }

        if (m_lastObservedPhase == Flight::Phase::Departure && isGroundBased)
        {
            if (!m_startCuePlayed)
            {
                playFamilyCue(family, AircraftSoundSituation::Start, 0.95f);
                m_startCuePlayed = true;
            }

            if (!m_towCuePlayed && m_departurePhaseElapsedSec < 25.0f && groundSpeedKt < 8.0f)
            {
                playFamilyCue(family, AircraftSoundSituation::Tow, 0.80f);
                m_towCuePlayed = true;
            }

            if ((family == AircraftSoundFamily::JetB || family == AircraftSoundFamily::JetC ||
                 family == AircraftSoundFamily::JetD || family == AircraftSoundFamily::JetE ||
                 family == AircraftSoundFamily::JetF) &&
                !m_togaCuePlayed && thrustRatio > 0.70f && groundSpeedKt > 20.0f)
            {
                playFamilyCue(family, AircraftSoundSituation::Toga, 1.0f);
                m_togaCuePlayed = true;
            }

            if ((family == AircraftSoundFamily::JetB || family == AircraftSoundFamily::JetC ||
                 family == AircraftSoundFamily::JetD || family == AircraftSoundFamily::JetE ||
                 family == AircraftSoundFamily::JetF) &&
                !m_maxCuePlayed && thrustRatio > 0.90f && groundSpeedKt > 35.0f)
            {
                playFamilyCue(family, AircraftSoundSituation::Max, 1.0f);
                m_maxCuePlayed = true;
            }
        }

        if ((m_lastObservedPhase == Flight::Phase::Arrival || touchedDown) && reverseRatio > 0.15f && !m_backCuePlayed)
        {
            if (family == AircraftSoundFamily::JetB || family == AircraftSoundFamily::JetC ||
                family == AircraftSoundFamily::JetD || family == AircraftSoundFamily::JetE ||
                family == AircraftSoundFamily::JetF)
            {
                playFamilyCue(family, AircraftSoundSituation::Back, 0.95f);
            }
            m_backCuePlayed = true;
        }
    }

    void safeUpdatePosition(float elapsedSinceLastCall)
    {
        m_frameCount++;

        const auto currentPhase = m_flight->phase();
        const bool phaseChanged = currentPhase != m_lastObservedPhase;
        if (phaseChanged)
        {
            m_departurePhaseElapsedSec = 0.0f;
            m_startCuePlayed = false;
            m_towCuePlayed = false;
            m_togaCuePlayed = false;
            m_maxCuePlayed = false;
            m_backCuePlayed = false;
            m_burnerCuePlayed = false;
            m_sonicBoomCuePlayed = false;
        }

        m_lastObservedPhase = currentPhase;

        if (currentPhase == Flight::Phase::Departure)
        {
            m_departurePhaseElapsedSec += max(0.0f, elapsedSinceLastCall);
        }
        else
        {
            m_departurePhaseElapsedSec = 0.0f;
        }

        if (label.length() > 0 && !m_config->showAIAircraftLabels)
        {
            label.clear();
        }

        auto changeSet = m_onQueryChanges();
        bool anyUpdates = changeSet && hasKey(changeSet->flights().updated(), m_flight->id());
        bool configChanged = changeSet && changeSet->configurationChanged();
        bool isDebugMode = m_config->showAIAircraftDebugLabels;
        const bool shouldUpdateVisualState = anyUpdates || configChanged || isDebugMode;

        //m_host->writeLog("Flight %s: updating sim aircraft location", m_flight->callSign().c_str());

        auto source = m_flight->aircraft();
        bool touchedDown = false;

        if (shouldUpdateVisualState)
        {
            const auto& location = source->location();
            const auto& attitude = source->attitude();
            const auto& altitude = source->altitude();
            touchedDown = source->justTouchedDown(m_host->getWorld()->timestamp());

            float pitchAdjustment = 0.0f;
            const float dt = max(0.016f, elapsedSinceLastCall);
            const float altitudeFeet = altitudeMslFeet(source);
            const float altitudeMeters = altitudeFeet / FEET_IN_1_METER;
            const float groundSpeedKt = static_cast<float>(fabs(source->groundSpeedKt()));
            const float verticalSpeedFpm = static_cast<float>(source->verticalSpeedFpm());
            const float worldTimeSeconds = static_cast<float>(m_host->getWorld()->timestamp().count()) / 1000000.0f;
            WeatherSnapshot weather;
            const bool hasWeather = !altitude.isGroundBased() && tryGetWeatherSnapshot(source, altitudeMeters, weather);
            const float turbulenceStrength = hasWeather ? weather.turbulenceStrength : 0.0f;
            const float crosswindMetersPerSecond = hasWeather
                ? crosswindComponentMetersPerSecond(static_cast<float>(source->track()), weather.windDirectionTrueDegrees, weather.windSpeedMetersPerSecond)
                : 0.0f;
            const float rollTurbulence = altitude.isGroundBased()
                ? 0.0f
                : turbulenceStrength * (
                    2.7f * turbulenceWave(worldTimeSeconds, 0.61f, 1.3f) +
                    1.2f * turbulenceWave(worldTimeSeconds, 1.37f, 2.1f));
            const float pitchTurbulence = altitude.isGroundBased()
                ? 0.0f
                : turbulenceStrength * (
                    0.9f * turbulenceWave(worldTimeSeconds, 0.83f, 0.7f) +
                    0.45f * turbulenceWave(worldTimeSeconds, 1.91f, 1.9f));
            const float headingTurbulence = altitude.isGroundBased()
                ? 0.0f
                : turbulenceStrength * 1.4f * turbulenceWave(worldTimeSeconds, 0.47f, 3.4f);
            const float crosswindBank = altitude.isGroundBased()
                ? 0.0f
                : clampValue(-crosswindMetersPerSecond * 0.55f, -5.5f, 5.5f);
            const float crabCorrection = altitude.isGroundBased()
                ? 0.0f
                : clampValue(crosswindMetersPerSecond * 0.45f, -4.0f, 4.0f);

            const float trackHeading = static_cast<float>(source->track());
            const float commandedHeading = static_cast<float>(attitude.heading());
            const float headingLead = shortestTurnDegrees(commandedHeading, trackHeading);
            float targetHeading = trackHeading + clampValue(headingLead * 0.25f, -6.0f, 6.0f) + crabCorrection + headingTurbulence;
            float targetPitch = static_cast<float>(attitude.pitch()) + pitchTurbulence;
            float targetRoll = static_cast<float>(attitude.roll()) + crosswindBank + rollTurbulence;

            if (!m_visualStateInitialized)
            {
                m_smoothedHeading = normalizeHeadingDegrees(static_cast<float>(attitude.heading()));
                m_smoothedPitch = static_cast<float>(attitude.pitch());
                m_smoothedRoll = static_cast<float>(attitude.roll());
                m_visualStateInitialized = true;
            }

            const float headingRateDegreesPerSecond = altitude.isGroundBased()
                ? max(18.0f, 8.0f + groundSpeedKt * 0.35f)
                : max(6.0f, min(28.0f, 7.0f + fabsf(targetRoll) * 0.65f + turbulenceStrength * 8.0f));
            const float pitchRateDegreesPerSecond = altitude.isGroundBased()
                ? 8.0f
                : max(3.5f, min(11.0f, 4.5f + fabsf(verticalSpeedFpm) / 1800.0f + turbulenceStrength * 2.5f));
            const float rollRateDegreesPerSecond = altitude.isGroundBased()
                ? 18.0f
                : max(10.0f, min(36.0f, 16.0f + fabsf(crosswindMetersPerSecond) * 0.9f + turbulenceStrength * 10.0f));

            SetLocation(location.latitude, location.longitude, altitude.isGroundBased() ? 0.0f : altitude.feet());

            if (altitude.isGroundBased())
            {
                safeClampToGround(pitchAdjustment);
                targetPitch = 0.0f;
                targetRoll = 0.0f;

                if (altitude.type() == Altitude::Type::AGL)
                {
                    drawInfo.y += altitude.feet() / FEET_IN_1_METER;
                }
            }

            m_smoothedHeading = approachHeadingDegrees(m_smoothedHeading, targetHeading, headingRateDegreesPerSecond * dt);
            m_smoothedPitch = approachLinear(m_smoothedPitch, targetPitch, pitchRateDegreesPerSecond * dt);
            m_smoothedRoll = approachLinear(m_smoothedRoll, targetRoll, rollRateDegreesPerSecond * dt);

            label = getLabelText(source);

            SetHeading(m_smoothedHeading);
            SetPitch(m_smoothedPitch + pitchAdjustment);
            SetRoll(m_smoothedRoll);
            SetYokePitchRatio(clampValue(m_smoothedPitch / 12.0f, -1.0f, 1.0f));
            SetYokeHeadingRatio(clampValue(shortestTurnDegrees(m_smoothedHeading, targetHeading) / 20.0f, -1.0f, 1.0f));
            SetYokeRollRatio(clampValue(m_smoothedRoll / 28.0f, -1.0f, 1.0f));

            SetLightsBeacon(source->isLightsOn(world::Aircraft::LightBits::Beacon));
            SetLightsTaxi(source->isLightsOn(world::Aircraft::LightBits::Taxi)); //TODO: taxi lights not working?
            SetLightsStrobe(source->isLightsOn(world::Aircraft::LightBits::Strobe));
            SetLightsLanding(
                source->isLightsOn(world::Aircraft::LightBits::Taxi) ||
                source->isLightsOn(world::Aircraft::LightBits::Landing));
            SetLightsNav(source->isLightsOn(world::Aircraft::LightBits::Nav));

            SetGearRatio(source->gearState());
            SetFlapRatio(source->flapState());
            SetSlatRatio(source->flapState());
            SetSpeedbrakeRatio(source->spoilerState());
            SetSpoilerRatio(source->spoilerState());

            SetTouchDown(touchedDown);
        }

        ensureEngineRunning(source, touchedDown, elapsedSinceLastCall);
        updateContrails(source);
        triggerSituationalSounds(source, touchedDown);
    }

    void safeClampToGround(float& groundPitch)
    {
        ClampToGround();

        //TODO: use actual model matched by XPMP2
        groundPitch = -1.5;
        // const string& model = m_flight->aircraft()->modelIcao();

        // if (model.compare("B738") == 0)
        // {
        //     SetPitch(-1.5);
        // }
        // else if (model.compare("A320") == 0)
        // {
        //     drawInfo.y += 0.4;
        //     SetPitch(-0.5);
        // }
    }

    static float clampRatio(float value)
    {
        return max(0.0f, min(1.0f, value));
    }

    static float clampValue(float value, float minValue, float maxValue)
    {
        return max(minValue, min(maxValue, value));
    }

    static float normalizeHeadingDegrees(float heading)
    {
        float normalized = fmodf(heading, 360.0f);
        if (normalized < 0.0f)
        {
            normalized += 360.0f;
        }
        return normalized;
    }

    static float shortestTurnDegrees(float fromHeading, float toHeading)
    {
        return GeoMath::getTurnDegrees(fromHeading, toHeading);
    }

    static float approachLinear(float current, float target, float maxDelta)
    {
        if (maxDelta <= 0.0f || fabsf(target - current) <= maxDelta)
        {
            return target;
        }

        return current + (target > current ? maxDelta : -maxDelta);
    }

    static float approachHeadingDegrees(float current, float target, float maxDelta)
    {
        const float delta = shortestTurnDegrees(current, target);
        if (maxDelta <= 0.0f || fabsf(delta) <= maxDelta)
        {
            return normalizeHeadingDegrees(target);
        }

        return normalizeHeadingDegrees(current + (delta > 0.0f ? maxDelta : -maxDelta));
    }

    static float crosswindComponentMetersPerSecond(float trackDegrees, float windDirectionTrueDegrees, float windSpeedMetersPerSecond)
    {
        const float relativeDegrees = windDirectionTrueDegrees - trackDegrees;
        return windSpeedMetersPerSecond * static_cast<float>(sin(GeoMath::degreesToRadians(relativeDegrees)));
    }

    float turbulenceWave(float timeSeconds, float frequencyMultiplier, float phaseMultiplier) const
    {
        const float seed = static_cast<float>((m_flight ? m_flight->id() : 1) * 0.173f);
        return static_cast<float>(sin(timeSeconds * frequencyMultiplier + seed * phaseMultiplier));
    }

    static float smoothSignal(float current, float target, float elapsedSeconds, float riseRatePerSecond, float fallRatePerSecond)
    {
        const float rate = target >= current ? riseRatePerSecond : fallRatePerSecond;
        return approachLinear(current, target, max(0.0f, rate) * max(0.0f, elapsedSeconds));
    }

    float familyEngineVolumeFloor(AircraftSoundFamily family) const
    {
        switch (family)
        {
        case AircraftSoundFamily::SingleA:
        case AircraftSoundFamily::SingleB:
        case AircraftSoundFamily::TwinA:
            return 0.72f;
        case AircraftSoundFamily::TwinB:
        case AircraftSoundFamily::TurbopropB:
        case AircraftSoundFamily::TurbopropC:
            return 0.78f;
        case AircraftSoundFamily::JetB:
        case AircraftSoundFamily::JetC:
            return 0.84f;
        case AircraftSoundFamily::JetD:
        case AircraftSoundFamily::JetE:
        case AircraftSoundFamily::JetF:
            return 0.90f;
        case AircraftSoundFamily::Fighter:
            return 0.98f;
        case AircraftSoundFamily::Helicopter:
            return 0.88f;
        default:
            return 0.80f;
        }
    }

    float familyTaxiLoopBaseVolume(AircraftSoundFamily family) const
    {
        switch (family)
        {
        case AircraftSoundFamily::SingleA:
        case AircraftSoundFamily::SingleB:
            return 0.46f;
        case AircraftSoundFamily::TwinA:
        case AircraftSoundFamily::TwinB:
            return 0.50f;
        case AircraftSoundFamily::TurbopropB:
        case AircraftSoundFamily::TurbopropC:
            return 0.57f;
        case AircraftSoundFamily::JetB:
        case AircraftSoundFamily::JetC:
            return 0.54f;
        case AircraftSoundFamily::JetD:
        case AircraftSoundFamily::JetE:
        case AircraftSoundFamily::JetF:
            return 0.62f;
        case AircraftSoundFamily::Fighter:
            return 0.64f;
        case AircraftSoundFamily::Helicopter:
            return 0.58f;
        default:
            return 0.50f;
        }
    }

    pair<float, float> thrustSpoolRates(const shared_ptr<world::Aircraft>& source) const
    {
        if (!source)
        {
            return { 0.35f, 0.25f };
        }

        switch (source->category())
        {
        case world::Aircraft::Category::Heavy:
            return { 0.18f, 0.12f };
        case world::Aircraft::Category::Jet:
            return { 0.22f, 0.14f };
        case world::Aircraft::Category::Fighter:
            return { 0.40f, 0.28f };
        case world::Aircraft::Category::Turboprop:
            return { 0.28f, 0.20f };
        case world::Aircraft::Category::Helicopter:
            return { 0.20f, 0.16f };
        case world::Aircraft::Category::LightProp:
        case world::Aircraft::Category::Prop:
            return { 0.34f, 0.26f };
        default:
            return { 0.25f, 0.18f };
        }
    }

    pair<float, float> engineRpmSpoolRates(const shared_ptr<world::Aircraft>& source) const
    {
        if (!source)
        {
            return { 400.0f, 280.0f };
        }

        switch (source->category())
        {
        case world::Aircraft::Category::Heavy:
            return { 220.0f, 150.0f };
        case world::Aircraft::Category::Jet:
            return { 280.0f, 180.0f };
        case world::Aircraft::Category::Fighter:
            return { 650.0f, 460.0f };
        case world::Aircraft::Category::Turboprop:
            return { 380.0f, 260.0f };
        case world::Aircraft::Category::Helicopter:
            return { 180.0f, 140.0f };
        case world::Aircraft::Category::LightProp:
        case world::Aircraft::Category::Prop:
            return { 460.0f, 320.0f };
        default:
            return { 320.0f, 220.0f };
        }
    }

    float propRpmResponseRate(const shared_ptr<world::Aircraft>& source) const
    {
        if (!source)
        {
            return 420.0f;
        }

        switch (source->category())
        {
        case world::Aircraft::Category::Helicopter:
            return 120.0f;
        case world::Aircraft::Category::Turboprop:
            return 260.0f;
        case world::Aircraft::Category::LightProp:
        case world::Aircraft::Category::Prop:
            return 340.0f;
        default:
            return 220.0f;
        }
    }

    float altitudeMslFeet(const shared_ptr<world::Aircraft>& source) const
    {
        if (!source)
        {
            return 0.0f;
        }

        const auto altitude = source->altitude();
        switch (altitude.type())
        {
        case Altitude::Type::Ground:
            return m_host ? m_host->queryTerrainElevationAt(source->location()) : 0.0f;
        case Altitude::Type::AGL:
            return altitude.feet() + (m_host ? m_host->queryTerrainElevationAt(source->location()) : 0.0f);
        case Altitude::Type::MSL:
        default:
            return altitude.feet();
        }
    }

    float altitudeAglFeet(const shared_ptr<world::Aircraft>& source) const
    {
        if (!source)
        {
            return 0.0f;
        }

        return altitudeMslFeet(source) - (m_host ? m_host->queryTerrainElevationAt(source->location()) : 0.0f);
    }

    bool tryGetWeatherSnapshot(const shared_ptr<world::Aircraft>& source, float altitudeMeters, WeatherSnapshot& snapshot) const
    {
        if (!m_host || !source)
        {
            return false;
        }

        if (const auto weatherService = m_host->services().tryGet<WeatherService>())
        {
            snapshot = weatherService->getWeatherAt(source->location(), altitudeMeters);
            return snapshot.available;
        }

        return false;
    }

    bool isLargeTurbopropContrailCandidate(const shared_ptr<world::Aircraft>& source) const
    {
        return source &&
            source->category() == world::Aircraft::Category::Turboprop &&
            chooseSoundFamily(source) == AircraftSoundFamily::TurbopropC;
    }

    void updateContrails(const shared_ptr<world::Aircraft>& source)
    {
        if (!source || source->altitude().isGroundBased())
        {
            ContrailRemove();
            return;
        }

        const bool jetCandidate =
            source->category() == world::Aircraft::Category::Jet ||
            source->category() == world::Aircraft::Category::Heavy ||
            source->category() == world::Aircraft::Category::Fighter;
        const bool largeTurbopropCandidate = isLargeTurbopropContrailCandidate(source);
        if (!jetCandidate && !largeTurbopropCandidate)
        {
            ContrailRemove();
            return;
        }

        const float altitudeFeet = altitudeMslFeet(source);
        const float altitudeMeters = altitudeFeet / FEET_IN_1_METER;
        const float machNumber = estimateMachNumber(source);
        WeatherSnapshot weather;
        const bool hasWeather = tryGetWeatherSnapshot(source, altitudeMeters, weather);
        const float dewpointSpreadC = hasWeather ? max(0.0f, weather.temperatureC - weather.dewpointC) : 99.0f;

        bool cloudMoisture = false;
        if (hasWeather)
        {
            for (size_t i = 0; i < weather.cloudLayerCoverages.size(); ++i)
            {
                const float coverage = weather.cloudLayerCoverages.at(i);
                const float baseMeters = i < weather.cloudLayerBasesMeters.size() ? weather.cloudLayerBasesMeters.at(i) : 0.0f;
                const float topMeters = i < weather.cloudLayerTopsMeters.size() ? weather.cloudLayerTopsMeters.at(i) : baseMeters;
                if (coverage >= 0.60f && baseMeters <= altitudeMeters + 250.0f && altitudeMeters <= topMeters + 750.0f)
                {
                    cloudMoisture = true;
                    break;
                }
            }
        }

        const bool coldEnough = altitudeFeet >= 18500.0f && ((hasWeather && weather.temperatureC <= -30.0f) || altitudeFeet >= 25500.0f);
        const bool moistEnough = cloudMoisture || (hasWeather ? dewpointSpreadC <= 4.0f : altitudeFeet >= 24000.0f);
        const bool fastEnough = machNumber >= 0.45f || abs(source->groundSpeedKt()) >= 240.0f;

        if (!coldEnough || !moistEnough || !fastEnough)
        {
            ContrailRemove();
            return;
        }

        unsigned desiredCount = 1;
        unsigned desiredSpacingMeters = 5;
        if (source->category() == world::Aircraft::Category::Heavy)
        {
            desiredCount = 2;
            desiredSpacingMeters = 12;
        }
        else if (source->category() == world::Aircraft::Category::Jet)
        {
            const auto family = chooseSoundFamily(source);
            desiredCount = (family == AircraftSoundFamily::JetD || family == AircraftSoundFamily::JetE || family == AircraftSoundFamily::JetF) ? 2u : 1u;
            desiredSpacingMeters = desiredCount > 1 ? 10u : 6u;
        }
        else if (source->category() == world::Aircraft::Category::Fighter)
        {
            desiredCount = 1;
            desiredSpacingMeters = 4;
        }
        else if (largeTurbopropCandidate)
        {
            desiredCount = 2;
            desiredSpacingMeters = 7;
        }

        unsigned lifeTimeSeconds = 20;
        if (hasWeather)
        {
            const float persistenceFactor = max(0.0f, min(1.0f, (4.0f - dewpointSpreadC) / 4.0f));
            lifeTimeSeconds = static_cast<unsigned>(20.0f + persistenceFactor * 14.0f + (cloudMoisture ? 6.0f : 0.0f));
        }
        else if (altitudeFeet >= 28000.0f)
        {
            lifeTimeSeconds = 28;
        }

        ContrailRequest(desiredCount, desiredSpacingMeters, lifeTimeSeconds);
    }

    float categoryStartupDurationSeconds(const shared_ptr<world::Aircraft>& source) const
    {
        if (!source)
        {
            return 0.0f;
        }

        switch (source->category())
        {
        case world::Aircraft::Category::Helicopter:
            return 18.0f;
        case world::Aircraft::Category::LightProp:
            return 6.0f;
        case world::Aircraft::Category::Prop:
            return 8.0f;
        case world::Aircraft::Category::Turboprop:
            return 12.0f;
        case world::Aircraft::Category::Fighter:
            return 10.0f;
        case world::Aircraft::Category::Heavy:
        case world::Aircraft::Category::Jet:
        default:
            return 12.0f;
        }
    }

    float categoryIdleThrustRatio(const shared_ptr<world::Aircraft>& source) const
    {
        if (!source)
        {
            return 0.0f;
        }

        switch (source->category())
        {
        case world::Aircraft::Category::Helicopter:
            return 0.14f;
        case world::Aircraft::Category::LightProp:
            return 0.10f;
        case world::Aircraft::Category::Prop:
            return 0.12f;
        case world::Aircraft::Category::Turboprop:
            return 0.16f;
        case world::Aircraft::Category::Fighter:
            return 0.20f;
        case world::Aircraft::Category::Heavy:
        case world::Aircraft::Category::Jet:
        default:
            return 0.18f;
        }
    }

    float categoryIdleEngineRpm(const shared_ptr<world::Aircraft>& source) const
    {
        if (!source)
        {
            return 0.0f;
        }

        switch (source->category())
        {
        case world::Aircraft::Category::Helicopter:
            return 900.0f;
        case world::Aircraft::Category::LightProp:
            return 650.0f;
        case world::Aircraft::Category::Prop:
            return 750.0f;
        case world::Aircraft::Category::Turboprop:
            return 850.0f;
        case world::Aircraft::Category::Fighter:
            return 1400.0f;
        case world::Aircraft::Category::Heavy:
        case world::Aircraft::Category::Jet:
        default:
            return 900.0f;
        }
    }

    float categoryIdlePropRpm(const shared_ptr<world::Aircraft>& source) const
    {
        if (!source)
        {
            return 0.0f;
        }

        switch (source->category())
        {
        case world::Aircraft::Category::Helicopter:
            return 180.0f;
        case world::Aircraft::Category::LightProp:
            return 600.0f;
        case world::Aircraft::Category::Prop:
            return 700.0f;
        case world::Aircraft::Category::Turboprop:
            return 850.0f;
        default:
            return 0.0f;
        }
    }

    float departureStartupProgress(const shared_ptr<world::Aircraft>& source) const
    {
        if (!source || isParked(source) || m_flight->phase() != Flight::Phase::Departure || !source->altitude().isGroundBased())
        {
            return 1.0f;
        }

        const float startupDuration = max(1.0f, categoryStartupDurationSeconds(source));
        return clampRatio(m_departurePhaseElapsedSec / startupDuration);
    }

    bool isAfterburnerActive(const shared_ptr<world::Aircraft>& source, float thrustRatio) const
    {
        if (!source || source->category() != world::Aircraft::Category::Fighter)
        {
            return false;
        }

        if (!source->isLightsOn(world::Aircraft::LightBits::BeaconNavStrobe))
        {
            return false;
        }

        const float groundSpeedKt = static_cast<float>(abs(source->groundSpeedKt()));
        const float verticalSpeedFpm = static_cast<float>(source->verticalSpeedFpm());
        return thrustRatio > 0.90f || groundSpeedKt > 180.0f || verticalSpeedFpm > 2500.0f;
    }

    float estimateMachNumber(const shared_ptr<world::Aircraft>& source) const
    {
        if (!source)
        {
            return 0.0f;
        }

        const float groundSpeedMetersPerSecond = static_cast<float>(std::fabs(source->groundSpeedKt()) / KNOT_IN_1_METER_PER_SEC);
        const float altitudeMeters = source->altitude().isGroundBased() ? 0.0f : static_cast<float>(source->altitude().feet() / FEET_IN_1_METER);
        float temperatureC = 15.0f - (6.5f * max(0.0f, altitudeMeters) / 1000.0f);
        float windSpeedMetersPerSecond = 0.0f;
        float windDirectionTrueDegrees = 0.0f;

        if (m_host)
        {
            if (const auto weatherService = m_host->services().tryGet<WeatherService>())
            {
                const auto weather = weatherService->getWeatherAt(source->location(), altitudeMeters);
                if (weather.available)
                {
                    temperatureC = weather.temperatureC;
                    windSpeedMetersPerSecond = max(0.0f, weather.windSpeedMetersPerSecond);
                    windDirectionTrueDegrees = weather.windDirectionTrueDegrees;
                }
            }
        }

        const float temperatureKelvin = max(-80.0f, temperatureC) + 273.15f;
        const float speedOfSoundMetersPerSecond = static_cast<float>(sqrt(max(0.0f, 1.4f * 287.05f * temperatureKelvin)));
        if (speedOfSoundMetersPerSecond <= 0.0f)
        {
            return 0.0f;
        }

        const float trackDegrees = static_cast<float>(source->track());
        const float windAngleDegrees = windDirectionTrueDegrees - trackDegrees;
        const float headwindComponentMetersPerSecond = windSpeedMetersPerSecond * static_cast<float>(cos(GeoMath::degreesToRadians(windAngleDegrees)));
        const float trueAirspeedMetersPerSecond = max(0.0f, groundSpeedMetersPerSecond + headwindComponentMetersPerSecond);
        return trueAirspeedMetersPerSecond / speedOfSoundMetersPerSecond;
    }

    bool isParked(const shared_ptr<world::Aircraft>& source) const
    {
        if (!source)
        {
            return false;
        }

        try
        {
            return source->getFlightOrThrow()->phase() == Flight::Phase::TurnAround;
        }
        catch (const exception&)
        {
            return false;
        }
    }

    float calculateReverseRatio(const shared_ptr<world::Aircraft>& source, bool touchedDown) const
    {
        if (!source || isParked(source) || !source->altitude().isGroundBased())
        {
            return 0.0f;
        }

        if (touchedDown)
        {
            return 1.0f;
        }

        const float groundSpeedKt = static_cast<float>(abs(source->groundSpeedKt()));
        const float spoilerState = source->spoilerState();

        if (spoilerState > 0.2f && groundSpeedKt > 20.0f)
        {
            return clampRatio((groundSpeedKt - 20.0f) / 80.0f);
        }

        return 0.0f;
    }

    float calculateThrustRatio(const shared_ptr<world::Aircraft>& source, float reverseRatio) const
    {
        if (!source || isParked(source))
        {
            return 0.0f;
        }

        const bool isGroundBased = source->altitude().isGroundBased();
        const float groundSpeedKt = static_cast<float>(abs(source->groundSpeedKt()));
        const float verticalSpeedFpm = static_cast<float>(source->verticalSpeedFpm());
        const float altitudeFeet = altitudeMslFeet(source);
        WeatherSnapshot weather;
        const bool hasWeather = tryGetWeatherSnapshot(source, altitudeFeet / FEET_IN_1_METER, weather);

        float thrustRatio = 0.0f;
        if (isGroundBased)
        {
            if (groundSpeedKt > 140.0f)
            {
                thrustRatio = 0.92f;
            }
            else if (groundSpeedKt > 60.0f)
            {
                thrustRatio = 0.68f;
            }
            else if (groundSpeedKt > 15.0f)
            {
                thrustRatio = 0.34f;
            }
        }
        else
        {
            if (verticalSpeedFpm > 1500.0f || groundSpeedKt > 220.0f)
            {
                thrustRatio = 0.95f;
            }
            else if (verticalSpeedFpm > 300.0f)
            {
                thrustRatio = 0.76f;
            }
            else if (verticalSpeedFpm < -1000.0f)
            {
                thrustRatio = 0.44f;
            }
            else if (groundSpeedKt > 180.0f)
            {
                thrustRatio = 0.62f;
            }
            else
            {
                thrustRatio = 0.52f;
            }
        }

        if (!isGroundBased)
        {
            if (altitudeFeet > 12000.0f)
            {
                thrustRatio += min(0.06f, (altitudeFeet - 12000.0f) / 28000.0f * 0.06f);
            }

            if (hasWeather)
            {
                const float gustExcessMetersPerSecond = max(0.0f, weather.windGustMetersPerSecond - weather.windSpeedMetersPerSecond);
                thrustRatio += min(0.06f, weather.turbulenceStrength * 0.05f + gustExcessMetersPerSecond * 0.006f);
                if (weather.windShearMetersPerSecond > 4.0f && fabsf(verticalSpeedFpm) > 500.0f)
                {
                    thrustRatio += min(0.05f, weather.windShearMetersPerSecond / 180.0f);
                }
            }
        }

        if (isGroundBased)
        {
            thrustRatio = max(thrustRatio, categoryIdleThrustRatio(source) * departureStartupProgress(source));
        }

        if (isAfterburnerActive(source, thrustRatio))
        {
            thrustRatio = max(thrustRatio, 1.0f);
        }

        if (reverseRatio > 0.0f)
        {
            thrustRatio *= (1.0f - (0.5f * reverseRatio));
        }

        return clampRatio(thrustRatio);
    }

    float calculateEngineRpm(const shared_ptr<world::Aircraft>& source, float thrustRatio, float reverseRatio) const
    {
        if (!source || isParked(source))
        {
            return 0.0f;
        }

        const bool isGroundBased = source->altitude().isGroundBased();
        float engineRpm = 0.0f;
        if (isGroundBased)
        {
            if (source->groundSpeedKt() > 140.0f)
            {
                engineRpm = 800.0f + thrustRatio * 1700.0f;
            }
            else if (source->groundSpeedKt() > 60.0f)
            {
                engineRpm = 700.0f + thrustRatio * 1500.0f;
            }
            else if (source->groundSpeedKt() > 15.0f)
            {
                engineRpm = 600.0f + thrustRatio * 1100.0f;
            }

            engineRpm = max(engineRpm, categoryIdleEngineRpm(source) * departureStartupProgress(source));
        }
        else
        {
            engineRpm = 1000.0f + thrustRatio * 2200.0f;
        }

        if (reverseRatio > 0.0f)
        {
            engineRpm = max(engineRpm, 1200.0f + reverseRatio * 700.0f);
        }

        if (isAfterburnerActive(source, thrustRatio))
        {
            engineRpm = max(engineRpm, 3600.0f);
        }

        return engineRpm;
    }

    float calculatePropRpm(const shared_ptr<world::Aircraft>& source, float thrustRatio) const
    {
        if (!source || isParked(source))
        {
            return 0.0f;
        }

        float propRpm = 0.0f;
        switch (source->category())
        {
        case world::Aircraft::Category::Helicopter:
            propRpm = thrustRatio * 420.0f;
            break;
        case world::Aircraft::Category::LightProp:
            propRpm = 450.0f + thrustRatio * 550.0f;
            break;
        case world::Aircraft::Category::Prop:
            propRpm = 500.0f + thrustRatio * 800.0f;
            break;
        case world::Aircraft::Category::Turboprop:
            propRpm = 650.0f + thrustRatio * 850.0f;
            break;
        default:
            return 0.0f;
        }

        if (source->altitude().isGroundBased())
        {
            propRpm = max(propRpm, categoryIdlePropRpm(source) * departureStartupProgress(source));
        }

        return propRpm;
    }

    void ensureEngineRunning(const shared_ptr<world::Aircraft>& source, bool touchedDown, float elapsedSinceLastCall = 0.0f)
    {
        const float reverseRatio = calculateReverseRatio(source, touchedDown);
        const float thrustRatio = calculateThrustRatio(source, reverseRatio);
        const float engineRpm = calculateEngineRpm(source, thrustRatio, reverseRatio);
        const float propRpm = calculatePropRpm(source, thrustRatio);

        if (!m_engineStateInitialized || elapsedSinceLastCall <= 0.0f)
        {
            m_smoothedThrustRatio = thrustRatio;
            m_smoothedReverseRatio = reverseRatio;
            m_smoothedEngineRpm = engineRpm;
            m_smoothedPropRpm = propRpm;
            m_engineStateInitialized = true;
        }
        else
        {
            const auto thrustRates = thrustSpoolRates(source);
            const auto rpmRates = engineRpmSpoolRates(source);
            const float propRate = propRpmResponseRate(source);

            m_smoothedThrustRatio = smoothSignal(m_smoothedThrustRatio, thrustRatio, elapsedSinceLastCall, thrustRates.first, thrustRates.second);
            m_smoothedReverseRatio = smoothSignal(m_smoothedReverseRatio, reverseRatio, elapsedSinceLastCall, 0.85f, 0.60f);
            m_smoothedEngineRpm = smoothSignal(m_smoothedEngineRpm, engineRpm, elapsedSinceLastCall, rpmRates.first, rpmRates.second);
            m_smoothedPropRpm = smoothSignal(m_smoothedPropRpm, propRpm, elapsedSinceLastCall, propRate, propRate * 0.85f);
        }

        m_afterburnerActive = isAfterburnerActive(source, m_smoothedThrustRatio);

        SetThrustRatio(m_smoothedThrustRatio);
        SetThrustReversRatio(m_smoothedReverseRatio);
        SetReversDeployRatio(m_smoothedReverseRatio);
        SetEngineRotRpm(m_smoothedEngineRpm);
        SetPropRotRpm(m_smoothedPropRpm);
    }
private:

    string getLabelText(const shared_ptr<world::Aircraft>& aircraft)
    {
        if (!m_config->showAIAircraftLabels)
        {
            return "";
        }

        stringstream text;
        text << m_flight->callSign() << " ";

        switch (m_flight->phase())
        {
        case Flight::Phase::Arrival:
            text << "(A)";
            break;
        case Flight::Phase::Departure:
            text << "(D)";
            break;
        case Flight::Phase::TurnAround:
            text << "(t/a)";
            break;
        }

        string altitudeString = aircraft->altitude().toString();
        if (!altitudeString.empty())
        {
            text << " | " << altitudeString;
        }

        if (m_config->showAIAircraftDebugLabels)
        {
            text << " | " << aircraft->frequencyKhz()
                 << " | " << acRadar.code
                 << (IsCurrentlyShownAsAI() ? 'A' : 'a')
                 << (IsCurrentlyShownAsTcasTarget() ? 'T' : 't')
                 << '/' << tcasTargetIdx;

            string debugString = aircraft->getStatusString();
            if (!debugString.empty())
            {
                text << " | " << debugString;
            }
        }

        return text.str();
    }
};

class Xpmp2AircraftObjectService : public AircraftObjectService
{
private:
    shared_ptr<HostServices> m_host;
    shared_ptr<World::ChangeSet> m_lastChangeSet;
    vector<shared_ptr<Xpmp2AircraftObject>> m_simAircraft; //TODO: replace vector with linked list
public:
    explicit Xpmp2AircraftObjectService(shared_ptr<HostServices> _host) :
        m_host(std::move(_host))
    {
        m_host->writeLog("MP2SVC|Xpmp2AircraftObjectService::Xpmp2AircraftObjectService()");

        string resourceDirectory = m_host->getResourceFilePath({ "Resources" });
        const char* error = XPMPMultiplayerInit(
            "AT&C",               // plugin name,
            resourceDirectory.c_str(),         // path to supplemental files
            CBIntPrefsFunc,                    // configuration callback function
            "B738");              // default ICAO type

        if (error[0])
        {
            m_host->writeLog("MP2SVC|XPMPMultiplayerInit: FAILED!");
            return;
        }

        m_host->writeLog("MP2SVC|XPMPMultiplayerInit: success.");

        // Load our CSL models
        error = XPMPLoadCSLPackage(resourceDirectory.c_str());     // CSL folder root path
        if (error[0])
        {
            m_host->writeLog("MP2SVC|XPMPLoadCSLPackage: FAILED!");
            return;
        }

        m_host->writeLog("MP2SVC|XPMPLoadCSLPackage: success.");

        // Now we also try to get control of AI planes. That's optional, though,
        // other plugins (like LiveTraffic, XSquawkBox, X-IvAp...)
        // could have control already
        error = XPMPMultiplayerEnable(CPRequestAIAgain);
        if (error[0]) {
            m_host->writeLog("MP2SVC|XPMPMultiplayerEnable FAILED! %s", error);
            return;
        }

        m_host->writeLog("MP2SVC|XPMPMultiplayerEnable: SUCCESS");
    }

    ~Xpmp2AircraftObjectService()
    {
        m_host->writeLog("MP2SVC|Xpmp2AircraftObjectService::Xpmp2AircraftObjectService()");

        m_host->writeLog("MP2SVC|invoking XPMPMultiplayerDisable");
        XPMPMultiplayerDisable();

        m_host->writeLog("MP2SVC|invoking XPMPMultiplayerCleanup");
        XPMPMultiplayerCleanup();
    }

public:

    void processEvents(shared_ptr<World::ChangeSet> changeSet) override
    {
        m_lastChangeSet = changeSet;

        for (const auto& addedFlight : m_lastChangeSet->flights().added())
        {
            if (addedFlight->aircraft()->nature() != world::Actor::Nature::AI)
            {
                continue;
            }

            auto newSimAircraft = shared_ptr<Xpmp2AircraftObject>(new Xpmp2AircraftObject(m_host, addedFlight));
            newSimAircraft->onQueryChanges([this, addedFlight](){
                //m_host->writeLog("onQueryChanges from %s", addedFlight->callSign().c_str());
                return m_lastChangeSet;
            });

            m_simAircraft.push_back(newSimAircraft);
        }
    }

    void clearAll() override
    {
        m_simAircraft.clear();
    }

private:

    /// This is a callback the XPMP2 calls regularly to learn about configuration settings.
    /// Only 3 are left, all of them integers.
    static int CBIntPrefsFunc(const char*, [[maybe_unused]] const char* item, int defaultVal)
    {
        //if (!strcmp(item, "model_matching")) return 1;
        //if (!strcmp(item, "log_level")) return 0;       // DEBUG logging level
        return defaultVal;
    }

    static void CPRequestAIAgain(void*)
    {
        PrintDebugString("MP2SVC|CPRequestAIAgain: invoking XPMPMultiplayerEnable");
        XPMPMultiplayerEnable(CPRequestAIAgain);
    }
};
