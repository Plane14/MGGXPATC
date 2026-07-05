// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#include <iostream>
#include <functional>
#include <string>
#include <cstring>

// SDK
#include "XPLMPlugin.h"
#include "XPLMNavigation.h"
#include "XPLMUtilities.h"

#if !XPLM300
#error This plugin requires version 300 of the SDK
#endif

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
#include "owneddata.h"

// tnc
#include "utils.h"
#include "libworld.h"
#include "pluginInstance.hpp"

using namespace std;
using namespace PPL;

static PluginInstance* pInstance = nullptr;

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc)
{
    strncpy(outName, "Air Traffic & Control", 255);
    outName[255] = '\0';
    strncpy(outSig, "felix-b.atc", 255);
    outSig[255] = '\0';
    strncpy(outDesc, "Offline virtual world of air traffic and ATC", 255);
    outDesc[255] = '\0';
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

    // Runtime SDK version check (we compile against 301+, but verify at runtime).
    int xplmVersion = 0;
    XPLMGetVersions(nullptr, &xplmVersion, nullptr);
    if (xplmVersion < 301)
    {
        XPLMDebugString("ATC|ERROR: X-Plane SDK version is too old. ATC plugin requires SDK 301+.\n");
        return 0;
    }

    PluginPath::setPluginDirectoryName("airTrafficAndControl");
    LogWriter::getLogger().setLogFile(PluginPath::prependPluginPath("atc_log.txt"));
    Log() << Log::Info << "XPluginStart" << Log::endl;

    initPluginUtils();
    PrintDebugString(
        "ENTRYP|XPluginStart, platform-build[%s], plugin-directory[%s] sim-directory[%s]\n",
        getBuildPlatformId(),
        getPluginDirectory().c_str(),
        getHostDirectory().c_str()
    );

    return 1;
}

PLUGIN_API int XPluginEnable(void)
{
    char name[256];
    char filePath[256];
    char signature[256];
    char description[256];

    int myPluginId = XPLMGetMyID();
    XPLMGetPluginInfo(myPluginId, name, filePath, signature, description);

    PrintDebugString("ENTRYP|XPluginEnable");
    PrintDebugString(
        "ENTRYP|XPLMGetPluginInfo(pluginId=%d) -> name=[%s] filePath=[%s] signature=[%s] description=[%s]",
        myPluginId, name, filePath, signature, description);

    try
    {
        pInstance = new PluginInstance();
        return 1;
    }
    catch (const exception& e)
    {
        PrintDebugString("ENTRYP|XPluginEnable failed: %s", e.what());
        Log() << Log::Error << "XPluginEnable failed: " << e.what() << Log::endl;
        pInstance = nullptr;
        return 0;
    }
    catch (...)
    {
        PrintDebugString("ENTRYP|XPluginEnable failed: unknown exception");
        Log() << Log::Error << "XPluginEnable failed: unknown exception" << Log::endl;
        pInstance = nullptr;
        return 0;
    }
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID fromId, long inMsg, void*)
{
    char name[256];
    char filePath[256];
    char signature[256];
    char description[256];
    XPLMGetPluginInfo(fromId, name, filePath, signature, description);

    PrintDebugString("ENTRYP|XPluginReceiveMessage(fromId=[%d|%s], inMsg=[%ld])\n", fromId, name, inMsg);
    if (!pInstance)
    {
        return;
    }

    if (inMsg == XPLM_MSG_AIRPORT_LOADED)
    {
        pInstance->notifyAirportLoaded();
    }
    else if (inMsg == XPLM_MSG_SCENERY_LOADED)
    {
        PrintDebugString("ENTRYP|Scenery loaded, re-evaluating airport data.\n");
        pInstance->notifySceneryLoaded();
    }
    else if (inMsg == XPLM_MSG_PLANE_LOADED && fromId == XPLM_PLUGIN_XPLANE)
    {
        PrintDebugString("ENTRYP|User plane loaded, re-initializing user aircraft.\n");
        pInstance->notifyPlaneLoaded();
    }
    else if (inMsg == XPLM_MSG_PLANE_CRASHED)
    {
        PrintDebugString("ENTRYP|Plane crashed, suspending AI schedules.\n");
        pInstance->notifyPlaneCrashed();
    }

//    DataRef<double> userAircraftLatitude("sim/flightmodel/position/latitude", PPL::ReadOnly);
//    DataRef<double> userAircraftLongitude("sim/flightmodel/position/longitude", PPL::ReadOnly);
//    float userLat = userAircraftLatitude;
//    float userLon = userAircraftLongitude;
//    char icaoCode[10] = { 0 };
//    XPLMNavRef navRef = XPLMFindNavAid( nullptr, nullptr, &userLat, &userLon, nullptr, xplm_Nav_Airport);
//    if (navRef != XPLM_NAV_NOT_FOUND)
//    {
//        XPLMGetNavAidInfo(navRef, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, icaoCode, nullptr, nullptr);
//    }
//    PrintDebugString("     > user@(%f,%f) -> ICAO[%s]\n", userLat, userLon, strlen(icaoCode) > 0 ? icaoCode : "N/A");
}

PLUGIN_API void XPluginDisable(void)
{
    XPLMDebugString("ENTRYP|XPluginDisable\n");
    Log() << Log::Info << "XPluginDisable" << Log::endl;
    if (pInstance)
    {
        delete pInstance;
        pInstance = nullptr;
    }
}

PLUGIN_API void	XPluginStop(void)
{
    XPLMDebugString("ENTRYP|XPluginStop\n");
    Log() << Log::Info << "XPluginStop" << Log::endl;
}
