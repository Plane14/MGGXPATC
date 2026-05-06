// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#include <string>

#include "gtest/gtest.h"
#include "libworld.h"

#define private public
#include "aircraftPerformanceTable.hpp"
#undef private

using namespace std;
using namespace world;
using namespace ai;

namespace
{
    string doc8643As50Text()
    {
        return "H1T L/G TECHNICAL DATA Wing Span (m)-Length (m)10.9Height (m)3.1MTOW (t)2.1Fuel Capacity (ltr)540"
               "Maximum Range (Nm)390Persons On Board5Take Off Distance (m)-Landing Distance (m)-Absolute Ceiling (x100ft)160"
               "Optimum Ceiling (x100ft)120Maximum Speed (kts/M)146Optimum Speed (kts/M)135Maximum Climb Rate (ft/min)1200 MANUFACTURERS";
    }

    string doc8643Dc3Text()
    {
        return "L2P M/G TECHNICAL DATA Wing Span (m)28.9Length (m)19.6Height (m)5.1MTOW (t)12.7Fuel Capacity (ltr)3000"
               "Maximum Range (Nm)300Persons On Board21-32Take Off Distance (m)Landing Distance (m)1402Absolute Ceiling (x100ft)240"
               "Optimum Ceiling (x100ft)180Maximum Speed (kts/M)199Optimum Speed (kts/M)151Maximum Climb Rate (ft/min)1130 MANUFACTURERS";
    }

    string doc8643F22Text()
    {
        return "L2J M/F TECHNICAL DATA Wing Span (m)13.5Length (m)18.9Height (m)5MTOW (t)27.2Fuel Capacity (ltr)Maximum Range (Nm)1500"
               "Persons On Board2Take Off Distance (m)Landing Distance (m)Absolute Ceiling (x100ft)555Optimum Ceiling (x100ft)300"
               "Maximum Speed (kts/M)800 / 1.70Optimum Speed (kts/M)1.33Maximum Climb Rate (ft/min) MANUFACTURERS";
    }
}

TEST(AircraftPerformanceTableTest, parsesDoc8643As50PerformanceData)
{
    auto profile = AircraftPerformanceTable::defaultProfile(Aircraft::Category::Helicopter);

    EXPECT_TRUE(AircraftPerformanceTable::parseDoc8643DetailsText(doc8643As50Text(), profile));
    EXPECT_EQ(profile.classification, "H1T");
    EXPECT_EQ(profile.wakeCategory, "L/G");
    EXPECT_FLOAT_EQ(profile.rangeNm, 390.0f);
    EXPECT_EQ(profile.ceilingFl, 160);
    EXPECT_FLOAT_EQ(profile.initialClimbRocFpm, 1200.0f);
    EXPECT_FLOAT_EQ(profile.approachSpeedKt, 70.0f);
}

TEST(AircraftPerformanceTableTest, parsesDoc8643Dc3PerformanceData)
{
    auto profile = AircraftPerformanceTable::defaultProfile(Aircraft::Category::Prop);

    EXPECT_TRUE(AircraftPerformanceTable::parseDoc8643DetailsText(doc8643Dc3Text(), profile));
    EXPECT_EQ(profile.classification, "L2P");
    EXPECT_EQ(profile.wakeCategory, "M/G");
    EXPECT_FLOAT_EQ(profile.rangeNm, 300.0f);
    EXPECT_EQ(profile.ceilingFl, 240);
    EXPECT_FLOAT_EQ(profile.initialClimbRocFpm, 1130.0f);
    EXPECT_FLOAT_EQ(profile.approachSpeedKt, 95.0f);
}

TEST(AircraftPerformanceTableTest, parsesDoc8643F22PerformanceData)
{
    auto profile = AircraftPerformanceTable::defaultProfile(Aircraft::Category::Fighter);

    EXPECT_TRUE(AircraftPerformanceTable::parseDoc8643DetailsText(doc8643F22Text(), profile));
    EXPECT_EQ(profile.classification, "L2J");
    EXPECT_EQ(profile.wakeCategory, "M/F");
    EXPECT_FLOAT_EQ(profile.rangeNm, 1500.0f);
    EXPECT_EQ(profile.ceilingFl, 555);
    EXPECT_FLOAT_EQ(profile.initialClimbRocFpm, 20000.0f);
    EXPECT_FLOAT_EQ(profile.approachSpeedKt, 176.0f);
}

TEST(AircraftPerformanceTableTest, classifiesHelicopterFamiliesFromIcao)
{
    const char* helicopterIcaos[] = {
        "B06", "AS32", "B105", "EC25", "S58T", "S65C",
        "UH1", "UH60", "OH58", "CH47", "CH53", "HH60",
        "MH60", "SH60", "S70", "MD500", "MD600", "MD902"
    };

    for (const char* icao : helicopterIcaos)
    {
        EXPECT_EQ(AircraftPerformanceTable::classifyFromIcao(icao), Aircraft::Category::Helicopter) << icao;
    }
}

TEST(AircraftPerformanceTableTest, usesDeterministicFallbackForCommonProfilesWithoutHostAccess)
{
    auto jetProfile = AircraftPerformanceTable::lookup(nullptr, "B738", Aircraft::Category::Jet);
    auto lightProfile = AircraftPerformanceTable::lookup(nullptr, "C172", Aircraft::Category::LightProp);

    EXPECT_GT(jetProfile.approachSpeedKt, 180.0f);
    EXPECT_LT(lightProfile.approachSpeedKt, jetProfile.approachSpeedKt);
    EXPECT_LT(lightProfile.approachSpeedKt, 160.0f);
}

TEST(AircraftPerformanceTableTest, excludesRegionalAirlinerTurbopropsFromGaTraffic)
{
    EXPECT_FALSE(AircraftPerformanceTable::isGeneralAviationTrafficType("DH8D", Aircraft::Category::Turboprop));
    EXPECT_FALSE(AircraftPerformanceTable::isGeneralAviationTrafficType("AT72", Aircraft::Category::Turboprop));
    EXPECT_TRUE(AircraftPerformanceTable::isGeneralAviationTrafficType("C208", Aircraft::Category::Turboprop));
    EXPECT_TRUE(AircraftPerformanceTable::isGeneralAviationTrafficType("PC12", Aircraft::Category::Turboprop));
}

TEST(AircraftPerformanceTableTest, usesDistinctTakeoffProfilesForJetAndTurboprop)
{
    auto jetProfile = AircraftPerformanceTable::lookup(nullptr, "A320", Aircraft::Category::Jet);
    auto turbopropProfile = AircraftPerformanceTable::lookup(nullptr, "DH8D", Aircraft::Category::Turboprop);

    EXPECT_GT(jetProfile.takeoffRotateSpeedKt, turbopropProfile.takeoffRotateSpeedKt);
    EXPECT_GT(jetProfile.takeoffLiftOffSpeedKt, turbopropProfile.takeoffLiftOffSpeedKt);
    EXPECT_GT(jetProfile.takeoffInitialClimbSpeedKt, turbopropProfile.takeoffInitialClimbSpeedKt);
}

TEST(AircraftPerformanceTableTest, usesDistinctLandingProfilesForJetAndLightProp)
{
    auto jetProfile = AircraftPerformanceTable::lookup(nullptr, "A320", Aircraft::Category::Jet);
    auto lightProfile = AircraftPerformanceTable::lookup(nullptr, "C172", Aircraft::Category::LightProp);

    EXPECT_GT(jetProfile.landingTouchdownDistanceMeters, lightProfile.landingTouchdownDistanceMeters);
    EXPECT_GT(jetProfile.landingTouchdownSpeedKt, lightProfile.landingTouchdownSpeedKt);
    EXPECT_GT(jetProfile.landingExitSpeedKt, lightProfile.landingExitSpeedKt);
}

TEST(AircraftPerformanceTableTest, helicoptersCanOperateWithoutRunwayLengthConstraint)
{
    EXPECT_FLOAT_EQ(AircraftPerformanceTable::minimumRunwayLengthMeters(Aircraft::Category::Helicopter), 0.0f);
    EXPECT_TRUE(AircraftPerformanceTable::canOperateFromRunwayLengthMeters(
        Aircraft::Category::Helicopter,
        AircraftPerformanceTable::minimumRunwayLengthMeters(Aircraft::Category::Helicopter)));
}
