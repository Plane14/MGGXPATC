//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#include <sstream>
#include <vector>
#include "gtest/gtest.h"
#include "libworld.h"
#include "libdataxp.h"
#include "libworld_test.h"
#include "libdataxp_test.h"

using namespace world;
using namespace dataxp;
using namespace std;

TEST(OpenAirReaderTest, parseSingleAirspace)
{
    stringstream input;
    input << "AC D\n";
    input << "AN TEST AREA\n";
    input << "AL GND\n";
    input << "AH 2500 MSL\n";
    input << "DP  26:50:47 N  080:16:49 W\n";
    input << "DP  26:50:24 N  080:21:49 W\n";
    input << "DP  26:51:07 N  080:23:03 W\n";
    input << "\n";

    OpenAirAirspaceReader reader(makeHost());
    auto entries = reader.read(input);

    EXPECT_EQ(entries.size(), 1);
    EXPECT_EQ(entries[0].classCode, "D");
    EXPECT_EQ(entries[0].name, "TEST AREA");
    EXPECT_EQ(entries[0].lowerLimit, "GND");
    EXPECT_EQ(entries[0].upperLimit, "2500 MSL");
    EXPECT_EQ(entries[0].polygon.size(), 3);
    EXPECT_NEAR(entries[0].polygon[0].latitude, 26.846389, 0.0001);
    EXPECT_NEAR(entries[0].polygon[0].longitude, -80.280278, 0.0001);
}

TEST(OpenAirReaderTest, parseMultipleAirspaces)
{
    stringstream input;
    input << "AC A\n";
    input << "AN AREA ONE\n";
    input << "AL FL100\n";
    input << "AH FL600\n";
    input << "DP  0:00:00 N  000:00:00 E\n";
    input << "DP  1:00:00 N  000:00:00 E\n";
    input << "DP  1:00:00 N  001:00:00 E\n";
    input << "\n";
    input << "AC C\n";
    input << "AN AREA TWO\n";
    input << "AL 1000 ft\n";
    input << "AH 3500 ft\n";
    input << "DP  10:00:00 S  010:00:00 W\n";
    input << "DP  11:00:00 S  010:00:00 W\n";
    input << "DP  11:00:00 S  011:00:00 W\n";
    input << "\n";

    OpenAirAirspaceReader reader(makeHost());
    auto entries = reader.read(input);

    EXPECT_EQ(entries.size(), 2);
    EXPECT_EQ(entries[0].classCode, "A");
    EXPECT_EQ(entries[0].name, "AREA ONE");
    EXPECT_EQ(entries[1].classCode, "C");
    EXPECT_EQ(entries[1].name, "AREA TWO");
    EXPECT_EQ(entries[1].polygon.size(), 3);
    EXPECT_NEAR(entries[1].polygon[0].latitude, -10.0, 0.0001);
    EXPECT_NEAR(entries[1].polygon[0].longitude, -10.0, 0.0001);
}

TEST(OpenAirReaderTest, parseEmptyStream)
{
    stringstream input;
    OpenAirAirspaceReader reader(makeHost());
    auto entries = reader.read(input);
    EXPECT_EQ(entries.size(), 0);
}

TEST(XPAtcDatReaderTest, parseSingleController)
{
    stringstream input;
    input << "A\n";
    input << "1000\n";
    input << "ATCFILE\n";
    input << "\n";
    input << "CONTROLLER\n";
    input << "NAME ALGIERS\n";
    input << "FACILITY_ID DAAA\n";
    input << "ROLE ctr\n";
    input << "FREQ 12045\n";
    input << "FREQ 12380\n";
    input << "AIRSPACE_POLYGON_BEGIN 0 60000\n";
    input << "POINT 39.000000 4.666667\n";
    input << "POINT 39.000000 4.685000\n";
    input << "POINT 39.100000 4.685000\n";
    input << "AIRSPACE_POLYGON_END\n";
    input << "CONTROLLER_END\n";

    XPAtcDatReader reader(makeHost());
    auto entries = reader.read(input);

    EXPECT_EQ(entries.size(), 1);
    EXPECT_EQ(entries[0].name, "ALGIERS");
    EXPECT_EQ(entries[0].facilityId, "DAAA");
    EXPECT_EQ(entries[0].role, "ctr");
    EXPECT_EQ(entries[0].frequenciesKhz.size(), 2);
    EXPECT_EQ(entries[0].frequenciesKhz[0], 12045);
    EXPECT_EQ(entries[0].frequenciesKhz[1], 12380);
    EXPECT_EQ(entries[0].airspaces.size(), 1);
    EXPECT_FLOAT_EQ(entries[0].airspaces[0].floorFeet, 0.0f);
    EXPECT_FLOAT_EQ(entries[0].airspaces[0].ceilingFeet, 60000.0f);
    EXPECT_EQ(entries[0].airspaces[0].points.size(), 3);
    EXPECT_DOUBLE_EQ(entries[0].airspaces[0].points[0].latitude, 39.0);
    EXPECT_DOUBLE_EQ(entries[0].airspaces[0].points[0].longitude, 4.666667);
}

TEST(XPAtcDatReaderTest, parseMultipleControllers)
{
    stringstream input;
    input << "CONTROLLER\n";
    input << "NAME TWR1\n";
    input << "FACILITY_ID KJFK\n";
    input << "ROLE twr\n";
    input << "FREQ 11910\n";
    input << "CONTROLLER_END\n";
    input << "CONTROLLER\n";
    input << "NAME APP1\n";
    input << "FACILITY_ID KJFK\n";
    input << "ROLE app\n";
    input << "FREQ 12850\n";
    input << "CONTROLLER_END\n";

    XPAtcDatReader reader(makeHost());
    auto entries = reader.read(input);

    EXPECT_EQ(entries.size(), 2);
    EXPECT_EQ(entries[0].name, "TWR1");
    EXPECT_EQ(entries[0].role, "twr");
    EXPECT_EQ(entries[1].name, "APP1");
    EXPECT_EQ(entries[1].role, "app");
}

TEST(XPAtcDatReaderTest, parseEmptyStream)
{
    stringstream input;
    XPAtcDatReader reader(makeHost());
    auto entries = reader.read(input);
    EXPECT_EQ(entries.size(), 0);
}

TEST(XPAirportReaderTest, readHoldingPattern1140)
{
    stringstream aptDat = makeAptDat({
        "1 123 0 0 KBFI Boeing Field",
        "1140 47.440000 -122.310000 R 5.0 N HOLD_NORTH",
        "1140 47.430000 -122.300000 L 4.0 T HOLD_SOUTH"
    });

    XPAirportReader builder(makeHost());
    builder.readAirport(aptDat);
    const auto airport = builder.getAirport();

    const auto& holds = airport->holdingPatterns();
    EXPECT_EQ(holds.size(), 2);

    EXPECT_EQ(holds[0]->name(), "HOLD_NORTH");
    EXPECT_NEAR(holds[0]->location().latitude, 47.44, 0.001);
    EXPECT_NEAR(holds[0]->location().longitude, -122.31, 0.001);
    EXPECT_EQ(holds[0]->turnDirection(), HoldingPattern::TurnDirection::Right);
    EXPECT_FLOAT_EQ(holds[0]->legValue(), 5.0f);
    EXPECT_EQ(holds[0]->legType(), HoldingPattern::LegType::Distance);

    EXPECT_EQ(holds[1]->name(), "HOLD_SOUTH");
    EXPECT_NEAR(holds[1]->location().latitude, 47.43, 0.001);
    EXPECT_NEAR(holds[1]->location().longitude, -122.30, 0.001);
    EXPECT_EQ(holds[1]->turnDirection(), HoldingPattern::TurnDirection::Left);
    EXPECT_FLOAT_EQ(holds[1]->legValue(), 4.0f);
    EXPECT_EQ(holds[1]->legType(), HoldingPattern::LegType::Time);
}

TEST(XPAirportReaderTest, readHoldingPattern1140_defaultName)
{
    stringstream aptDat = makeAptDat({
        "1 123 0 0 KBFI Boeing Field",
        "1140 47.440000 -122.310000 R 5.0 N"
    });

    XPAirportReader builder(makeHost());
    builder.readAirport(aptDat);
    const auto airport = builder.getAirport();

    const auto& holds = airport->holdingPatterns();
    EXPECT_EQ(holds.size(), 1);
    EXPECT_EQ(holds[0]->name(), "HOLD");
}
