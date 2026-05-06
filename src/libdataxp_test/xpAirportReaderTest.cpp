// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include "gtest/gtest.h"
#include "libworld.h"
#include "libdataxp.h"
#include "libworld_test.h"
#include "libdataxp_test.h"

//#include <direct.h>
//#define GetCurrentDir _getcwd

using namespace world;
using namespace std;

namespace
{
    class NavdataHostServices : public TestHostServices
    {
    private:
        string m_hostRoot;
        unordered_map<string, string> m_fileContentsByPath;
    public:
        explicit NavdataHostServices(const string& hostRoot) :
            m_hostRoot(hostRoot)
        {
        }
    public:
        string getHostFilePath(const vector<string>& relativePathParts) override
        {
            string fullPath = m_hostRoot;
            for (const auto& part : relativePathParts)
            {
                fullPath.append("/");
                fullPath.append(part);
            }
            return fullPath;
        }

        shared_ptr<istream> openFileForRead(const string& filePath) override
        {
            const auto found = m_fileContentsByPath.find(filePath);
            if (found != m_fileContentsByPath.end())
            {
                return shared_ptr<istream>(new stringstream(found->second));
            }

            return TestHostServices::openFileForRead(filePath);
        }

        void addHostFile(const vector<string>& relativePathParts, const vector<string>& lines)
        {
            stringstream output;
            output.exceptions(ios::failbit | ios::badbit);
            for (const auto& line : lines)
            {
                output << line << endl;
            }

            m_fileContentsByPath[getHostFilePath(relativePathParts)] = output.str();
        }
    };

    shared_ptr<NavdataHostServices> makeNavdataHost(
        const string& testName,
        const vector<string>& atcLines,
        const vector<string>& airspaceLines = {})
    {
        auto host = make_shared<NavdataHostServices>("HOST_DIR_" + testName);
        host->addHostFile({ "Custom Data", "1200 atc data", "Earth nav data", "atc.dat" }, atcLines);
        if (!airspaceLines.empty())
        {
            host->addHostFile({ "Custom Data", "airspaces", "airspace.txt" }, airspaceLines);
        }
        return host;
    }

    shared_ptr<ControllerPosition> findTowerPosition(
        const shared_ptr<Airport>& airport,
        ControllerPosition::Type type)
    {
        if (!airport || !airport->tower())
        {
            return nullptr;
        }

        for (const auto& position : airport->tower()->positions())
        {
            if (position && position->type() == type)
            {
                return position;
            }
        }

        return nullptr;
    }
}

// std::string GetCurrentWorkingDir( void ) {
//   char buff[FILENAME_MAX];
//   GetCurrentDir( buff, FILENAME_MAX );
//   std::string current_working_dir(buff);
//   return current_working_dir;
// }
 
// void writeAirportJson(shared_ptr<const Airport> airport, ostream& output);
// void writeTaxiPathJson(shared_ptr<const TaxiPath> taxiPath, ostream& output);

TEST(XPAirportReaderTest, readAptDat_empty) {
    XPAirportReader builder(makeHost());
    const auto airport = builder.getAirport();
    const auto taxiNet = airport->taxiNet();

    EXPECT_EQ(taxiNet->nodes().size(), 0);
    EXPECT_EQ(taxiNet->edges().size(), 0);
    EXPECT_EQ(airport->runways().size(), 0);
}

TEST(XPAirportReaderTest, readAptDat_header) {
    XPAirportReader builder(makeHost());
    stringstream aptDat = makeAptDat({"1 123 0 0 KBFI Boeing Field King Co Intl"});

    builder.readAirport(aptDat);

    const auto airport = builder.getAirport();

    EXPECT_EQ(airport->header().icao(), "KBFI");
    EXPECT_EQ(airport->header().name(), "Boeing Field King Co Intl");
    EXPECT_FLOAT_EQ(airport->header().elevation(), 123);
}

TEST(XPAirportReaderTest, readAptDat_metadata) {
    XPAirportReader builder(makeHost());
    stringstream aptDat = makeAptDat({
        "1 123 0 0 KBFI Boeing Field King Co Intl",
        "1302 datum_lat 47.44",
        "1302 datum_lon -122.31",
        "1302 iata_code SEA",
        "1302 icao_code KSEA",
    });

    builder.readAirport(aptDat);

    const auto airport = builder.getAirport();

    EXPECT_EQ(airport->header().icao(), "KSEA");
    EXPECT_EQ(airport->header().name(), "Boeing Field King Co Intl");
    EXPECT_FLOAT_EQ(airport->header().elevation(), 123);
    EXPECT_FLOAT_EQ(airport->header().datum().latitude, 47.44);
    EXPECT_FLOAT_EQ(airport->header().datum().longitude, -122.31);
    EXPECT_EQ(airport->header().iata(), "SEA");
}

TEST(XPAirportReaderTest, readAptDat_singleTaxiNode) {
    XPAirportReader builder(makeHost());
    stringstream aptDat = makeAptDat({"1201  32.1234  034.5678 both 231 K3_stop"});

    builder.readAirport(aptDat);
    const auto airport = builder.getAirport();

    const auto taxiNet = airport->taxiNet();
    ASSERT_EQ(taxiNet->nodes().size(), 1);
    EXPECT_EQ(taxiNet->nodes()[0]->id(), 231);
    EXPECT_FLOAT_EQ(taxiNet->nodes()[0]->location().x(), 3456.78);
    EXPECT_FLOAT_EQ(taxiNet->nodes()[0]->location().z(), 3212.34);
    EXPECT_FLOAT_EQ(taxiNet->nodes()[0]->location().y(), 0);
    EXPECT_EQ(taxiNet->nodes()[0]->isJunction(), false);
    EXPECT_TRUE(taxiNet->nodes()[0]->isRouteStart());
    EXPECT_TRUE(taxiNet->nodes()[0]->isRouteEnd());
}

TEST(XPAirportReaderTest, readAptDat_singleTaxiEdge) {
    XPAirportReader builder(makeHost());
    stringstream aptDat = makeAptDat({
        "1201  32.1234  034.5678 init 231 K3 stop 1",
        "1201  33.2345  035.6789 dest 233 K3_stop_2",
        "1202  231  233  twoway  taxiway  K3 E1"
    });

    builder.readAirport(aptDat);
    const auto airport = builder.getAirport();

    const auto taxiNet = airport->taxiNet();
    ASSERT_EQ(taxiNet->nodes().size(), 2);
    
    EXPECT_EQ(taxiNet->nodes()[0]->id(), 231);
    EXPECT_FLOAT_EQ(taxiNet->nodes()[0]->location().x(), 3456.78);
    EXPECT_FLOAT_EQ(taxiNet->nodes()[0]->location().z(), 3212.34);
    EXPECT_FLOAT_EQ(taxiNet->nodes()[0]->location().y(), 0);
    EXPECT_TRUE(taxiNet->nodes()[0]->isRouteStart());
    EXPECT_FALSE(taxiNet->nodes()[0]->isRouteEnd());
    
    EXPECT_EQ(taxiNet->nodes()[1]->id(), 233);
    EXPECT_FLOAT_EQ(taxiNet->nodes()[1]->location().x(), 3567.89);
    EXPECT_FLOAT_EQ(taxiNet->nodes()[1]->location().z(), 3323.45);
    EXPECT_FLOAT_EQ(taxiNet->nodes()[1]->location().y(), 0);
    EXPECT_FALSE(taxiNet->nodes()[1]->isRouteStart());
    EXPECT_TRUE(taxiNet->nodes()[1]->isRouteEnd());

    ASSERT_EQ(taxiNet->edges().size(), 1);
    EXPECT_EQ(taxiNet->edges()[0]->id(), 1001);
    EXPECT_EQ(taxiNet->edges()[0]->type(), TaxiEdge::Type::Taxiway);
    EXPECT_EQ(taxiNet->edges()[0]->isOneWay(), false);    
    EXPECT_EQ(taxiNet->edges()[0]->name(), "K3 E1");
    EXPECT_EQ(taxiNet->edges()[0]->node1(), taxiNet->nodes()[0]);
    EXPECT_EQ(taxiNet->edges()[0]->node2(), taxiNet->nodes()[1]);

    //writeAirportJson(airport, cout);
}

TEST(XPAirportReaderTest, routeAwareTaxiPathsPreferRouteNodes) {
    auto host = makeHost();

    auto n1 = shared_ptr<TaxiNode>(new TaxiNode(1, UniPoint::fromGeo(host, {40.00000000, -073.00000000})));
    auto n2 = shared_ptr<TaxiNode>(new TaxiNode(2, UniPoint::fromGeo(host, {40.00000000, -073.00100000})));
    auto n3 = shared_ptr<TaxiNode>(new TaxiNode(3, UniPoint::fromGeo(host, {40.00000000, -073.00200000})));
    auto n4 = shared_ptr<TaxiNode>(new TaxiNode(4, UniPoint::fromGeo(host, {40.00000000, -072.99950000})));
    auto n5 = shared_ptr<TaxiNode>(new TaxiNode(5, UniPoint::fromGeo(host, {40.00000000, -073.00450000})));
    auto n6 = shared_ptr<TaxiNode>(new TaxiNode(6, UniPoint::fromGeo(host, {40.00000000, -073.00400000})));

    n1->setRouteStart(true);
    n2->setRouteStart(true);
    n2->setRouteEnd(true);
    n3->setRouteEnd(true);
    n4->setJunction(true);
    n5->setRouteEnd(true);

    auto e12 = shared_ptr<TaxiEdge>(new TaxiEdge(12, "A1", 1, 2));
    // e23 is the runway edge connecting taxiway to runway node n3
    auto e23 = shared_ptr<TaxiEdge>(new TaxiEdge(23, "09", 2, 3, TaxiEdge::Type::Runway));
    auto e42 = shared_ptr<TaxiEdge>(new TaxiEdge(42, "A3", 4, 2));
    auto e25 = shared_ptr<TaxiEdge>(new TaxiEdge(25, "A4", 2, 5));
    auto e26 = shared_ptr<TaxiEdge>(new TaxiEdge(26, "A5", 2, 6));

    auto runway = shared_ptr<Runway>(new Runway(
        Runway::End("09", 0, 0, n3->location()),
        Runway::End("27", 0, 0, UniPoint::fromGeo(host, {40.00000000, -073.01000000})),
        45.0f));

    auto gate = make_shared<ParkingStand>(
        1,
        "Gate 6",
        ParkingStand::Type::Gate,
        UniPoint::fromGeo(host, {40.00000000, -073.00420000}),
        90.0f,
        "C",
        Aircraft::Category::Jet,
        Aircraft::OperationType::Airline);

    auto airport = WorldBuilder::assembleAirport(
        host,
        Airport::Header("KRTT", "Route Taxi Test", GeoPoint(40.00000000, -073.00500000), 0),
        { runway },
        { gate },
        { n1, n2, n3, n4, n5, n6 },
        { e12, e23, e42, e25, e26 });

    const auto taxiNet = airport->taxiNet();
    const auto& runwayEnd = airport->getRunwayOrThrow("09")->getEndOrThrow("09");

    EXPECT_TRUE(taxiNet->getNodeById(1)->isRouteStart());
    EXPECT_TRUE(taxiNet->getNodeById(2)->isRouteStart());
    EXPECT_TRUE(taxiNet->getNodeById(2)->isRouteEnd());
    EXPECT_TRUE(taxiNet->getNodeById(3)->isRouteEnd());
    EXPECT_TRUE(taxiNet->getNodeById(4)->isJunction());
    EXPECT_TRUE(taxiNet->getNodeById(5)->isRouteEnd());

    auto departurePath = taxiNet->tryFindDepartureTaxiPathToRunway(
        GeoPoint(40.00000000, -072.99970000),
        runwayEnd);
    ASSERT_TRUE(!!departurePath);
    ASSERT_EQ(departurePath->fromNode->id(), 1);
    // Path ends at runway node (3) since there's no hold-short edge with active zone
    ASSERT_EQ(departurePath->toNode->id(), 3);
    ASSERT_EQ(departurePath->edges.size(), 2);
    EXPECT_EQ(departurePath->edges[0]->node1()->id(), 1);
    EXPECT_EQ(departurePath->edges[1]->node2()->id(), 3);

    auto arrivalPath = taxiNet->tryFindTaxiPathToGate(
        gate,
        GeoPoint(40.00000000, -073.00000000));
    ASSERT_TRUE(!!arrivalPath);
    ASSERT_GE(arrivalPath->edges.size(), 4);
    EXPECT_EQ(arrivalPath->fromNode->id(), 1);
    EXPECT_EQ(arrivalPath->edges[arrivalPath->edges.size() - 3]->node2()->id(), 5);
}

TEST(XPAirportReaderTest, readAptDat_singleTaxiAndGroundEdge) {
    XPAirportReader builder(makeHost());
    stringstream aptDat = makeAptDat({
        "1201  32.1234  034.5678 init 231 K3 stop 1",
        "1201  33.2345  035.6789 dest 233 K3_stop_2",
        "1201  33.2345  035.7890 dest 234 K3_svc_1",
        "1202  231  233  twoway  taxiway  K3 E1",
        "1206  233  234  twoway  GR12"
    });

    builder.readAirport(aptDat);
    const auto airport = builder.getAirport();

    const auto taxiNet = airport->taxiNet();
    ASSERT_EQ(taxiNet->nodes().size(), 3);
    EXPECT_EQ(taxiNet->nodes()[0]->id(), 231);
    EXPECT_EQ(taxiNet->nodes()[1]->id(), 233);
    EXPECT_EQ(taxiNet->nodes()[2]->id(), 234);

    ASSERT_EQ(taxiNet->edges().size(), 2);
    EXPECT_EQ(taxiNet->edges()[1]->id(), 1002);
    EXPECT_EQ(taxiNet->edges()[1]->type(), TaxiEdge::Type::Groundway);
    EXPECT_EQ(taxiNet->edges()[1]->isOneWay(), false);
    EXPECT_EQ(taxiNet->edges()[1]->name(), "GR12");
    EXPECT_EQ(taxiNet->edges()[1]->node1()->id(), 233);
    EXPECT_EQ(taxiNet->edges()[1]->node2()->id(), 234);

    //writeAirportJson(airport, cout);
}

TEST(XPAirportReaderTest, readAptDat_runways) {
    XPAirportReader builder(makeHost());
    stringstream aptDat = makeAptDat({
        "100 46.02 1 0 0.00 1 3 0 13L  40.1234 -073.4567 277  0 3 2 1 0 31R  40.6437 -073.7592  314  0 3 8 1 0",
        "100 60.00 2 0 0.00 1 3 0 04L  40.2345 -073.5678 140  0 3 0 0 1 22R  40.6505 -073.7633 1044  0 3 0 0 0",
    });

    builder.readAirport(aptDat);
    const auto airport = builder.getAirport();

    const auto& runways = airport->runways();

    ASSERT_EQ(runways.size(), 2);

    const auto rwy1 = runways[0];
    EXPECT_EQ(rwy1->end1().name(), "13L");
    EXPECT_EQ(rwy1->end2().name(), "31R");
    EXPECT_EQ(
        rwy1->end1().heading(), 
        GeoMath::getHeadingFromPoints({40.1234, -073.4567}, {40.6437, -073.7592}));
    EXPECT_EQ(
        rwy1->end2().heading(), 
        GeoMath::getHeadingFromPoints({40.6437, -073.7592}, {40.1234, -073.4567}));
    EXPECT_FLOAT_EQ(rwy1->widthMeters(), 46.02);

    const auto rwy2 = runways[1];
    EXPECT_EQ(rwy2->end1().name(), "04L");
    EXPECT_EQ(rwy2->end2().name(), "22R");
    EXPECT_FLOAT_EQ(rwy2->widthMeters(), 60.00);
}

TEST(XPAirportReaderTest, readAptDat_assignRunwaysHeaderElevation) {
    XPAirportReader builder(makeHost());
    stringstream aptDat = makeAptDat({
        "1 1234 0 0 ABCD Test",
        "100 46.02 1 0 0.00 1 3 0 13L  40.1234 -073.4567 277  0 3 2 1 0 31R  40.6437 -073.7592  314  0 3 8 1 0",
        "100 60.00 2 0 0.00 1 3 0 04L  40.2345 -073.5678 140  0 3 0 0 1 22R  40.6505 -073.7633 1044  0 3 0 0 0",
    });

    builder.readAirport(aptDat);
    const auto airport = builder.getAirport();
    const auto& runways = airport->runways();

    ASSERT_EQ(runways.size(), 2);
    EXPECT_FLOAT_EQ(runways[0]->end1().elevationFeet(), 1234);
    EXPECT_FLOAT_EQ(runways[0]->end2().elevationFeet(), 1234);
    EXPECT_FLOAT_EQ(runways[1]->end1().elevationFeet(), 1234);
    EXPECT_FLOAT_EQ(runways[1]->end2().elevationFeet(), 1234);
}

TEST(XPAirportReaderTest, readAptDat_runwayEdges) {
    XPAirportReader builder(makeHost());
    stringstream aptDat = makeAptDat({
        "100 45.00 1 0 0.00 1 3 0 06  40.100 050.200 277  0 3 2 1 0 22  40.130 050.280  314  0 3 8 1 0",
        "100 60.00 1 0 0.00 1 3 0 14  40.160 050.210 277  0 3 2 1 0 32  40.100 050.270  314  0 3 8 1 0",
        "",
        "# nodes",
        "",
        "1201  40.100 50.200 both 11 n1",
        "1201  40.120 50.250 both 22 n2",
        "1201  40.130 50.280 both 33 n3",
        "1201  40.140 50.280 both 44 n4",
        "1201  40.130 50.250 both 55 n5",
        "1201  40.130 50.240 both 66 n6",
        "1201  40.160 50.210 both 77 n7",
        "1201  40.150 50.200 both 88 n8",
        "1201  40.120 50.230 both 99 n9",
        "1201  40.100 50.270 both 110 n10",
        "",
        "# edges - runways",
        "",
        "1202 11 22 twoway runway 06/22",
        "1204 departure 06,22",
        "1204 arrival 06,22",
        "1204 ils 06,22",
        "1202 22 33 twoway runway 06/22",
        "1204 departure 06,22",
        "1204 arrival 06,22",
        "1204 ils 06,22",
        "1202 77 66 twoway runway 14/32",
        "1204 departure 14,32",
        "1204 arrival 14,32",
        "1204 ils 14,32",
        "1202 66 22 twoway runway 14/32",
        "1204 departure 14,32",
        "1204 arrival 14,32",
        "1204 ils 14,32",
        "1202 22 110 twoway runway 14/32",
        "1204 departure 14,32",
        "1204 arrival 14,32",
        "1204 ils 14,32",
        "",
        "# edges - taxiways",
        "",
        "1202 77 88 twoway taxiway_F A1",
        "1204 departure 14,32",
        "1204 ils 14,32",
        "1202 88 99 twoway taxiway_F A",
        "1202 99 66 twoway taxiway_F A2",
        "1204 departure 14,32",
        "1204 arrival 14,32",
        "1202 33 44 twoway taxiway_E B1",
        "1204 departure 06,22",
        "1204 ils 06,22",
        "1202 44 55 twoway taxiway_E B",
        "1202 55 22 twoway taxiway_E B2",
        "1204 arrival 06,22,14,32",
        "1202 66 55 twoway taxiway_D C1",
        "1204 arrival 06,22",
        "1202 99 22 twoway taxiway_D C2",
        "1204 arrival 06,22,14,32",
    });

    builder.readAirport(aptDat);
    const auto airport = builder.getAirport();
    const auto taxiNet = airport->taxiNet();
    
    const auto rwy0622 = airport->getRunwayOrThrow("06/22");
    const auto rwy1432 = airport->getRunwayOrThrow("14/32");
    auto n1 = taxiNet->getNodeById(11);
    auto n2 = taxiNet->getNodeById(22);
    auto n3 = taxiNet->getNodeById(33);
    auto n4 = taxiNet->getNodeById(44);
    auto n5 = taxiNet->getNodeById(55);
    auto n6 = taxiNet->getNodeById(66);
    auto n7 = taxiNet->getNodeById(77);
    auto n8 = taxiNet->getNodeById(88);
    auto n9 = taxiNet->getNodeById(99);
    auto n10 = taxiNet->getNodeById(110);

    auto e12_0622 = n1->getEdgeTo(n2);
    auto e23_0622 = n2->getEdgeTo(n3);
    auto e34_b1   = n3->getEdgeTo(n4);
    auto e45_b    = n4->getEdgeTo(n5);
    auto e52_b2   = n5->getEdgeTo(n2);
    auto e67_1432 = n6->getEdgeTo(n7);
    auto e62_1432 = n6->getEdgeTo(n2);
    auto e210_1432 = n2->getEdgeTo(n10);
    auto e78_a1   = n7->getEdgeTo(n8);
    auto e89_a    = n8->getEdgeTo(n9);
    auto e96_a2   = n9->getEdgeTo(n6);
    auto e56_c1   = n5->getEdgeTo(n6);
    auto e92_c2   = n9->getEdgeTo(n2);

    EXPECT_EQ(e12_0622->type(), TaxiEdge::Type::Runway);
    EXPECT_EQ(e12_0622->runway(), rwy0622);
    EXPECT_EQ(e23_0622->type(), TaxiEdge::Type::Runway);
    EXPECT_EQ(e23_0622->runway(), rwy0622);

    EXPECT_EQ(e67_1432->type(), TaxiEdge::Type::Runway);
    EXPECT_EQ(e67_1432->runway(), rwy1432);
    EXPECT_EQ(e62_1432->type(), TaxiEdge::Type::Runway);
    EXPECT_EQ(e62_1432->runway(), rwy1432);
    EXPECT_EQ(e210_1432->type(), TaxiEdge::Type::Runway);
    EXPECT_EQ(e210_1432->runway(), rwy1432);

    EXPECT_EQ(e78_a1->type(), TaxiEdge::Type::Taxiway);
    EXPECT_TRUE(e78_a1->activeZones().departue.has(rwy1432));
    EXPECT_TRUE(e78_a1->activeZones().ils.has(rwy1432));
    EXPECT_FALSE(e78_a1->activeZones().arrival.has(rwy1432));
    EXPECT_FALSE(e78_a1->activeZones().departue.has(rwy0622));

    EXPECT_EQ(e92_c2->type(), TaxiEdge::Type::Taxiway);
    EXPECT_FALSE(e92_c2->activeZones().departue.hasAny());
    EXPECT_FALSE(e92_c2->activeZones().ils.hasAny());
    EXPECT_TRUE(e92_c2->activeZones().arrival.has(rwy0622));
    EXPECT_TRUE(e92_c2->activeZones().arrival.has(rwy1432));

    EXPECT_EQ(e45_b->type(), TaxiEdge::Type::Taxiway);
    EXPECT_FALSE(e45_b->activeZones().hasAny());

    //writeAirportJson(airport, cout);
}

TEST(XPAirportReaderTest, readAptDat_parkingStands) {
    XPAirportReader reader(makeHost());
    stringstream aptDat = makeAptDat({
        "1300  40.100 -073.200 152.39 gate heavy|jets|turboprops T1 5",
        "1301 F airline AAL, SWA ",
        "1300  40.110 -073.220 105.95 hangar turboprops|props|helos T1 3",
        "1301 E cargo",
    });

    reader.readAirport(aptDat);
    const auto airport = reader.getAirport();
    const vector<shared_ptr<ParkingStand>>& parkingStands = airport->parkingStands();

    shared_ptr<ParkingStand> gate_t15 = airport->getParkingStandOrThrow("T1 5");
    shared_ptr<ParkingStand> gate_t13 = airport->getParkingStandOrThrow("T1 3");

    ASSERT_EQ(parkingStands.size(), 2);
    EXPECT_EQ(parkingStands[0], gate_t15);
    EXPECT_EQ(parkingStands[1], gate_t13);

    EXPECT_EQ(gate_t15->id(), 301);
    EXPECT_EQ(gate_t15->name(), "T1 5");
    EXPECT_EQ(gate_t15->type(), ParkingStand::Type::Gate);
    EXPECT_FLOAT_EQ(gate_t15->location().latitude(), 40.100);
    EXPECT_FLOAT_EQ(gate_t15->location().longitude(), -73.200);
    EXPECT_FLOAT_EQ(gate_t15->heading(), 152.39);
    EXPECT_EQ(gate_t15->widthCode(), "F");
    EXPECT_EQ(gate_t15->aircraftCategories(), Aircraft::Category::Heavy | Aircraft::Category::Jet | Aircraft::Category::Turboprop);
    EXPECT_EQ(gate_t15->operationTypes(), Aircraft::OperationType::Airline);
    ASSERT_EQ(gate_t15->airlines().size(), 2);
    EXPECT_EQ(gate_t15->airlines()[0], "AAL");
    EXPECT_EQ(gate_t15->airlines()[1], "SWA");

    EXPECT_EQ(gate_t13->id(), 302);
    EXPECT_EQ(gate_t13->name(), "T1 3");
    EXPECT_EQ(gate_t13->type(), ParkingStand::Type::Hangar);
    EXPECT_FLOAT_EQ(gate_t13->location().latitude(), 40.110);
    EXPECT_FLOAT_EQ(gate_t13->location().longitude(), -73.220);
    EXPECT_FLOAT_EQ(gate_t13->heading(), 105.95);
    EXPECT_EQ(gate_t13->widthCode(), "E");
    EXPECT_EQ(
        gate_t13->aircraftCategories(), 
        Aircraft::Category::Turboprop | Aircraft::Category::Prop | Aircraft::Category::Helicopter);
    EXPECT_EQ(gate_t13->operationTypes(), Aircraft::OperationType::Cargo);
    EXPECT_EQ(gate_t13->airlines().size(), 0);

    //writeAirportJson(airport, cout);
}

TEST(XPAirportReaderTest, readAptDat_skipUnrecognizedLines) {
    XPAirportReader builder(makeHost());
    stringstream aptDat = makeAptDat({
        "I",
        "1000 Generated by WorldEditor",        
        "  ",
        "1     13 1 0 KJFK John F Kennedy Intl",
        "1302 city New York",
        "1302 country United States",
        "1201  32.1234  034.5678 both 231 K3_stop",
        "987655  1  12  13  ",
        ""
    });

    builder.readAirport(aptDat);
    const auto airport = builder.getAirport();
    const auto taxiNet = airport->taxiNet();

    ASSERT_EQ(taxiNet->nodes().size(), 1);
}

TEST(XPAirportReaderTest, readAptDat_realKJFK) {
    XPAirportReader reader(makeHost());
    ifstream aptDat;
    openTestInputStream("apt_kjfk.dat", aptDat);

    reader.readAirport(aptDat);
    const auto airport = reader.getAirport();

    assertRunwaysExist(airport, { "04L", "04R", "13L", "13R", "31L", "31R", "22L", "22R" });
    assertGatesExist(airport, { "DelEx Cargo 1", "Korea Cargo", "T8 12", "Prologis Cargo 2", "T4 55" });
    assertTaxiEdgesExist(airport, { "P", "F", "PF", "VA", "W", "MB", "K2", "Z", "TA", "TB" });
}

TEST(XPAirportReaderTest, findTaxiPath_KJFK_1) {
    XPAirportReader reader(makeHost());
    ifstream aptDat;
    aptDat.exceptions(ifstream::failbit | ifstream::badbit);
    aptDat.open("../../src/libdataxp_test/testInputs/apt_kjfk.dat");
    reader.readAirport(aptDat);
    ofstream jsonOutput;
    jsonOutput.exceptions(ofstream::failbit | ofstream::badbit);
    jsonOutput.open("../../src/libdataxp_test/testOutputs/taxi_kjfk_79_378.json", std::ios_base::out | std::ios_base::trunc);
    
    const auto airport = reader.getAirport();
    const auto taxiNet = airport->taxiNet();
    const auto taxiPath = TaxiPath::find(
        taxiNet, 
        taxiNet->getNodeById(79), 
        taxiNet->getNodeById(378));

    //writeTaxiPathJson(taxiPath, jsonOutput);
}

TEST(XPAirportReaderTest, findTaxiPath_KJFK_2) {
    XPAirportReader reader(makeHost());
    ifstream aptDat;
    aptDat.exceptions(ifstream::failbit | ifstream::badbit);
    aptDat.open("../../src/libdataxp_test/testInputs/apt_kjfk.dat");
    reader.readAirport(aptDat);
    ofstream jsonOutput;
    jsonOutput.exceptions(ofstream::failbit | ofstream::badbit);
    jsonOutput.open("../../src/libdataxp_test/testOutputs/taxi_kjfk_79_581.json", std::ios_base::out | std::ios_base::trunc);
    
    const auto airport = reader.getAirport();
    const auto taxiNet = airport->taxiNet();
    const auto taxiPath = TaxiPath::find(
        taxiNet, 
        taxiNet->getNodeById(79), 
        taxiNet->getNodeById(581));

    //writeTaxiPathJson(taxiPath, jsonOutput);
}

TEST(XPAirportReaderTest, readAptDat_realKMIA) {
    XPAirportReader reader(makeHost());
    ifstream aptDat;
    openTestInputStream("apt_kmia.dat", aptDat);

    reader.readAirport(aptDat);
    auto airport = reader.getAirport();

    EXPECT_EQ(airport->header().iata(), "MIA");
    assertRunwaysExist(airport, { "12", "30", "09", "27", "08R", "08L", "26L", "26R" });
    assertGatesExist(airport, { "F19", "D4", "J49", "N7", "Western U Cargo 70" });
    assertTaxiEdgesExist(airport, { "S", "U", "V", "M", "Z", "K", "M10", "JJ", "S2" });
}

TEST(XPAirportReaderTest, readAptDat_realKORD) {
    XPAirportReader reader(makeHost());
    ifstream aptDat;
    openTestInputStream("apt_kord.dat", aptDat);

    reader.readAirport(aptDat);
    auto airport = reader.getAirport();

    assertRunwaysExist(airport, {
        "10L", "10C", "10R", "28L", "28C", "28R", "22L", "22R", "04L", "04R", "09L", "09R", "27L", "27R"
    });
    assertGatesExist(airport, { "Terminal 1 Gate B5", "Suparna Cargo", "Terminal 5 M25" });
    assertTaxiEdgesExist(airport, { "CC", "DD", "W", "PP", "TT", "R", "B", "A7", "RR", "AA" });
}

TEST(XPAirportReaderTest, readToEndOfLine) {
    stringstream aptDat1 = makeAptDat({ "no_whitespace\rABCD" });
    stringstream aptDat2 = makeAptDat({ "  leading_and_trailing_spaces  \r\nABCD" });
    stringstream aptDat3 = makeAptDat({ "  all kinds of\x20\x20spaces  \n\r\nABCD" });

    string text1 = XPAirportReader::readToEndOfLine(aptDat1);
    string text2 = XPAirportReader::readToEndOfLine(aptDat2);
    string text3 = XPAirportReader::readToEndOfLine(aptDat3);

    EXPECT_EQ(text1, "no_whitespace");
    EXPECT_EQ(text2, "leading_and_trailing_spaces");
    EXPECT_EQ(text3, "all kinds of\x20spaces");

    EXPECT_EQ(aptDat1.peek(), 'A');
    EXPECT_EQ(aptDat2.peek(), 'A');
    EXPECT_EQ(aptDat3.peek(), 'A');
}

TEST(XPAirportReaderTest, skipToNextLine) {
    stringstream aptDat = makeAptDat({ 
        "AAA no_spacing",
        "BBB regular spacing",
        "CCC   arbitrary   spacing   ",
        "DDD"
    });

    string token1 = XPAirportReader::readFirstToken(aptDat);
    XPAirportReader::skipToNextLine(aptDat);
    string token2 = XPAirportReader::readFirstToken(aptDat);
    XPAirportReader::skipToNextLine(aptDat);
    string token3 = XPAirportReader::readFirstToken(aptDat);
    XPAirportReader::skipToNextLine(aptDat);
    string token4 = XPAirportReader::readFirstToken(aptDat);

    EXPECT_EQ(token1, "AAA");
    EXPECT_EQ(token2, "BBB");
    EXPECT_EQ(token3, "CCC");
    EXPECT_EQ(token4, "DDD");
}

TEST(XPAirportReaderTest, readAptDat_assembleTower) {
    auto airspace = makeAirspace(40.63, -73.77, 10.0, "KJFK");
    stringstream aptDat = makeAptDat({
        "1    13 0 0 KJFK John F Kennedy Intl",
        "1302 datum_lat 40.63",
        "1302 datum_lon -73.77",
        "1050 128725 ATIS",
        "1051 122950 UNICOM",
        "1052 135050 CLNC DEL",
        "1053 121650 GND",
        "1053 121900 GND",
        "1053 125050 GND",
        "1054 119100 KENNEDY TWR",
        "1054 123900 KENNEDY TWR",
        "1055 123700 NEW YORK APP",
        "1055 126800 NEW YORK APP",
        "1055 127400 NEW YORK APP",
        "1055 132400 NEW YORK APP",
        "1056 123700 NEW YORK DEP",
        "1056 124750 NEW YORK DEP",
        "1056 134350 NEW YORK DEP",
        "1056 135900 NEW YORK DEP"
    });
    XPAirportReader builder(makeHost(), -1, [&](const Airport::Header& header) {
        return airspace;
    });
    builder.readAirport(aptDat);

    const auto airport = builder.getAirport();
    const auto tower = airport->tower();

    ASSERT_TRUE(!!tower);
    EXPECT_EQ(airport->tower().get(), tower.get());
    EXPECT_EQ(tower->type(), ControlFacility::Type::Tower);
    EXPECT_EQ(tower->callSign(), "J F K"); //TODO: Kennedy
    EXPECT_EQ(tower->airport().get(), airport.get());
    EXPECT_EQ(tower->airspace().get(), airspace.get());
    ASSERT_EQ(tower->positions().size(), 5);

    EXPECT_EQ(tower->positions()[0]->type(), ControllerPosition::Type::ClearanceDelivery);
    EXPECT_EQ(tower->positions()[0]->frequency()->khz(), 135050);
    EXPECT_EQ(tower->positions()[0]->callSign(), "J F K Clearance"); //TODO: "Kennedy Clearance"

    EXPECT_EQ(tower->positions()[1]->type(), ControllerPosition::Type::Ground);
    EXPECT_EQ(tower->positions()[1]->frequency()->khz(), 121650);
    EXPECT_EQ(tower->positions()[1]->callSign(), "J F K Ground"); //TODO: "Kennedy Ground"

    EXPECT_EQ(tower->positions()[2]->type(), ControllerPosition::Type::Local);
    EXPECT_EQ(tower->positions()[2]->frequency()->khz(), 119100);
    EXPECT_EQ(tower->positions()[2]->callSign(), "J F K Tower"); //TODO: "Kennedy Tower"

    EXPECT_EQ(tower->positions()[3]->type(), ControllerPosition::Type::Approach);
    EXPECT_EQ(tower->positions()[3]->frequency()->khz(), 123700);
    EXPECT_EQ(tower->positions()[3]->callSign(), "J F K Approach"); //TODO: "New York Approach"

    EXPECT_EQ(tower->positions()[4]->type(), ControllerPosition::Type::Departure);
    EXPECT_EQ(tower->positions()[4]->frequency()->khz(), 124750);
    EXPECT_EQ(tower->positions()[4]->callSign(), "J F K Departure"); //TODO: "New York Departure"
}

TEST(XPAirportReaderTest, readAptDat_augmentsControllerScopesFromAtcNavData)
{
    auto host = makeNavdataHost("nav_scope_augments", {
        "CONTROLLER",
        "NAME TEST AIRPORT",
        "FACILITY_ID KTST",
        "ROLE twr",
        "CLASS D",
        "ICAO KTST",
        "FREQ 11830",
        "AIRSPACE_POLYGON_BEGIN 0 3000",
        "POINT 40.0200 -73.9800",
        "POINT 40.0200 -73.9400",
        "POINT 40.0600 -73.9400",
        "POINT 40.0600 -73.9800",
        "AIRSPACE_POLYGON_END",
        "CONTROLLER_END",
        "CONTROLLER",
        "NAME TEST AIRPORT",
        "FACILITY_ID KTST",
        "ROLE tracon",
        "CLASS C",
        "ICAO KTST",
        "FREQ 12345",
        "FREQ 12455",
        "AIRSPACE_POLYGON_BEGIN 0 7000",
        "POINT 40.0000 -74.0000",
        "POINT 40.0000 -73.9000",
        "POINT 40.1000 -73.9000",
        "POINT 40.1000 -74.0000",
        "AIRSPACE_POLYGON_END",
        "CONTROLLER_END",
    });
    stringstream aptDat = makeAptDat({
        "1 10 0 0 KTST Test Airport",
        "1302 datum_lat 40.05",
        "1302 datum_lon -73.95",
        "1054 119100 TEST TWR",
        "1055 120700 TEST APP",
        "1056 121700 TEST DEP"
    });

    XPAirportReader reader(host, -1, [&](const Airport::Header& header) {
        return XPAtcNavData::queryAirportAirspace(host, header);
    });
    reader.readAirport(aptDat);

    const auto airport = reader.getAirport();
    ASSERT_TRUE(airport->tower());

    const auto local = findTowerPosition(airport, ControllerPosition::Type::Local);
    const auto approach = findTowerPosition(airport, ControllerPosition::Type::Approach);
    const auto departure = findTowerPosition(airport, ControllerPosition::Type::Departure);

    ASSERT_TRUE(local);
    ASSERT_TRUE(approach);
    ASSERT_TRUE(departure);

    EXPECT_EQ(local->frequency()->khz(), 119100);
    EXPECT_EQ(approach->frequency()->khz(), 120700);
    EXPECT_EQ(departure->frequency()->khz(), 121700);

    EXPECT_FALSE(local->radarScope()->scopeLimit().isEmpty());
    EXPECT_FALSE(approach->radarScope()->scopeLimit().isEmpty());
    EXPECT_FALSE(departure->radarScope()->scopeLimit().isEmpty());
    EXPECT_GT(local->radarScope()->scopeLimit().edges.size(), 1u);
    EXPECT_GT(approach->radarScope()->scopeLimit().edges.size(), 1u);
    EXPECT_GT(departure->radarScope()->scopeLimit().edges.size(), 1u);

    ASSERT_TRUE(airport->tower()->airspace());
    EXPECT_EQ(airport->tower()->airspace()->type(), ControlledAirspace::Type::TerminalControlArea);
    EXPECT_GT(airport->tower()->airspace()->geometry()->lateralBounds().edges.size(), 1u);
    EXPECT_TRUE(airport->tower()->airspace()->geometry()->hasLowerBound());
    EXPECT_TRUE(airport->tower()->airspace()->geometry()->hasUpperBound());
    EXPECT_FLOAT_EQ(airport->tower()->airspace()->geometry()->lowerBoundFeet(), 0.0f);
    EXPECT_FLOAT_EQ(airport->tower()->airspace()->geometry()->upperBoundFeet(), 7000.0f);
}

TEST(XPAirportReaderTest, readAptDat_addsControllerPositionsFromAtcNavDataWhenMissingInAptDat)
{
    auto host = makeNavdataHost("nav_add_positions", {
        "CONTROLLER",
        "NAME TEST AIRPORT",
        "FACILITY_ID KNAV",
        "ROLE twr",
        "CLASS D",
        "ICAO KNAV",
        "FREQ 11830",
        "AIRSPACE_POLYGON_BEGIN 0 3000",
        "POINT 41.0000 -74.0000",
        "POINT 41.0000 -73.9500",
        "POINT 41.0500 -73.9500",
        "POINT 41.0500 -74.0000",
        "AIRSPACE_POLYGON_END",
        "CONTROLLER_END",
        "CONTROLLER",
        "NAME TEST AIRPORT",
        "FACILITY_ID KNAV",
        "ROLE tracon",
        "CLASS C",
        "ICAO KNAV",
        "FREQ 12345",
        "FREQ 12455",
        "AIRSPACE_POLYGON_BEGIN 0 8000",
        "POINT 40.9500 -74.0500",
        "POINT 40.9500 -73.9000",
        "POINT 41.1000 -73.9000",
        "POINT 41.1000 -74.0500",
        "AIRSPACE_POLYGON_END",
        "CONTROLLER_END",
    });
    stringstream aptDat = makeAptDat({
        "1 10 0 0 KNAV Navdata Airport",
        "1302 datum_lat 41.02",
        "1302 datum_lon -73.97"
    });

    XPAirportReader reader(host, -1, [&](const Airport::Header& header) {
        return XPAtcNavData::queryAirportAirspace(host, header);
    });
    reader.readAirport(aptDat);

    const auto airport = reader.getAirport();
    ASSERT_TRUE(airport->tower());

    const auto local = findTowerPosition(airport, ControllerPosition::Type::Local);
    const auto approach = findTowerPosition(airport, ControllerPosition::Type::Approach);
    const auto departure = findTowerPosition(airport, ControllerPosition::Type::Departure);
    const auto area = findTowerPosition(airport, ControllerPosition::Type::Area);

    ASSERT_TRUE(local);
    ASSERT_TRUE(approach);
    ASSERT_TRUE(departure);
    ASSERT_TRUE(area);

    EXPECT_EQ(local->frequency()->khz(), 118300);
    EXPECT_EQ(approach->frequency()->khz(), 123450);
    EXPECT_EQ(departure->frequency()->khz(), 124550);
    EXPECT_EQ(area->frequency()->khz(), 123450);

    EXPECT_FALSE(local->radarScope()->scopeLimit().isEmpty());
    EXPECT_FALSE(approach->radarScope()->scopeLimit().isEmpty());
    EXPECT_FALSE(departure->radarScope()->scopeLimit().isEmpty());
    EXPECT_FALSE(area->radarScope()->scopeLimit().isEmpty());
}

TEST(XPAirportReaderTest, readAptDat_enrichesTerminalAirspaceNameFromAirspaceTxtAlias)
{
    auto host = makeNavdataHost(
        "nav_named_terminal_alias",
        {
            "CONTROLLER",
            "NAME NEW YORK",
            "FACILITY_ID KJFK",
            "ROLE twr",
            "CLASS D",
            "ICAO KJFK",
            "FREQ 11910",
            "AIRSPACE_POLYGON_BEGIN 0 3000",
            "POINT 40.6000 -73.9000",
            "POINT 40.6000 -73.7000",
            "POINT 40.7000 -73.7000",
            "POINT 40.7000 -73.9000",
            "AIRSPACE_POLYGON_END",
            "CONTROLLER_END",
            "CONTROLLER",
            "NAME NEW YORK",
            "FACILITY_ID KJFK",
            "ROLE tracon",
            "CLASS C",
            "ICAO KJFK",
            "FREQ 12370",
            "AIRSPACE_POLYGON_BEGIN 0 8000",
            "POINT 40.5500 -73.9500",
            "POINT 40.5500 -73.6500",
            "POINT 40.7500 -73.6500",
            "POINT 40.7500 -73.9500",
            "AIRSPACE_POLYGON_END",
            "CONTROLLER_END",
        },
        {
            "AC B",
            "AN NEW YORK",
            "AL GND",
            "AH 7000 MSL",
            "DP 40:33:00 N 073:57:00 W",
            "DP 40:33:00 N 073:39:00 W",
            "DP 40:45:00 N 073:39:00 W",
            "DP 40:45:00 N 073:57:00 W",
        });

    stringstream aptDat = makeAptDat({
        "1 13 0 0 KJFK John F Kennedy Intl",
        "1302 datum_lat 40.6413",
        "1302 datum_lon -73.7781",
        "1054 119100 KENNEDY TWR",
        "1055 123700 NEW YORK APP",
        "1056 124750 NEW YORK DEP"
    });

    XPAirportReader reader(host, -1, [&](const Airport::Header& header) {
        return XPAtcNavData::queryAirportAirspace(host, header);
    });
    reader.readAirport(aptDat);

    const auto airport = reader.getAirport();
    ASSERT_TRUE(airport->tower());
    ASSERT_TRUE(airport->tower()->airspace());

    const auto airspace = airport->tower()->airspace();
    EXPECT_EQ(airspace->type(), ControlledAirspace::Type::TerminalControlArea);
    EXPECT_EQ(airspace->name(), "NEW YORK");
    EXPECT_EQ(airspace->classification().letter, AirspaceClass::Letter::B);
    EXPECT_TRUE(airspace->geometry()->hasLowerBound());
    EXPECT_TRUE(airspace->geometry()->hasUpperBound());
    EXPECT_FLOAT_EQ(airspace->geometry()->lowerBoundFeet(), 0.0f);
    EXPECT_FLOAT_EQ(airspace->geometry()->upperBoundFeet(), 7000.0f);
}

TEST(XPAirportReaderTest, readAptDat_canonicalizesDecoratedAirspaceTxtNames)
{
    auto host = makeNavdataHost(
        "nav_named_terminal_area",
        {
            "CONTROLLER",
            "NAME METRO",
            "FACILITY_ID KTMA",
            "ROLE twr",
            "CLASS D",
            "ICAO KTMA",
            "FREQ 11830",
            "AIRSPACE_POLYGON_BEGIN 0 3000",
            "POINT 41.9500 -74.0500",
            "POINT 41.9500 -73.9500",
            "POINT 42.0500 -73.9500",
            "POINT 42.0500 -74.0500",
            "AIRSPACE_POLYGON_END",
            "CONTROLLER_END",
            "CONTROLLER",
            "NAME METRO",
            "FACILITY_ID KTMA",
            "ROLE tracon",
            "CLASS C",
            "ICAO KTMA",
            "FREQ 12455",
            "AIRSPACE_POLYGON_BEGIN 0 9000",
            "POINT 41.9000 -74.1000",
            "POINT 41.9000 -73.9000",
            "POINT 42.1000 -73.9000",
            "POINT 42.1000 -74.1000",
            "AIRSPACE_POLYGON_END",
            "CONTROLLER_END",
        },
        {
            "AC A",
            "AN METRO TMA AREA 1",
            "AL 1500 MSL",
            "AH 19500 MSL",
            "DP 41:54:00 N 074:06:00 W",
            "DP 41:54:00 N 073:54:00 W",
            "DP 42:06:00 N 073:54:00 W",
            "DP 42:06:00 N 074:06:00 W",
        });

    stringstream aptDat = makeAptDat({
        "1 13 0 0 KTMA Metro Test Airport",
        "1302 datum_lat 42.0000",
        "1302 datum_lon -74.0000",
        "1054 118300 METRO TWR",
        "1055 124550 METRO APP"
    });

    XPAirportReader reader(host, -1, [&](const Airport::Header& header) {
        return XPAtcNavData::queryAirportAirspace(host, header);
    });
    reader.readAirport(aptDat);

    const auto airport = reader.getAirport();
    ASSERT_TRUE(airport->tower());
    ASSERT_TRUE(airport->tower()->airspace());

    const auto airspace = airport->tower()->airspace();
    EXPECT_EQ(airspace->type(), ControlledAirspace::Type::TerminalControlArea);
    EXPECT_EQ(airspace->name(), "METRO TMA");
    EXPECT_EQ(airspace->classification().letter, AirspaceClass::Letter::A);
    EXPECT_TRUE(airspace->geometry()->hasLowerBound());
    EXPECT_TRUE(airspace->geometry()->hasUpperBound());
    EXPECT_FLOAT_EQ(airspace->geometry()->lowerBoundFeet(), 1500.0f);
    EXPECT_FLOAT_EQ(airspace->geometry()->upperBoundFeet(), 19500.0f);
}

TEST(XPAptDatReaderTest, readAptDat_allAirports)
{
    ifstream input;
    openTestInputStream("apt_many.dat", input);
    XPAptDatReader reader(makeHost());
    vector<shared_ptr<Airport>> output;

    reader.readAptDat(
        input,
        XPAirportReader::noopQueryAirspace,
        XPAirportReader::noopFilterAirport,
        [&](shared_ptr<Airport> airport) {
            output.push_back(airport);
        }
    );

    ASSERT_EQ(output.size(), 4);

    EXPECT_EQ(output[0]->header().icao(), "ABCD");
    EXPECT_EQ(output[0]->getParkingStandOrThrow("A1")->name(), "A1");

    EXPECT_EQ(output[1]->header().icao(), "EFGH");
    EXPECT_EQ(output[1]->getParkingStandOrThrow("B1")->name(), "B1");

    EXPECT_EQ(output[2]->header().icao(), "IJKL");
    EXPECT_EQ(output[2]->getParkingStandOrThrow("C1")->name(), "C1");

    EXPECT_EQ(output[3]->header().icao(), "MNOP");
    EXPECT_EQ(output[3]->getParkingStandOrThrow("D1")->name(), "D1");
}

TEST(XPAptDatReaderTest, readAptDat_filterAirports)
{
    ifstream input;
    openTestInputStream("apt_many.dat", input);
    XPAptDatReader reader(makeHost());
    vector<shared_ptr<Airport>> output;

    reader.readAptDat(
        input,
        XPAirportReader::noopQueryAirspace,
        [&](const Airport::Header& header) {
            return (header.icao() == "EFGH" || header.icao() == "MNOP");
        },
        [&](shared_ptr<Airport> airport) {
            output.push_back(airport);
        }
    );

    ASSERT_EQ(output.size(), 2);

    EXPECT_EQ(output[0]->header().icao(), "EFGH");
    EXPECT_EQ(output[0]->getParkingStandOrThrow("B1")->name(), "B1");

    EXPECT_EQ(output[1]->header().icao(), "MNOP");
    EXPECT_EQ(output[1]->getParkingStandOrThrow("D1")->name(), "D1");
}

TEST(XPAptDatReaderTest, readAptDat_skipAirportsFailingToLoad)
{
    ifstream input;
    openTestInputStream("apt_errors.dat", input);
    XPAptDatReader reader(makeHost());
    vector<shared_ptr<Airport>> output;

    reader.readAptDat(
        input,
        XPAirportReader::noopQueryAirspace,
        [&](const Airport::Header& header) {
            return true;
        },
        [&](shared_ptr<Airport> airport) {
            output.push_back(airport);
        }
    );

    ASSERT_EQ(output.size(), 3);
    EXPECT_EQ(output[0]->header().icao(), "ABCD");
    EXPECT_EQ(output[1]->header().icao(), "IJKL");
    EXPECT_EQ(output[2]->header().icao(), "MNOP");
}

#if 0
TEST(XPAptDatReaderTest, readAll_realDefaultAptDat)
{
    ifstream input;
    input.exceptions(ifstream::failbit | ifstream::badbit);
    input.open(R"(E:\X-Plane 11\Resources\default scenery\default apt dat\Earth nav data\apt.dat)");

    ofstream output;
    output.exceptions(ofstream::failbit | ofstream::badbit);
    output.open("../../src/libdataxp_test/testOutputs/gates-with-long-names.txt", std::ios_base::out | std::ios_base::trunc);

    int count = 0;

    XPAptDatReader reader(makeHost());
    reader.readAptDat(
        input,
        [&](const Airport::Header& header) {
            return WorldBuilder::assembleSampleAirportControlZone(header);
        },
        [&](const Airport::Header& header) {
            return true;
        },
        [&](shared_ptr<Airport> airport) {
            count++;
            if ((count % 100) == 0)
            {
                cout << "done: # " << count << " " << airport->header().icao() << endl;
            }
            for (const auto& gate : airport->parkingStands())
            {
                if (gate->name().length() > 5)
                {
                    output << gate->name() << endl;
                }
            }
        }
    );
}
#endif

shared_ptr<HostServices> makeHost()
{
    return make_shared<TestHostServices>();
}

shared_ptr<ControlledAirspace> makeAirspace(
    double centerLat, 
    double centerLon, 
    float radiusNm, 
    const string& name)
{
    GeoPolygon airspaceBounds({ 
        GeoPolygon::circleEdge(GeoPoint(centerLat, centerLon), radiusNm)
    });
    auto airspaceGeometry = shared_ptr<AirspaceGeometry>(new AirspaceGeometry(airspaceBounds, false, 0, true, 10000));
    auto airspace = shared_ptr<ControlledAirspace>(new ControlledAirspace(
        1, 
        "USA", 
        "TST", 
        name, 
        name, 
        ControlledAirspace::Type::ControlZone, 
        AirspaceClass::ClassB, 
        airspaceGeometry));
    return airspace;
}

stringstream makeAptDat(const vector<string>& lines)
{
    stringstream output;
    output.exceptions(ios::failbit | ios::badbit);

    for (const auto& line : lines)
    {
        output << line << endl;
    }

    output.seekg(0);
    return output;
}

void openTestInputStream(const string& fileName, ifstream& str)
{
    string fullPath = "../../src/libdataxp_test/testInputs/" + fileName;
    str.exceptions(ifstream::failbit | ifstream::badbit);
    str.open(fullPath.c_str());
}

void createTestOutputStream(const string& fileName, ofstream& str)
{
    string fullPath = "../../src/libdataxp_test/testInputs/" + fileName;
    str.exceptions(ofstream::failbit | ofstream::badbit);
    str.open(fullPath.c_str(), ofstream::out | ofstream::trunc);
}

void assertRunwaysExist(shared_ptr<Airport> airport, const vector<string>& names)
{
    for (const string& name : names)
    {
        try
        {
            auto runway = airport->getRunwayOrThrow(name);
            runway->getEndOrThrow(name);
        }
        catch (const exception& e)
        {
            stringstream message;
            message << "assertRunwaysExist FAILED name [" << name << "] error [" << e.what() << "]";
            throw runtime_error(message.str());
        }
    }
}

void assertGatesExist(shared_ptr<Airport> airport, const vector<string>& names)
{
    for (const string& name : names)
    {
        try
        {
            airport->getParkingStandOrThrow(name);
        }
        catch (const exception& e)
        {
            stringstream message;
            message << "assertGatesExist FAILED name [" << name << "] error [" << e.what() << "]";
            throw runtime_error(message.str());
        }
    }
}

void assertTaxiEdgesExist(shared_ptr<Airport> airport, const unordered_set<string>& names)
{
    unordered_set<string> remainingNames = names;

    for (auto edge : airport->taxiNet()->edges())
    {
        if (edge->type() == TaxiEdge::Type::Taxiway)
        {
            remainingNames.erase(edge->name());
        }
    }

    if (!remainingNames.empty())
    {
        stringstream message;
        message << "assertTaxiEdgesExist FAILED missing:";
        for (const auto& name : remainingNames)
        {
            message << " [" << name << "]";
        }
        throw runtime_error(message.str());
    }
}

// void writeAirportJson(shared_ptr<const Airport> airport, ostream& output)
// {
//     console::JsonWriter writer(output);
//     console::JsonProtocol protocol(writer);
//     protocol.writeAirport(airport);
// }

// void writeTaxiPathJson(shared_ptr<const TaxiPath> taxiPath, ostream& output)
// {
//     console::JsonWriter writer(output);
//     console::JsonProtocol protocol(writer);
//     protocol.writeTaxiPath(taxiPath);
// }
