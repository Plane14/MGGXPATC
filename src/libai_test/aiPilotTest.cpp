// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#include <memory>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

#include "gtest/gtest.h"
#include "libworld.h"
#include "libworld_test.h"
#include "aiPilot.hpp"
#include "maneuverFactory.hpp"

using namespace std;
using namespace world;
using namespace ai;

namespace
{
    shared_ptr<Airport> createMinimalDepartureAirport(shared_ptr<HostServices> host)
    {
        Airport::Header header("KAAA", "Test Departure", GeoPoint(30.0, 40.0), 0);
        auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);

        vector<ControllerPosition::Structure> positions = {
            { ControllerPosition::Type::Ground, 121900, GeoPolygon::empty(), "" }
        };

        auto tower = WorldBuilder::assembleAirportTower(host, header, airspace, positions);
        return WorldBuilder::assembleAirport(host, header, {}, {}, {}, {}, tower, airspace);
    }

    shared_ptr<Runway> createRunway(
        shared_ptr<HostServices> host,
        const GeoPoint& p1,
        const GeoPoint& p2,
        const string& name1,
        const string& name2,
        float widthMeters = 50,
        float displacedThresholdMeters = 0)
    {
        Runway::End end1(name1, displacedThresholdMeters, 0.0f, UniPoint::fromGeo(host, p1));
        Runway::End end2(name2, displacedThresholdMeters, 10.0f, UniPoint::fromGeo(host, p2));
        return shared_ptr<Runway>(new Runway(end1, end2, widthMeters));
    }

    GeoPoint lecoRunway03Threshold()
    {
        return GeoPoint(43.291702778, -8.385755556);
    }

    GeoPoint lecoRunway21Threshold()
    {
        return GeoPoint(43.308594444, -8.371891667);
    }

    GeoPoint lecoAirportDatum()
    {
        return GeoPoint(
            (lecoRunway03Threshold().latitude + lecoRunway21Threshold().latitude) / 2.0,
            (lecoRunway03Threshold().longitude + lecoRunway21Threshold().longitude) / 2.0);
    }

    GeoPoint lecoWaypointFromThreshold(float distanceNm)
    {
        return GeoMath::getPointAtDistance(lecoRunway03Threshold(), 292.0f, distanceNm * 1852.0f);
    }

    GeoPoint lecoMissedApproachWaypointFromThreshold(float distanceNm)
    {
        return GeoMath::getPointAtDistance(lecoRunway03Threshold(), 112.0f, distanceNm * 1852.0f);
    }

    class LecoArrivalTestHostServices : public TestHostServices
    {
    public:
        shared_ptr<istream> openFileForRead(const string& filePath) override
        {
            if (filePath.find("LECO.dat") != string::npos)
            {
                return make_shared<stringstream>(sampleLecoCifp());
            }

            if (filePath.find("earth_fix.dat") != string::npos)
            {
                return make_shared<stringstream>(sampleEarthFix());
            }

            if (filePath.find("earth_nav.dat") != string::npos)
            {
                return make_shared<stringstream>(string());
            }

            return TestHostServices::openFileForRead(filePath);
        }

    private:
        static string sampleLecoCifp()
        {
            return R"(STAR:010,2,LOMD4A,ALL,LOMDA,LE,E,A,E   , ,   ,IF, , , , , ,      ,    ,    ,    ,    ,+,FL120,     ,     , ,   ,    ,   , , , , , , , , ;
STAR:020,2,LOMD4A,ALL,GALZO,LE,P,C,E  H, ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   , , , , , , , , ;
STAR:030,2,LOMD4A,ALL,LRA,LE,D, ,VE H, ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   , , , , , , , , ;
APPCH:010,A,R03,IDOTU,IDOTU,LE,P,C,E  C, ,010,IF, , , , , ,      ,    ,    ,    ,    , ,05000,     ,     , ,   ,    ,   , , , , , ,B,J,S;
APPCH:020,A,R03,IDOTU,CO05W,LE,P,C,EE B, ,010,TF, , , , , ,      ,    ,    ,    ,    ,+,02500,     ,     , ,   ,    ,   , , , , , ,B,J,S;
APPCH:010,R,R03, ,CO05W,LE,P,C,E  I, ,010,IF, , , , , ,      ,    ,    ,    ,    ,+,02500,     ,     , ,   ,    ,   , , , , , ,B,J,S;
APPCH:020,R,R03, ,CO03W,LE,P,C,E  F, ,010,TF, , , , , ,      ,    ,    ,1120,0020,+,02500,     ,     , ,   ,    ,   ,LECO,LE,P,A, ,B,J,S;
PRDAT: ,          , ,          ,A,      LNAV, ,   , ,   , ,   , ,   ,J,S;
APPCH:030,R,R03, ,CO401,LE,P,C,EY M, ,302,TF, , , , , ,      ,    ,    ,1120,0032, ,01500,     ,     ,-,160,-300,   , , , , , ,B,J,S;
APPCH:040,R,R03, ,CO402,LE,P,C,E M ,L,010,DF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     ,-,160,    ,   , , , , , ,B,J,S;
APPCH:050,R,R03, ,CO403,LE,E,A,E   , ,010,TF, , , , , ,      ,    ,    ,    ,    ,+,04000,     ,     ,-,200,    ,   , , , , , ,B,J,S;
APPCH:060,R,R03, ,IDOTU,LE,P,C,E   , ,010,TF, , , , , ,      ,    ,    ,    ,    , ,05000,     ,     ,-,200,    ,   , , , , , ,B,J,S;
APPCH:070,R,R03, ,IDOTU,LE,P,C,EE H,R,   ,HM, , , , , ,      ,    ,    ,1120,T010,+,05000,     ,     , ,   ,    ,   , , , , , ,B,J,S;
RWY:RW03 ,     ,      ,00329, ,    , ,   ;N43173013,W008230872,0000;
RWY:RW21 ,     ,      ,00272, ,LCO ,2,   ;N43183094,W008221881,0000;
)";
        }

        static string sampleEarthFix()
        {
            ostringstream output;
            output << "I\n";
            output << "1200 Version - data cycle 2604\n";

            const auto addFix = [&](const string& name, const GeoPoint& location) {
                output << fixed << setprecision(9)
                       << location.latitude << ' ' << location.longitude << ' ' << name << '\n';
            };

            addFix("LOMDA", lecoWaypointFromThreshold(20.0f));
            addFix("GALZO", lecoWaypointFromThreshold(15.0f));
            addFix("BERAX", lecoWaypointFromThreshold(14.0f));
            addFix("LRA", lecoWaypointFromThreshold(12.0f));
            addFix("IDOTU", lecoWaypointFromThreshold(10.0f));
            addFix("CO05W", lecoWaypointFromThreshold(5.0f));
            addFix("CO03W", lecoWaypointFromThreshold(3.2f));
            addFix("CO401", lecoWaypointFromThreshold(1.5f));
            addFix("CO402", lecoMissedApproachWaypointFromThreshold(2.0f));
            addFix("CO403", lecoMissedApproachWaypointFromThreshold(4.0f));

            return output.str();
        }
    };

    shared_ptr<Airport> createLecoArrivalAirport(shared_ptr<HostServices> host)
    {
        Airport::Header header("LECO", "A Coruna", lecoAirportDatum(), 330);
        auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);

        vector<ControllerPosition::Structure> positions = {
            { ControllerPosition::Type::Local, 121705, GeoPolygon::empty(), "" }
        };

        auto tower = WorldBuilder::assembleAirportTower(host, header, airspace, positions);
        auto runway = createRunway(host, lecoRunway03Threshold(), lecoRunway21Threshold(), "RW03", "RW21");

        auto gateLocation = UniPoint::fromGeo(host, GeoPoint(43.3008, -8.3793));
        auto gate = shared_ptr<ParkingStand>(new ParkingStand(
            1,
            "G1",
            ParkingStand::Type::Gate,
            gateLocation,
            90.0f,
            "C"));

        return WorldBuilder::assembleAirport(host, header, { runway }, { gate }, {}, {}, tower, airspace);
    }

    shared_ptr<Airport> createMinimalArrivalAirport(shared_ptr<HostServices> host)
    {
        Airport::Header header("KBBB", "Test Arrival", GeoPoint(31.0, 41.0), 0);
        auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);

        vector<ControllerPosition::Structure> positions = {
            { ControllerPosition::Type::Local, 118300, GeoPolygon::empty(), "" }
        };

        auto runway = createRunway(host, { 31.00, 41.00 }, { 31.00, 41.02 }, "09L", "27R");
        auto tower = WorldBuilder::assembleAirportTower(host, header, airspace, positions);
        return WorldBuilder::assembleAirport(host, header, { runway }, {}, {}, {}, tower, airspace);
    }

    shared_ptr<Airport> createHelicopterAirport(shared_ptr<HostServices> host, const string& icao, const GeoPoint& datum)
    {
        Airport::Header header(icao, icao + " Heliport", datum, 0);
        auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);

        vector<ControllerPosition::Structure> positions = {
            { ControllerPosition::Type::Local, 118300, GeoPolygon::empty(), "" }
        };

        auto tower = WorldBuilder::assembleAirportTower(host, header, airspace, positions);
        auto stand = shared_ptr<ParkingStand>(new ParkingStand(
            1,
            "H1",
            ParkingStand::Type::Remote,
            UniPoint::fromGeo(host, datum),
            45.0f,
            "A",
            Aircraft::Category::Helicopter));

        return WorldBuilder::assembleAirport(host, header, {}, { stand }, {}, {}, tower, airspace);
    }

    class ProcedureAwareTestHostServices : public TestHostServices
    {
    public:
        string getHostFilePath(const vector<string>& relativePathParts) override
        {
            return relativePathParts.empty() ? string() : relativePathParts.back();
        }

        shared_ptr<istream> openFileForRead(const string& filePath) override
        {
            if (filePath == "KBBB.dat")
            {
                return make_shared<stringstream>(sampleCifp());
            }

            if (filePath == "earth_fix.dat")
            {
                return make_shared<stringstream>(sampleEarthFix());
            }

            return make_shared<stringstream>(string());
        }

    private:
        static string sampleCifp()
        {
            return
                "STAR:010,4,STAR1,RW09L,STARFIX;\n"
                "STAR:020,4,STAR1,RW09L,APPFIX;\n"
                "APPCH:010,A,I09L,APPFIX,APPFIX;\n"
                "APPCH:020,A,I09L,APPFIX,09L;\n";
        }

        static string sampleEarthFix()
        {
            return
                "I\n"
                "1200 Version - data cycle 2604\n"
                " 31.000000000   40.900000000  STARFIX ENRT KB 0000000 STARFIX\n"
                " 31.000000000   40.950000000  APPFIX ENRT KB 0000000 APPFIX\n";
        }
    };

    class BridgeProcedureTestHostServices : public TestHostServices
    {
    public:
        string getHostFilePath(const vector<string>& relativePathParts) override
        {
            return relativePathParts.empty() ? string() : relativePathParts.back();
        }

        shared_ptr<istream> openFileForRead(const string& filePath) override
        {
            if (filePath == "KBBB.dat")
            {
                return make_shared<stringstream>(sampleCifp());
            }

            if (filePath == "earth_fix.dat")
            {
                return make_shared<stringstream>(sampleEarthFix());
            }

            return make_shared<stringstream>(string());
        }

    private:
        static string sampleCifp()
        {
            return
                "STAR:010,4,STAR1,RW09L,GALZO;\n"
                "STAR:020,4,STAR1,RW09L,BERAX;\n"
                "APPCH:010,A,I09L,IDOTU,CO403;\n"
                "APPCH:020,A,I09L,CO403,09L;\n";
        }

        static string sampleEarthFix()
        {
            return
                "I\n"
                "1200 Version - data cycle 2604\n"
                " 31.000000000   40.880000000  GALZO ENRT KB 0000000 GALZO\n"
                " 31.000000000   40.900000000  BERAX ENRT KB 0000000 BERAX\n"
                " 31.000000000   40.920000000  IDOTU ENRT KB 0000000 IDOTU\n"
                " 31.000000000   40.940000000  CO403 ENRT KB 0000000 CO403\n";
        }
    };

    class ZeroRandomTestHostServices : public TestHostServices
    {
    public:
        int getNextRandom(int maxValue) override
        {
            return 0;
        }
    };

    bool containsManeuverId(const shared_ptr<Maneuver>& maneuver, const string& expectedId)
    {
        if (!maneuver)
        {
            return false;
        }

        if (maneuver->id() == expectedId)
        {
            return true;
        }

        for (auto child = maneuver->firstChild(); child; child = child->nextSibling())
        {
            if (containsManeuverId(child, expectedId))
            {
                return true;
            }
        }

        return false;
    }
}

TEST(AIPilotTest, flightCycleIncludesFinalApproachHandoff)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto departureAirport = createMinimalDepartureAirport(host);
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport });
    host->useWorld(world);

    const time_t departureTime = world->currentTime() + 3600;
    const time_t arrivalTime = departureTime + 7200;

    auto plan = make_shared<FlightPlan>(departureTime, arrivalTime, "KAAA", "KBBB");
    auto aircraft = make_shared<AIAircraft>(host, 101, "B738", "DAL", "N101", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 101, Flight::RulesType::IFR, "DAL", "101", "DAL 101", plan);
    flight->setAircraft(aircraft);

    auto pilot = make_shared<AIPilot>(host, 1, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    auto cycle = pilot->getFlightCycle();
    ASSERT_TRUE(!!cycle);

    shared_ptr<Maneuver> lastChild;
    size_t childCount = 0;
    for (auto child = cycle->firstChild(); child; child = child->nextSibling())
    {
        lastChild = child;
        ++childCount;
    }

    EXPECT_EQ(childCount, 11u);
    ASSERT_TRUE(!!lastChild);
    EXPECT_EQ(lastChild->id(), "enter_final");
    EXPECT_EQ(lastChild->type(), Maneuver::Type::Unspecified);
}

TEST(AIPilotTest, finalToGateStartsWithFinalAndLanding)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto departureAirport = createMinimalDepartureAirport(host);
    auto arrivalAirport = createMinimalArrivalAirport(host);
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport, arrivalAirport });
    host->useWorld(world);

    const time_t departureTime = world->currentTime() + 3600;
    const time_t arrivalTime = departureTime + 7200;

    auto plan = make_shared<FlightPlan>(departureTime, arrivalTime, "KAAA", "KBBB");
    plan->setArrivalRunway("09L");
    auto aircraft = make_shared<AIAircraft>(host, 102, "B738", "DAL", "N102", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 102, Flight::RulesType::IFR, "DAL", "102", "DAL 102", plan);
    flight->setAircraft(aircraft);

    auto pilot = make_shared<AIPilot>(host, 2, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    auto landingRunwayEnd = arrivalAirport->getRunwayOrThrow("09L")->getEndOrThrow("09L");
    auto finalToGate = pilot->getFinalToGate(landingRunwayEnd);
    ASSERT_TRUE(!!finalToGate);

    auto firstChild = finalToGate->firstChild();
    ASSERT_TRUE(!!firstChild);
    EXPECT_EQ(firstChild->id(), "final");

    auto secondChild = firstChild->nextSibling();
    ASSERT_TRUE(!!secondChild);
    EXPECT_TRUE(secondChild->isProxy());
    EXPECT_TRUE(secondChild->id().empty());
}

TEST(AIPilotTest, finalToGateIncludesStarAndApproachWhenAvailable)
{
    auto host = make_shared<ProcedureAwareTestHostServices>();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto departureAirport = createMinimalDepartureAirport(host);
    auto arrivalAirport = createMinimalArrivalAirport(host);
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport, arrivalAirport });
    host->useWorld(world);

    const time_t departureTime = world->currentTime() + 3600;
    const time_t arrivalTime = departureTime + 7200;

    auto plan = make_shared<FlightPlan>(departureTime, arrivalTime, "KAAA", "KBBB");
    plan->setStar("STAR1");
    plan->setApproach("I09L");
    plan->setArrivalRunway("09L");
    auto aircraft = make_shared<AIAircraft>(host, 103, "B738", "DAL", "N103", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 103, Flight::RulesType::IFR, "DAL", "103", "DAL 103", plan);
    flight->setAircraft(aircraft);

    auto pilot = make_shared<AIPilot>(host, 3, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    ASSERT_GE(flight->plan()->legs().size(), 3u);
    EXPECT_TRUE(any_of(flight->plan()->legs().begin(), flight->plan()->legs().end(), [](const shared_ptr<FlightPlan::Leg>& leg) {
        return leg && leg->type() == FlightPlan::LegType::Star;
    }));
    EXPECT_TRUE(any_of(flight->plan()->legs().begin(), flight->plan()->legs().end(), [](const shared_ptr<FlightPlan::Leg>& leg) {
        return leg && leg->type() == FlightPlan::LegType::Approach;
    }));

    auto landingRunwayEnd = arrivalAirport->getRunwayOrThrow("09L")->getEndOrThrow("09L");
    auto finalToGate = pilot->getFinalToGate(landingRunwayEnd);
    ASSERT_TRUE(!!finalToGate);

    auto firstChild = finalToGate->firstChild();
    ASSERT_TRUE(!!firstChild);
    EXPECT_EQ(firstChild->id(), "star_leg");

    auto secondChild = firstChild->nextSibling();
    ASSERT_TRUE(!!secondChild);
    EXPECT_EQ(secondChild->id(), "approach_leg");

    auto thirdChild = secondChild->nextSibling();
    ASSERT_TRUE(!!thirdChild);
    EXPECT_EQ(thirdChild->id(), "final");

    auto fourthChild = thirdChild->nextSibling();
    ASSERT_TRUE(!!fourthChild);
    EXPECT_TRUE(fourthChild->isProxy());
    EXPECT_TRUE(fourthChild->id().empty());
}

TEST(AIPilotTest, fighterFlightCycleIncludesUnrestrictedClimboutWhenHostRandomAllowsIt)
{
    auto host = make_shared<ZeroRandomTestHostServices>();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto departureAirport = createMinimalDepartureAirport(host);
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport });
    host->useWorld(world);

    const time_t departureTime = world->currentTime() + 3600;
    const time_t arrivalTime = departureTime + 7200;

    auto plan = make_shared<FlightPlan>(departureTime, arrivalTime, "KAAA", "KBBB");
    auto aircraft = make_shared<AIAircraft>(host, 104, "F22", "USA", "F104", Aircraft::Category::Fighter);
    auto flight = make_shared<Flight>(host, 104, Flight::RulesType::IFR, "USA", "104", "USA 104", plan);
    flight->setAircraft(aircraft);

    auto pilot = make_shared<AIPilot>(host, 4, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    auto cycle = pilot->getFlightCycle();
    ASSERT_TRUE(!!cycle);
    EXPECT_TRUE(containsManeuverId(cycle, "reset_cycle_state"));
    EXPECT_TRUE(containsManeuverId(cycle, "unrestricted_climbout"));
}

TEST(AIPilotTest, fighterWingmanFlightCycleSkipsOwnIfrAndUnrestrictedClimbout)
{
    auto host = make_shared<ZeroRandomTestHostServices>();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto departureAirport = createMinimalDepartureAirport(host);
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport });
    host->useWorld(world);

    const time_t departureTime = world->currentTime() + 3600;
    const time_t arrivalTime = departureTime + 7200;

    auto leaderAircraft = make_shared<AIAircraft>(host, 204, "F22", "USA", "F204", Aircraft::Category::Fighter);
    auto aircraft = make_shared<AIAircraft>(host, 205, "F22", "USA", "F205", Aircraft::Category::Fighter);
    aircraft->setFormationLeader(leaderAircraft, 0.35, -0.18, -40.0);

    auto plan = make_shared<FlightPlan>(departureTime, arrivalTime, "KAAA", "KBBB");
    auto flight = make_shared<Flight>(host, 205, Flight::RulesType::IFR, "USA", "205", "USA 205", plan);
    flight->setAircraft(aircraft);

    auto pilot = make_shared<AIPilot>(host, 5, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    auto cycle = pilot->getFlightCycle();
    ASSERT_TRUE(!!cycle);
    EXPECT_TRUE(containsManeuverId(cycle, "wait_for_formation_leader_airborne"));
    EXPECT_FALSE(containsManeuverId(cycle, "await_ifr_clr"));
    EXPECT_FALSE(containsManeuverId(cycle, "unrestricted_climbout"));
}

TEST(AIPilotTest, helicopterFlightCycleUsesRunwayFreeHelipadRoutingWhenNoRunwaysAreAssigned)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto departureAirport = createHelicopterAirport(host, "KHEL", GeoPoint(30.0, 40.0));
    auto arrivalAirport = createHelicopterAirport(host, "KHED", GeoPoint(30.6, 40.6));
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport, arrivalAirport });
    host->useWorld(world);

    const time_t departureTime = world->currentTime() + 3600;
    const time_t arrivalTime = departureTime + 1800;

    auto plan = make_shared<FlightPlan>(departureTime, arrivalTime, "KHEL", "KHED");
    plan->setDepartureGate("H1");
    plan->setArrivalGate("H1");
    auto aircraft = make_shared<AIAircraft>(host, 150, "AS32", "HEL", "N150", Aircraft::Category::Helicopter);
    auto flight = make_shared<Flight>(host, 150, Flight::RulesType::VFR, "HEL", "150", "HEL 150", plan);
    flight->setAircraft(aircraft);

    auto pilot = make_shared<AIPilot>(host, 15, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    auto cycle = pilot->getFlightCycle();
    ASSERT_TRUE(!!cycle);
    EXPECT_TRUE(containsManeuverId(cycle, "heli_departure"));
    EXPECT_TRUE(containsManeuverId(cycle, "enter_heli_final"));
    EXPECT_FALSE(containsManeuverId(cycle, "enter_final"));

    auto heliArrival = pilot->getHelipadFinalToGate(arrivalAirport->getParkingStandOrThrow("H1"));
    ASSERT_TRUE(!!heliArrival);
    EXPECT_TRUE(containsManeuverId(heliArrival, "heli_park"));
}

TEST(AIPilotTest, finalToGateIncludesBridgeLegBetweenStarAndApproachWhenNeeded)
{
    auto host = make_shared<BridgeProcedureTestHostServices>();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto departureAirport = createMinimalDepartureAirport(host);
    auto arrivalAirport = createMinimalArrivalAirport(host);
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport, arrivalAirport });
    host->useWorld(world);

    const time_t departureTime = world->currentTime() + 3600;
    const time_t arrivalTime = departureTime + 7200;

    auto plan = make_shared<FlightPlan>(departureTime, arrivalTime, "KAAA", "KBBB");
    plan->setStar("STAR1");
    plan->setApproach("I09L");
    plan->setArrivalRunway("09L");
    auto aircraft = make_shared<AIAircraft>(host, 104, "B738", "DAL", "N104", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 104, Flight::RulesType::IFR, "DAL", "104", "DAL 104", plan);
    flight->setAircraft(aircraft);

    auto pilot = make_shared<AIPilot>(host, 4, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    auto landingRunwayEnd = arrivalAirport->getRunwayOrThrow("09L")->getEndOrThrow("09L");
    auto finalToGate = pilot->getFinalToGate(landingRunwayEnd);
    ASSERT_TRUE(!!finalToGate);

    auto firstChild = finalToGate->firstChild();
    ASSERT_TRUE(!!firstChild);
    EXPECT_EQ(firstChild->id(), "star_leg");

    auto secondChild = firstChild->nextSibling();
    ASSERT_TRUE(!!secondChild);
    EXPECT_EQ(secondChild->id(), "arrival_bridge_leg");

    auto thirdChild = secondChild->nextSibling();
    ASSERT_TRUE(!!thirdChild);
    EXPECT_EQ(thirdChild->id(), "approach_leg");

    auto fourthChild = thirdChild->nextSibling();
    ASSERT_TRUE(!!fourthChild);
    EXPECT_EQ(fourthChild->id(), "final");

    auto fifthChild = fourthChild->nextSibling();
    ASSERT_TRUE(!!fifthChild);
    EXPECT_TRUE(fifthChild->isProxy());
    EXPECT_TRUE(fifthChild->id().empty());
}

TEST(AIPilotTest, arrivalAircraftCannotTouchDownBeforePassingThreshold)
{
    auto host = TestHostServices::create();
    auto departureAirport = createMinimalDepartureAirport(host);
    auto arrivalAirport = createMinimalArrivalAirport(host);
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport, arrivalAirport });
    host->useWorld(world);

    const time_t departureTime = world->currentTime() + 3600;
    const time_t arrivalTime = departureTime + 7200;

    auto plan = make_shared<FlightPlan>(departureTime, arrivalTime, "KAAA", "KBBB");
    plan->setArrivalRunway("09L");
    auto aircraft = make_shared<AIAircraft>(host, 151, "A320", "DAL", "N151", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 151, Flight::RulesType::IFR, "DAL", "151", "DAL 151", plan);
    flight->setAircraft(aircraft);
    flight->setPhase(Flight::Phase::Arrival);
    world->addFlight(flight);

    auto runway = arrivalAirport->getRunwayOrThrow("09L");
    const auto& runwayEnd = runway->getEndOrThrow("09L");
    const GeoPoint beforeThreshold = GeoMath::getPointAtDistance(
        runwayEnd.centerlinePoint().geo(),
        GeoMath::flipHeading(runwayEnd.heading()),
        120.0f);
    const GeoPoint afterTouchdownZone = GeoMath::getPointAtDistance(
        runwayEnd.centerlinePoint().geo(),
        runwayEnd.heading(),
        aircraft->landingTouchdownDistanceMetersForRunway(*runway) + 15.0f);

    aircraft->setLocation(beforeThreshold);
    aircraft->setAttitude(AircraftAttitude(runwayEnd.heading(), 0.0f, 0.0f));
    aircraft->setAltitude(Altitude::agl(2.0f));
    aircraft->setGroundSpeedKt(0.0f);
    aircraft->setVerticalSpeedFpm(-600.0f);
    aircraft->progressTo(world->timestamp() + chrono::seconds(1));
    EXPECT_NE(aircraft->altitude().type(), Altitude::Type::Ground);

    aircraft->setLocation(afterTouchdownZone);
    aircraft->setAltitude(Altitude::agl(2.0f));
    aircraft->setGroundSpeedKt(0.0f);
    aircraft->setVerticalSpeedFpm(-600.0f);
    aircraft->progressTo(world->timestamp() + chrono::seconds(2));
    EXPECT_EQ(aircraft->altitude().type(), Altitude::Type::Ground);
}

TEST(AIPilotTest, lecoArrivalFlowReachesIdotuAndStaysTerrainSafe)
{
    auto host = make_shared<LecoArrivalTestHostServices>();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto departureAirport = createMinimalDepartureAirport(host);
    auto arrivalAirport = createLecoArrivalAirport(host);
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport, arrivalAirport });
    host->useWorld(world);

    const auto landingRunwayEnd = arrivalAirport->getRunwayOrThrow("RW03")->getEndOrThrow("RW03");
    const GeoPoint runwayThreshold = landingRunwayEnd.centerlinePoint().geo();

    world->onQueryTerrainElevation([runwayThreshold](const GeoPoint& location) {
        const float distanceNm = GeoMath::getDistanceMeters(location, runwayThreshold) / METERS_IN_1_NAUTICAL_MILE;

        if (distanceNm > 4.0f)
        {
            return 900.0f;
        }

        if (distanceNm > 1.5f)
        {
            return 1100.0f;
        }

        return 330.0f;
    });

    const time_t departureTime = world->currentTime() + 3600;
    const time_t arrivalTime = departureTime + 7200;

    auto plan = make_shared<FlightPlan>(departureTime, arrivalTime, "KAAA", "LECO");
    plan->setStar("LOMD4A");
    plan->setApproach("R03");
    plan->setArrivalRunway("RW03");
    plan->setArrivalGate("G1");

    auto aircraft = make_shared<AIAircraft>(host, 201, "B738", "DAL", "N201", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 201, Flight::RulesType::IFR, "DAL", "201", "DAL 201", plan);
    flight->setAircraft(aircraft);

    auto pilot = make_shared<AIPilot>(host, 6, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);
    world->addFlight(flight);

    const auto& knownWaypoints = flight->plan()->knownWaypoints();
    EXPECT_TRUE(any_of(knownWaypoints.begin(), knownWaypoints.end(), [](const FlightPlan::RouteWaypoint& waypoint) {
        return waypoint.name == "IDOTU";
    }));
    EXPECT_TRUE(any_of(knownWaypoints.begin(), knownWaypoints.end(), [](const FlightPlan::RouteWaypoint& waypoint) {
        return waypoint.name == "CO05W";
    }));
    EXPECT_TRUE(any_of(knownWaypoints.begin(), knownWaypoints.end(), [](const FlightPlan::RouteWaypoint& waypoint) {
        return waypoint.name == "CO03W";
    }));
    EXPECT_TRUE(any_of(knownWaypoints.begin(), knownWaypoints.end(), [](const FlightPlan::RouteWaypoint& waypoint) {
        return waypoint.name == "CO401";
    }));
    EXPECT_TRUE(any_of(knownWaypoints.begin(), knownWaypoints.end(), [](const FlightPlan::RouteWaypoint& waypoint) {
        return waypoint.name == "CO402";
    }));
    EXPECT_TRUE(any_of(knownWaypoints.begin(), knownWaypoints.end(), [](const FlightPlan::RouteWaypoint& waypoint) {
        return waypoint.name == "CO403";
    }));

    const auto runwayWaypointIt = find_if(
        knownWaypoints.begin(),
        knownWaypoints.end(),
        [](const FlightPlan::RouteWaypoint& waypoint) {
            return waypoint.name == "RW03" || waypoint.name == "03";
        });
    ASSERT_NE(runwayWaypointIt, knownWaypoints.end());
    EXPECT_NE(runwayWaypointIt->location, GeoPoint::empty);

    auto landingClearance = shared_ptr<LandingClearance>(new LandingClearance(
        { 1, Clearance::Type::LandingClearance, world->timestamp(), nullptr, flight },
        "RW03",
        121705));
    flight->addClearance(landingClearance);

    FlightPlan::Cursor arrivalCursor(flight->plan());
    shared_ptr<FlightPlan::Leg> arrivalStartLeg;
    if (arrivalCursor.activateNextLegOfType(FlightPlan::LegType::Star) && arrivalCursor.activeLeg())
    {
        arrivalStartLeg = arrivalCursor.activeLeg();
    }
    else if (arrivalCursor.activateNextLegOfType(FlightPlan::LegType::Approach) && arrivalCursor.activeLeg())
    {
        arrivalStartLeg = arrivalCursor.activeLeg();
    }

    ASSERT_TRUE(!!arrivalStartLeg);

    const auto startWaypointIt = find_if(
        knownWaypoints.begin(),
        knownWaypoints.end(),
        [&](const FlightPlan::RouteWaypoint& waypoint) {
            return waypoint.name == arrivalStartLeg->fromNavaid();
        });
    ASSERT_NE(startWaypointIt, knownWaypoints.end()) << arrivalStartLeg->fromNavaid();

    aircraft->setOnFinal(landingRunwayEnd);
    const double initialDistanceToRunwayNm = GeoMath::getDistanceMeters(
        aircraft->location(),
        runwayThreshold) / METERS_IN_1_NAUTICAL_MILE;
    double minimumDistanceToRunwayNm = initialDistanceToRunwayNm;

    const auto& legs = flight->plan()->legs();
    shared_ptr<FlightPlan::Leg> bridgeLeg;
    shared_ptr<FlightPlan::Leg> firstApproachLeg;

    for (const auto& leg : legs)
    {
        if (!leg)
        {
            continue;
        }

        if (leg->type() == FlightPlan::LegType::Star ||
            leg->type() == FlightPlan::LegType::EnRoute ||
            leg->type() == FlightPlan::LegType::Approach ||
            leg->type() == FlightPlan::LegType::Landing)
        {
            if (!bridgeLeg && leg->type() == FlightPlan::LegType::EnRoute)
            {
                bridgeLeg = leg;
            }

            if (!firstApproachLeg && leg->type() == FlightPlan::LegType::Approach)
            {
                firstApproachLeg = leg;
            }
        }
    }

    ASSERT_TRUE(!!bridgeLeg);
    EXPECT_EQ(bridgeLeg->toNavaid(), "IDOTU");

    ASSERT_TRUE(!!firstApproachLeg);
    EXPECT_EQ(firstApproachLeg->fromNavaid(), "IDOTU");

    const auto startTimestamp = world->timestamp();

    for (int step = 1; step <= 24; ++step)
    {
        const auto timestamp = startTimestamp + chrono::seconds(step * 15);
        aircraft->progressTo(timestamp);

        const float terrainFeet = world->queryTerrainElevationAt(aircraft->location());
        double currentAltitudeFeet = aircraft->altitude().feet();

        if (aircraft->altitude().type() == Altitude::Type::AGL)
        {
            currentAltitudeFeet += terrainFeet;
        }
        else if (aircraft->altitude().type() == Altitude::Type::Ground)
        {
            currentAltitudeFeet = terrainFeet;
        }

        EXPECT_GE(currentAltitudeFeet + 1.0, terrainFeet);

        const double distanceToRunwayNm = GeoMath::getDistanceMeters(
            aircraft->location(),
            runwayThreshold) / METERS_IN_1_NAUTICAL_MILE;
        minimumDistanceToRunwayNm = min(minimumDistanceToRunwayNm, distanceToRunwayNm);
    }

    EXPECT_GE(aircraft->altitude().feet(), 0.0);
    EXPECT_LT(minimumDistanceToRunwayNm, initialDistanceToRunwayNm);
}

TEST(AIPilotTest, setOnFinalStartsOnInboundStarLegHeadingTowardAirport)
{
    auto host = make_shared<ProcedureAwareTestHostServices>();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto departureAirport = createMinimalDepartureAirport(host);
    auto arrivalAirport = createMinimalArrivalAirport(host);
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport, arrivalAirport });
    host->useWorld(world);

    const time_t departureTime = world->currentTime() + 3600;
    const time_t arrivalTime = departureTime + 7200;

    auto plan = make_shared<FlightPlan>(departureTime, arrivalTime, "KAAA", "KBBB");
    plan->setStar("STAR1");
    plan->setApproach("I09L");
    plan->setArrivalRunway("09L");
    auto aircraft = make_shared<AIAircraft>(host, 105, "B738", "DAL", "N105", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 105, Flight::RulesType::IFR, "DAL", "105", "DAL 105", plan);
    flight->setAircraft(aircraft);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 4, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    auto landingRunwayEnd = arrivalAirport->getRunwayOrThrow("09L")->getEndOrThrow("09L");
    aircraft->setOnFinal(landingRunwayEnd);

    EXPECT_NEAR(aircraft->location().latitude, 31.0, 1e-9);
    EXPECT_NEAR(aircraft->location().longitude, 40.9, 1e-9);

    const auto expectedHeading = GeoMath::getHeadingFromPoints(
        GeoPoint(31.0, 40.9),
        GeoPoint(31.0, 40.95));
    EXPECT_NEAR(aircraft->attitude().heading(), expectedHeading, 1e-6);
}

TEST(AIPilotTest, setOnFinalMovesLaterArrivalBehindExistingTraffic)
{
    auto host = make_shared<ProcedureAwareTestHostServices>();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto departureAirport = createMinimalDepartureAirport(host);
    auto arrivalAirport = createMinimalArrivalAirport(host);
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport, arrivalAirport });
    host->useWorld(world);

    auto makeArrival = [&](int id) {
        auto plan = make_shared<FlightPlan>(0, 3600, "KAAA", "KBBB");
        plan->setStar("STAR1");
        plan->setApproach("I09L");
        plan->setArrivalRunway("09L");

        auto aircraft = make_shared<AIAircraft>(host, id, "B738", "DAL", "N" + to_string(id), Aircraft::Category::Jet);
        auto flight = make_shared<Flight>(host, id, Flight::RulesType::IFR, "DAL", to_string(id), "DAL " + to_string(id), plan);
        flight->setAircraft(aircraft);
        world->addFlight(flight);

        auto pilot = make_shared<AIPilot>(host, id, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
        flight->setPilot(pilot);

        return pair<shared_ptr<Flight>, shared_ptr<AIAircraft>>(flight, aircraft);
    };

    auto first = makeArrival(201);
    auto second = makeArrival(202);
    auto landingRunwayEnd = arrivalAirport->getRunwayOrThrow("09L")->getEndOrThrow("09L");

    first.second->setOnFinal(landingRunwayEnd);
    second.second->setOnFinal(landingRunwayEnd);

    const double separationNm = GeoMath::getDistanceMeters(
        first.second->location(),
        second.second->location()) / METERS_IN_1_NAUTICAL_MILE;

    EXPECT_GT(separationNm, 2.5);
    EXPECT_LT(second.second->location().longitude, first.second->location().longitude);
}

TEST(AIPilotTest, approachProcedureSpeedUsesAircraftPerformanceProfile)
{
    auto host = make_shared<ProcedureAwareTestHostServices>();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto departureAirport = createMinimalDepartureAirport(host);
    auto arrivalAirport = createMinimalArrivalAirport(host);
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport, arrivalAirport });
    host->useWorld(world);

    const auto makeFlight = [&](int id, const string& model, Aircraft::Category category) {
        auto plan = make_shared<FlightPlan>(0, 3600, "KAAA", "KBBB");
        plan->setApproach("I09L");
        plan->setArrivalRunway("09L");

        auto aircraft = make_shared<AIAircraft>(host, id, model, "DAL", "N" + to_string(id), category);
        auto flight = make_shared<Flight>(host, id, Flight::RulesType::IFR, "DAL", to_string(id), "DAL " + to_string(id), plan);
        flight->setAircraft(aircraft);
        world->addFlight(flight);

        auto pilot = make_shared<AIPilot>(host, id, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
        flight->setPilot(pilot);

        auto finalToGate = pilot->getFinalToGate(arrivalAirport->getRunwayOrThrow("09L")->getEndOrThrow("09L"));
        finalToGate->progressTo(chrono::seconds(1));

        return aircraft;
    };

    auto lightAircraft = makeFlight(301, "C172", Aircraft::Category::LightProp);
    auto jetAircraft = makeFlight(302, "B738", Aircraft::Category::Jet);

    EXPECT_LT(lightAircraft->groundSpeedKt(), jetAircraft->groundSpeedKt());
    EXPECT_LT(lightAircraft->groundSpeedKt(), 160.0);
    EXPECT_GT(jetAircraft->groundSpeedKt(), 180.0);
}

TEST(AIPilotTest, arrivalDescentClampsToTerrainBeforePenetratingIt)
{
    auto host = TestHostServices::create();
    auto departureAirport = createMinimalDepartureAirport(host);
    auto arrivalAirport = createMinimalArrivalAirport(host);
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport, arrivalAirport });
    host->useWorld(world);
    world->onQueryTerrainElevation([](const GeoPoint&) {
        return 5000.0;
    });

    auto plan = make_shared<FlightPlan>(0, 3600, "KAAA", "KBBB");
    plan->setArrivalRunway("09L");
    auto aircraft = make_shared<AIAircraft>(host, 104, "B738", "DAL", "N104", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 104, Flight::RulesType::IFR, "DAL", "104", "DAL 104", plan);
    flight->setAircraft(aircraft);
    world->addFlight(flight);
    flight->setPhase(Flight::Phase::Arrival);

    auto runway = arrivalAirport->getRunwayOrThrow("09L");
    const auto& runwayEnd = runway->getEndOrThrow("09L");
    aircraft->setLocation(GeoMath::getPointAtDistance(
        runwayEnd.centerlinePoint().geo(),
        runwayEnd.heading(),
        aircraft->landingTouchdownDistanceMetersForRunway(*runway) + 10.0f));
    aircraft->setAltitude(Altitude::msl(4200.0f));
    aircraft->setVerticalSpeedFpm(-5000.0);

    aircraft->progressTo(chrono::seconds(10));

    EXPECT_EQ(aircraft->altitude().type(), Altitude::Type::Ground);
}

TEST(AIPilotTest, departureClimbLeavesGroundBasedAltitudeDuringSIDTransition)
{
    auto host = TestHostServices::create();
    auto departureAirport = createMinimalDepartureAirport(host);
    auto arrivalAirport = createMinimalArrivalAirport(host);
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport, arrivalAirport });
    host->useWorld(world);

    auto plan = make_shared<FlightPlan>(0, 3600, "KAAA", "KBBB");
    plan->setArrivalRunway("09L");
    auto aircraft = make_shared<AIAircraft>(host, 106, "B738", "DAL", "N106", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 106, Flight::RulesType::IFR, "DAL", "106", "DAL 106", plan);
    flight->setAircraft(aircraft);
    world->addFlight(flight);
    flight->setPhase(Flight::Phase::Departure);

    aircraft->setLocation({ 30.0, 40.0 });
    aircraft->setAltitude(Altitude::ground());
    aircraft->setGroundSpeedKt(0.0);
    const float terrainElevationFeet = static_cast<float>(world->queryTerrainElevationAt(aircraft->location()));
    aircraft->setVerticalSpeedFpm(1800.0);

    aircraft->progressTo(chrono::seconds(10));

    EXPECT_EQ(aircraft->altitude().type(), Altitude::Type::MSL);
    EXPECT_FALSE(aircraft->altitude().isGroundBased());
    EXPECT_GT(aircraft->altitude().feet(), terrainElevationFeet);
}
