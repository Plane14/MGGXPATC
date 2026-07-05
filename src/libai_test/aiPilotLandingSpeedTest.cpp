// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#include <iomanip>
#include <memory>

#include "gtest/gtest.h"

#define private public
#include "libworld.h"
#include "libworld_test.h"
#include "aiPilot.hpp"
#undef private

#include "maneuverFactory.hpp"

using namespace std;
using namespace world;
using namespace ai;

namespace
{
    class NoopPhraseologyService : public PhraseologyService
    {
    public:
        shared_ptr<Utterance> verbalizeIntent(shared_ptr<Intent>) override
        {
            return make_shared<Utterance>();
        }
    };

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

    shared_ptr<Airport> createMinimalAirport(shared_ptr<HostServices> host, const string& icao)
    {
        Airport::Header header(icao, "Test Airport", GeoPoint(30.0, 40.0), 0);
        auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);
        return WorldBuilder::assembleAirport(host, header, {}, {}, {}, {}, nullptr, airspace);
    }

    shared_ptr<Airport> createArrivalAirport(shared_ptr<HostServices> host, const string& icao)
    {
        Airport::Header header(icao, "Test Arrival", GeoPoint(31.0, 41.0), 0);
        auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);
        auto runway = createRunway(host, { 31.00, 41.00 }, { 31.00, 41.02 }, "09L", "27R");
        return WorldBuilder::assembleAirport(host, header, { runway }, {}, {}, {}, nullptr, airspace);
    }

    shared_ptr<Airport> createControlledArrivalAirport(shared_ptr<HostServices> host, const string& icao)
    {
        Airport::Header header(icao, "Test Arrival", GeoPoint(31.0, 41.0), 0);
        auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);
        vector<ControllerPosition::Structure> positions = {
            { ControllerPosition::Type::Local, 118300, GeoPolygon::empty(), "" }
        };

        auto runway = createRunway(host, { 31.00, 41.00 }, { 31.00, 41.02 }, "09L", "27R");
        auto tower = WorldBuilder::assembleAirportTower(host, header, airspace, positions);
        return WorldBuilder::assembleAirport(host, header, { runway }, {}, {}, {}, tower, airspace);
    }

    GeoPoint lecoRunway03Threshold()
    {
        return GeoPoint(43.291702778, -8.385755556);
    }

    GeoPoint lecoRunway21Threshold()
    {
        return GeoPoint(43.308594444, -8.371891667);
    }

    GeoPoint lecoCo03wFix()
    {
        return GeoPoint(43.286722222, -8.503277778);
    }

    GeoPoint lecoCo401Fix()
    {
        return GeoPoint(43.268000000, -8.436083333);
    }

    GeoPoint lecoAirportDatum()
    {
        return GeoPoint(
            (lecoRunway03Threshold().latitude + lecoRunway21Threshold().latitude) / 2.0,
            (lecoRunway03Threshold().longitude + lecoRunway21Threshold().longitude) / 2.0);
    }
}

TEST(AIPilotLandingSpeedTest, landingLegSpeedUsesTouchdownProfileInsteadOfInflatedApproachSegmentSpeed)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto airport = createMinimalAirport(host, "KAAA");
    auto world = WorldBuilder::assembleSampleWorld(host, { airport });
    host->useWorld(world);

    auto plan = make_shared<FlightPlan>(0, 3600, "KAAA", "KAAA");
    auto aircraft = make_shared<AIAircraft>(host, 901, "A21N", "VLG", "N901", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 901, Flight::RulesType::IFR, "VLG", "901", "VLG 901", plan);
    flight->setAircraft(aircraft);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 901, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    auto& profile = const_cast<AircraftPerformanceProfile&>(aircraft->performanceProfile());
    profile.approachSpeedKt = 250.0f;
    profile.landingTouchdownSpeedKt = 140.0f;

    auto landingLeg = make_shared<FlightPlan::Leg>(
        FlightPlan::LegType::Landing,
        GeoPolygon::empty(),
        "CO401",
        "03",
        549.0f,
        0.0f);

    EXPECT_FLOAT_EQ(
        pilot->procedureLegGroundSpeed(FlightPlan::LegType::Landing, landingLeg),
        145.0f);
}

TEST(AIPilotLandingSpeedTest, shortFinalGuidanceTargetsTouchdownPointBeyondThreshold)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    Airport::Header header("KBBB", "Test Arrival", GeoPoint(31.0, 41.0), 0);
    auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);
    auto runway = createRunway(host, { 31.00, 41.00 }, { 31.00, 41.02 }, "09L", "27R");
    auto airport = WorldBuilder::assembleAirport(host, header, { runway }, {}, {}, {}, nullptr, airspace);
    auto world = WorldBuilder::assembleSampleWorld(host, { airport });
    host->useWorld(world);

    auto plan = make_shared<FlightPlan>(0, 3600, "KBBB", "KBBB");
    plan->setArrivalRunway("09L");

    auto aircraft = make_shared<AIAircraft>(host, 902, "A21N", "VLG", "N902", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 902, Flight::RulesType::IFR, "VLG", "902", "VLG 902", plan);
    flight->setAircraft(aircraft);
    flight->setPhase(Flight::Phase::Arrival);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 902, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    const auto& runwayEnd = runway->getEndOrThrow("09L");
    const GeoPoint threshold = runwayEnd.centerlinePoint().geo();
    const GeoPoint justPastThreshold = GeoMath::getPointAtDistance(threshold, runwayEnd.heading(), 40.0f);

    aircraft->setLocation(justPastThreshold);
    aircraft->setAltitude(Altitude::agl(120.0f));
    aircraft->setGroundSpeedKt(145.0);
    aircraft->setAttitude(AircraftAttitude(runwayEnd.heading(), 0.0f, 0.0f));

    const float guidanceHeading = pilot->landingShortFinalGuidanceHeading(runwayEnd);
    EXPECT_NEAR(guidanceHeading, runwayEnd.heading(), 1.0f);
}

TEST(AIPilotLandingSpeedTest, shortFinalGuidanceKeepsRunwayHeadingAfterPassingTouchdownZone)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    Airport::Header header("KBBB", "Test Arrival", GeoPoint(31.0, 41.0), 0);
    auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);
    auto runway = createRunway(host, { 31.00, 41.00 }, { 31.00, 41.02 }, "09L", "27R");
    auto airport = WorldBuilder::assembleAirport(host, header, { runway }, {}, {}, {}, nullptr, airspace);
    auto world = WorldBuilder::assembleSampleWorld(host, { airport });
    host->useWorld(world);

    auto plan = make_shared<FlightPlan>(0, 3600, "KBBB", "KBBB");
    plan->setArrivalRunway("09L");

    auto aircraft = make_shared<AIAircraft>(host, 903, "A21N", "VLG", "N903", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 903, Flight::RulesType::IFR, "VLG", "903", "VLG 903", plan);
    flight->setAircraft(aircraft);
    flight->setPhase(Flight::Phase::Arrival);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 903, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    const auto& runwayEnd = runway->getEndOrThrow("09L");
    const GeoPoint threshold = runwayEnd.centerlinePoint().geo();
    const GeoPoint afterTouchdownZone = GeoMath::getPointAtDistance(
        threshold,
        runwayEnd.heading(),
        aircraft->landingTouchdownDistanceMetersForRunway(*runway) + 25.0f);

    aircraft->setLocation(afterTouchdownZone);
    aircraft->setAltitude(Altitude::agl(90.0f));
    aircraft->setGroundSpeedKt(140.0);
    aircraft->setAttitude(AircraftAttitude(runwayEnd.heading(), 0.0f, 0.0f));

    const GeoPoint aimPoint = pilot->landingShortFinalAimPoint(runwayEnd);
    EXPECT_TRUE(pilot->landingShortFinalAimPointReached(runwayEnd, aimPoint));
    EXPECT_NEAR(pilot->landingShortFinalGuidanceHeading(runwayEnd), runwayEnd.heading(), 1.0f);
}

TEST(AIPilotLandingSpeedTest, shortFinalCommitRequiresRunwayCorridorAndTouchdownProximity)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    Airport::Header header("KBBB", "Test Arrival", GeoPoint(31.0, 41.0), 0);
    auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);
    auto runway = createRunway(host, { 31.00, 41.00 }, { 31.00, 41.02 }, "09L", "27R");
    auto airport = WorldBuilder::assembleAirport(host, header, { runway }, {}, {}, {}, nullptr, airspace);
    auto world = WorldBuilder::assembleSampleWorld(host, { airport });
    host->useWorld(world);

    auto plan = make_shared<FlightPlan>(0, 3600, "KBBB", "KBBB");
    plan->setArrivalRunway("09L");

    auto aircraft = make_shared<AIAircraft>(host, 904, "A21N", "VLG", "N904", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 904, Flight::RulesType::IFR, "VLG", "904", "VLG 904", plan);
    flight->setAircraft(aircraft);
    flight->setPhase(Flight::Phase::Arrival);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 904, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    const auto& runwayEnd = runway->getEndOrThrow("09L");
    const GeoPoint threshold = runwayEnd.centerlinePoint().geo();
    const float touchdownDistanceMeters = aircraft->landingTouchdownDistanceMetersForRunway(*runway);
    const GeoPoint farBeforeThreshold = GeoMath::getPointAtDistance(
        threshold,
        GeoMath::flipHeading(runwayEnd.heading()),
        320.0f);
    const GeoPoint nearTouchdown = GeoMath::getPointAtDistance(
        threshold,
        runwayEnd.heading(),
        touchdownDistanceMeters - 60.0f);
    const GeoPoint offsetFromTouchdown = GeoMath::getPointAtDistance(
        nearTouchdown,
        GeoMath::addTurnToHeading(runwayEnd.heading(), 90.0f),
        runway->widthMeters());

    aircraft->setAltitude(Altitude::agl(50.0f));
    aircraft->setGroundSpeedKt(145.0);
    aircraft->setAttitude(AircraftAttitude(runwayEnd.heading(), 0.0f, 0.0f));

    aircraft->setLocation(farBeforeThreshold);
    EXPECT_FALSE(pilot->landingShortFinalReadyToCommit(runwayEnd));

    aircraft->setLocation(offsetFromTouchdown);
    EXPECT_FALSE(pilot->landingShortFinalReadyToCommit(runwayEnd));

    aircraft->setLocation(nearTouchdown);
    EXPECT_TRUE(pilot->landingShortFinalReadyToCommit(runwayEnd));
}

TEST(AIPilotLandingSpeedTest, waypointLookupTreatsRunwayAliasesAsEquivalent)
{
    auto plan = make_shared<FlightPlan>(0, 3600, "KAAA", "LECO");
    plan->m_knownWaypoints.push_back(FlightPlan::RouteWaypoint("RW03", "", lecoRunway03Threshold()));

    GeoPoint resolved = GeoPoint::empty;
    EXPECT_TRUE(AIPilot::tryFindWaypointLocation(plan, "03", resolved));
    EXPECT_NEAR(resolved.latitude, lecoRunway03Threshold().latitude, 1e-9);
    EXPECT_NEAR(resolved.longitude, lecoRunway03Threshold().longitude, 1e-9);
}

TEST(AIPilotLandingSpeedTest, landingManeuverFromLecoCo401TouchesDownOnRunway)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    Airport::Header header("LECO", "A Coruna", lecoAirportDatum(), 330);
    auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);
    auto runway = createRunway(host, lecoRunway03Threshold(), lecoRunway21Threshold(), "RW03", "RW21");
    auto airport = WorldBuilder::assembleAirport(host, header, { runway }, {}, {}, {}, nullptr, airspace);
    auto departureAirport = createMinimalAirport(host, "KAAA");
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport, airport });
    host->useWorld(world);
    world->onQueryTerrainElevation([](const GeoPoint&) {
        return 330.0;
    });

    auto plan = make_shared<FlightPlan>(0, 3600, "KAAA", "LECO");
    plan->setArrivalRunway("RW03");
    plan->m_knownWaypoints.push_back(FlightPlan::RouteWaypoint("CO401", "", lecoCo401Fix()));
    plan->m_knownWaypoints.push_back(FlightPlan::RouteWaypoint("RW03", "", lecoRunway03Threshold()));
    plan->m_legs.clear();
    plan->m_legs.push_back(make_shared<FlightPlan::Leg>(
        FlightPlan::LegType::Landing,
        GeoPolygon::empty(),
        "CO401",
        "RW03",
        627.0f,
        0.0f));

    auto aircraft = make_shared<AIAircraft>(host, 905, "A21N", "VLG", "N905", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 905, Flight::RulesType::IFR, "VLG", "905", "VLG 905", plan);
    flight->setAircraft(aircraft);
    flight->setPhase(Flight::Phase::Arrival);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 905, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    if (auto cursor = flight->planCursor())
    {
        cursor->reset();
    }

    auto landingClearance = make_shared<LandingClearance>(
        Clearance::Header{
            1,
            Clearance::Type::LandingClearance,
            world->timestamp(),
            nullptr,
            flight
        },
        "RW03",
        121705);
    flight->addClearance(landingClearance);

    const auto& runwayEnd = runway->getEndOrThrow("RW03");
    const float inboundHeading = GeoMath::getHeadingFromPoints(lecoCo03wFix(), lecoCo401Fix());
    aircraft->setLocation(lecoCo401Fix());
    aircraft->setAltitude(Altitude::msl(1500.0f));
    aircraft->setGroundSpeedKt(145.0);
    aircraft->setVerticalSpeedFpm(-900.0);
    aircraft->setAttitude(AircraftAttitude(inboundHeading, -2.5f, 0.0f));
    aircraft->setTrack(inboundHeading);
    aircraft->setManeuver(pilot->maneuverLanding());

    const auto startTimestamp = world->timestamp();
    bool touchedDown = false;
    GeoPoint touchdownLocation = GeoPoint::empty;

    for (int step = 1; step <= 180; ++step)
    {
        const auto timestamp = startTimestamp + chrono::seconds(step);
        aircraft->progressTo(timestamp);
        world->progressTo(timestamp);

        if (aircraft->altitude().type() == Altitude::Type::Ground)
        {
            touchedDown = true;
            touchdownLocation = aircraft->location();
            break;
        }
    }

    EXPECT_TRUE(touchedDown)
        << fixed << setprecision(9)
        << "final lat=" << aircraft->location().latitude
        << " lon=" << aircraft->location().longitude
        << " altType=" << static_cast<int>(aircraft->altitude().type())
        << " altFt=" << aircraft->altitude().feet()
        << " gsKt=" << aircraft->groundSpeedKt()
        << " vsFpm=" << aircraft->verticalSpeedFpm();
    if (touchedDown)
    {
        const auto alignment = pilot->landingRunwayAlignmentMeters(runwayEnd, touchdownLocation);
        const double alongTrackMeters = get<0>(alignment);
        const double crossTrackMeters = fabs(get<1>(alignment));

        EXPECT_GE(
            alongTrackMeters,
            aircraft->landingTouchdownDistanceMetersForRunway(*runway) - 60.0f);
        EXPECT_LE(alongTrackMeters, runway->lengthMeters() + 60.0f);
        EXPECT_LE(crossTrackMeters, max(12.0, static_cast<double>(runway->widthMeters()) * 0.5))
            << fixed << setprecision(9)
            << "touchdown lat=" << touchdownLocation.latitude
            << " lon=" << touchdownLocation.longitude;
    }
}

TEST(AIPilotLandingSpeedTest, landingAlignmentFromLecoCo401FighterTouchesDownWhenLandingLegUsesBareRunwayAlias)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    Airport::Header header("LECO", "A Coruna", lecoAirportDatum(), 330);
    auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);
    auto runway = createRunway(host, lecoRunway03Threshold(), lecoRunway21Threshold(), "RW03", "RW21");
    auto gateLocation = UniPoint::fromGeo(host, GeoPoint(43.3008, -8.3793));
    auto gate = shared_ptr<ParkingStand>(new ParkingStand(
        1,
        "G1",
        ParkingStand::Type::Gate,
        gateLocation,
        90.0f,
        "C"));
    auto airport = WorldBuilder::assembleAirport(host, header, { runway }, { gate }, {}, {}, nullptr, airspace);
    auto departureAirport = createMinimalAirport(host, "KAAA");
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport, airport });
    host->useWorld(world);
    world->onQueryTerrainElevation([](const GeoPoint&) {
        return 330.0;
    });

    auto plan = make_shared<FlightPlan>(0, 3600, "KAAA", "LECO");
    plan->setArrivalRunway("RW03");
    plan->setArrivalGate("G1");
    plan->m_knownWaypoints.push_back(FlightPlan::RouteWaypoint("CO401", "", lecoCo401Fix()));
    plan->m_knownWaypoints.push_back(FlightPlan::RouteWaypoint("RW03", "", lecoRunway03Threshold()));
    plan->m_legs.clear();
    plan->m_legs.push_back(make_shared<FlightPlan::Leg>(
        FlightPlan::LegType::Landing,
        GeoPolygon::empty(),
        "CO401",
        "03",
        627.0f,
        0.0f));

    auto aircraft = make_shared<AIAircraft>(host, 906, "F35", "BUSMC", "N906", Aircraft::Category::Fighter);
    auto flight = make_shared<Flight>(host, 906, Flight::RulesType::IFR, "BUSMC", "906", "BUSMC 906", plan);
    flight->setAircraft(aircraft);
    flight->setPhase(Flight::Phase::Arrival);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 906, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    if (auto cursor = flight->planCursor())
    {
        cursor->reset();
    }

    auto landingClearance = make_shared<LandingClearance>(
        Clearance::Header{
            1,
            Clearance::Type::LandingClearance,
            world->timestamp(),
            nullptr,
            flight
        },
        "RW03",
        121705);
    flight->addClearance(landingClearance);

    const auto& runwayEnd = runway->getEndOrThrow("RW03");
    const float inboundHeading = GeoMath::getHeadingFromPoints(lecoCo03wFix(), lecoCo401Fix());
    aircraft->setLocation(lecoCo401Fix());
    aircraft->setAltitude(Altitude::msl(1500.0f));
    aircraft->setGroundSpeedKt(155.0);
    aircraft->setVerticalSpeedFpm(-900.0);
    aircraft->setAttitude(AircraftAttitude(inboundHeading, -2.5f, 0.0f));
    aircraft->setTrack(inboundHeading);
    aircraft->setManeuver(maneuverFactory->sequence(Maneuver::Type::ArrivalApproach, "landing_alias_path", {
        pilot->maneuverProcedureLeg(FlightPlan::LegType::Landing, Flight::Phase::Arrival, "landing_alignment_leg"),
        pilot->maneuverLanding()
    }));

    const auto startTimestamp = world->timestamp();
    bool touchedDown = false;
    GeoPoint touchdownLocation = GeoPoint::empty;

    for (int step = 1; step <= 240; ++step)
    {
        const auto timestamp = startTimestamp + chrono::seconds(step);
        aircraft->progressTo(timestamp);
        world->progressTo(timestamp);

        if (aircraft->altitude().type() == Altitude::Type::Ground)
        {
            touchedDown = true;
            touchdownLocation = aircraft->location();
            break;
        }
    }

    EXPECT_TRUE(touchedDown)
        << fixed << setprecision(9)
        << "final lat=" << aircraft->location().latitude
        << " lon=" << aircraft->location().longitude
        << " altType=" << static_cast<int>(aircraft->altitude().type())
        << " altFt=" << aircraft->altitude().feet()
        << " gsKt=" << aircraft->groundSpeedKt()
        << " vsFpm=" << aircraft->verticalSpeedFpm()
        << " hdg=" << aircraft->attitude().heading();
    if (touchedDown)
    {
        const auto alignment = pilot->landingRunwayAlignmentMeters(runwayEnd, touchdownLocation);
        const double alongTrackMeters = get<0>(alignment);
        const double crossTrackMeters = fabs(get<1>(alignment));

        EXPECT_GE(
            alongTrackMeters,
            aircraft->landingTouchdownDistanceMetersForRunway(*runway) - 60.0f);
        EXPECT_LE(alongTrackMeters, runway->lengthMeters() + 60.0f);
        EXPECT_LE(crossTrackMeters, max(12.0, static_cast<double>(runway->widthMeters()) * 0.5))
            << fixed << setprecision(9)
            << "touchdown lat=" << touchdownLocation.latitude
            << " lon=" << touchdownLocation.longitude;
    }
}

TEST(AIPilotLandingSpeedTest, goAroundTargetGroundSpeedUsesRotorFriendlyProfileForHelicopter)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto airport = createMinimalAirport(host, "KHEL");
    auto world = WorldBuilder::assembleSampleWorld(host, { airport });
    host->useWorld(world);

    auto plan = make_shared<FlightPlan>(0, 3600, "KHEL", "KHEL");
    auto aircraft = make_shared<AIAircraft>(host, 904, "AS32", "HEL", "N904", Aircraft::Category::Helicopter);
    auto flight = make_shared<Flight>(host, 904, Flight::RulesType::VFR, "HEL", "904", "HEL 904", plan);
    flight->setAircraft(aircraft);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 904, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    auto& profile = const_cast<AircraftPerformanceProfile&>(aircraft->performanceProfile());
    profile.approachSpeedKt = 95.0f;
    profile.takeoffInitialClimbSpeedKt = 72.0f;
    profile.initialClimbRocFpm = 1400.0f;

    EXPECT_FLOAT_EQ(pilot->goAroundTargetGroundSpeedKt(), 72.0f);
    EXPECT_FLOAT_EQ(pilot->goAroundTargetVerticalSpeedFpm(), 800.0f);
    EXPECT_FLOAT_EQ(pilot->goAroundTargetFlapState(), 0.0f);
}

TEST(AIPilotLandingSpeedTest, autoGoAroundTriggerPrioritizesOvershootBeforeLateLandingClearance)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto arrivalAirport = createArrivalAirport(host, "KBBB");
    auto world = WorldBuilder::assembleSampleWorld(host, { arrivalAirport });
    host->useWorld(world);

    auto plan = make_shared<FlightPlan>(0, 3600, "KBBB", "KBBB");
    plan->setArrivalRunway("09L");
    auto aircraft = make_shared<AIAircraft>(host, 905, "A320", "DAL", "N905", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 905, Flight::RulesType::IFR, "DAL", "905", "DAL 905", plan);
    flight->setAircraft(aircraft);
    flight->setPhase(Flight::Phase::Arrival);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 905, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    const auto& runwayEnd = arrivalAirport->getRunwayOrThrow("09L")->getEndOrThrow("09L");
    world->progressTo(chrono::seconds(200));
    pilot->m_finalReportedTimestamp = world->timestamp() - chrono::seconds(90);

    aircraft->setLocation(GeoMath::getPointAtDistance(
        runwayEnd.centerlinePoint().geo(),
        runwayEnd.heading(),
        250.0f));
    aircraft->setAttitude(AircraftAttitude(runwayEnd.heading(), 0.0f, 0.0f));
    aircraft->setAltitude(Altitude::agl(180.0f));
    aircraft->setGroundSpeedKt(145.0);

    EXPECT_EQ(
        pilot->autoGoAroundTriggerOnFinal(),
        AIPilot::AutoGoAroundTrigger::RunwayOvershoot);
}

TEST(AIPilotLandingSpeedTest, autoGoAroundTriggerUsesLateLandingClearanceWhenStillInbound)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto arrivalAirport = createArrivalAirport(host, "KBBB");
    auto world = WorldBuilder::assembleSampleWorld(host, { arrivalAirport });
    host->useWorld(world);

    auto plan = make_shared<FlightPlan>(0, 3600, "KBBB", "KBBB");
    plan->setArrivalRunway("09L");
    auto aircraft = make_shared<AIAircraft>(host, 906, "A320", "DAL", "N906", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 906, Flight::RulesType::IFR, "DAL", "906", "DAL 906", plan);
    flight->setAircraft(aircraft);
    flight->setPhase(Flight::Phase::Arrival);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 906, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    const auto& runwayEnd = arrivalAirport->getRunwayOrThrow("09L")->getEndOrThrow("09L");
    world->progressTo(chrono::seconds(200));
    pilot->m_finalReportedTimestamp = world->timestamp() - chrono::seconds(90);

    aircraft->setLocation(GeoMath::getPointAtDistance(
        runwayEnd.centerlinePoint().geo(),
        GeoMath::flipHeading(runwayEnd.heading()),
        2.0f * METERS_IN_1_NAUTICAL_MILE));
    aircraft->setAttitude(AircraftAttitude(runwayEnd.heading(), 0.0f, 0.0f));
    aircraft->setAltitude(Altitude::agl(900.0f));
    aircraft->setGroundSpeedKt(145.0);

    EXPECT_EQ(
        pilot->autoGoAroundTriggerOnFinal(),
        AIPilot::AutoGoAroundTrigger::LateLandingClearance);
}

TEST(AIPilotLandingSpeedTest, approachClearanceTransmissionIsStoredAndReadBack)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);
    host->services().use<PhraseologyService>(make_shared<NoopPhraseologyService>());

    auto arrivalAirport = createControlledArrivalAirport(host, "KBBB");
    auto world = WorldBuilder::assembleSampleWorld(host, { arrivalAirport });
    host->useWorld(world);

    auto plan = make_shared<FlightPlan>(0, 3600, "KBBB", "KBBB");
    plan->setArrivalRunway("09L");
    auto aircraft = make_shared<AIAircraft>(host, 907, "C208", "CGL", "N907", Aircraft::Category::Prop);
    auto flight = make_shared<Flight>(host, 907, Flight::RulesType::IFR, "CGL", "907", "CGL 907", plan);
    flight->setAircraft(aircraft);
    flight->setPhase(Flight::Phase::Arrival);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 907, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    auto towerPosition = arrivalAirport->tower()->findPositionOrThrow(
        ControllerPosition::Type::Local,
        arrivalAirport->header().datum());
    ASSERT_TRUE(!!towerPosition);
    aircraft->setFrequency(towerPosition->frequency());

    auto clearance = make_shared<ApproachClearance>(
        Clearance::Header{
            1,
            Clearance::Type::ApproachClearance,
            world->timestamp(),
            towerPosition,
            flight
        },
        "09L",
        ApproachClearance::ApproachType::ILS,
        2500.0f,
        false,
        towerPosition->frequency()->khz());

    auto approachIntent = make_shared<TowerClearedForApproachIntent>(
        2,
        0,
        towerPosition,
        flight,
        true,
        clearance,
        vector<TrafficAdvisory>{});

    towerPosition->frequency()->enqueueTransmission(approachIntent);
    world->progressTo(chrono::seconds(1));
    world->progressTo(chrono::seconds(2));

    auto storedClearance = flight->tryFindClearance<ApproachClearance>(Clearance::Type::ApproachClearance);
    ASSERT_TRUE(!!storedClearance);
    EXPECT_EQ(storedClearance->runway(), "09L");

    const auto& transmissions = host->textToSpeechService()->transmissionHistory();
    const auto readbackCount = count_if(
        transmissions.begin(),
        transmissions.end(),
        [](const shared_ptr<Transmission>& transmission) {
            return transmission && transmission->intent() &&
                transmission->intent()->code() == PilotApproachClearanceReadbackIntent::IntentCode;
        });

    EXPECT_EQ(readbackCount, 1);
}

TEST(AIPilotLandingSpeedTest, arrivalRadarCheckInFallsBackToTowerWhenApproachUnavailable)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);
    host->services().use<PhraseologyService>(make_shared<NoopPhraseologyService>());

    auto arrivalAirport = createControlledArrivalAirport(host, "KBBB");
    auto world = WorldBuilder::assembleSampleWorld(host, { arrivalAirport });
    host->useWorld(world);

    auto plan = make_shared<FlightPlan>(0, 3600, "KBBB", "KBBB");
    plan->setArrivalRunway("09L");
    auto aircraft = make_shared<AIAircraft>(host, 908, "C208", "CGL", "N908", Aircraft::Category::Prop);
    auto flight = make_shared<Flight>(host, 908, Flight::RulesType::IFR, "CGL", "908", "CGL 908", plan);
    flight->setAircraft(aircraft);
    flight->setPhase(Flight::Phase::Arrival);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 908, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    const auto towerPosition = arrivalAirport->tower()->findPositionOrThrow(
        ControllerPosition::Type::Local,
        arrivalAirport->header().datum());
    ASSERT_TRUE(!!towerPosition);

    aircraft->setLocation(arrivalAirport->header().datum());
    aircraft->setAltitude(Altitude::agl(1500.0f));
    aircraft->setGroundSpeedKt(110.0);
    aircraft->setVerticalSpeedFpm(-600.0);

    auto arrivalRadarCheckIn = pilot->maneuverArrivalRadarCheckIn();
    ASSERT_TRUE(!!arrivalRadarCheckIn);

    arrivalRadarCheckIn->progressTo(chrono::seconds(1));
    world->progressTo(chrono::seconds(1));
    arrivalRadarCheckIn->progressTo(chrono::seconds(2));
    world->progressTo(chrono::seconds(2));

    ASSERT_TRUE(!!aircraft->frequency());
    EXPECT_EQ(aircraft->frequency()->khz(), towerPosition->frequency()->khz());
    EXPECT_TRUE(pilot->m_arrivalRadarCheckInDone);
    EXPECT_TRUE(pilot->m_arrivalTowerRadarCheckInDone);

    const auto& transmissions = host->textToSpeechService()->transmissionHistory();
    const auto radarCheckInCount = count_if(
        transmissions.begin(),
        transmissions.end(),
        [](const shared_ptr<Transmission>& transmission) {
            return transmission && transmission->intent() &&
                transmission->intent()->code() == PilotCheckInWithRadarIntent::IntentCode;
        });

    EXPECT_EQ(radarCheckInCount, 1);
}

TEST(AIPilotLandingSpeedTest, landingManeuverIncludesLandingCommitGateBeforeTouchdown)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto arrivalAirport = createArrivalAirport(host, "KBBB");
    auto world = WorldBuilder::assembleSampleWorld(host, { arrivalAirport });
    host->useWorld(world);

    auto plan = make_shared<FlightPlan>(0, 3600, "KBBB", "KBBB");
    plan->setArrivalRunway("09L");
    auto aircraft = make_shared<AIAircraft>(host, 909, "C750", "CGL", "N909", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 909, Flight::RulesType::IFR, "CGL", "909", "CGL 909", plan);
    flight->setAircraft(aircraft);
    flight->setPhase(Flight::Phase::Arrival);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 909, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    const auto& runwayEnd = arrivalAirport->getRunwayOrThrow("09L")->getEndOrThrow("09L");
    aircraft->setOnFinal(runwayEnd);
    aircraft->setLocation(GeoMath::getPointAtDistance(
        runwayEnd.centerlinePoint().geo(),
        GeoMath::flipHeading(runwayEnd.heading()),
        0.8f * METERS_IN_1_NAUTICAL_MILE));
    aircraft->setAttitude(AircraftAttitude(runwayEnd.heading(), 0.0f, 0.0f));
    aircraft->setAltitude(Altitude::agl(220.0f));
    aircraft->setGroundSpeedKt(135.0);
    aircraft->setVerticalSpeedFpm(-700.0);

    auto landing = pilot->maneuverLanding();
    ASSERT_TRUE(!!landing);

    auto firstChild = landing->firstChild();
    ASSERT_TRUE(!!firstChild);
    EXPECT_EQ(firstChild->id(), "landing_leg");

    auto secondChild = firstChild->nextSibling();
    ASSERT_TRUE(!!secondChild);
    EXPECT_EQ(secondChild->id(), "short_final");

    auto thirdChild = secondChild->nextSibling();
    ASSERT_TRUE(!!thirdChild);
    EXPECT_EQ(thirdChild->id(), "await_landing_commit");

    auto fourthChild = thirdChild->nextSibling();
    ASSERT_TRUE(!!fourthChild);
    EXPECT_TRUE(fourthChild->isProxy());
}

TEST(AIPilotLandingSpeedTest, lateLandingClearanceTimeoutScalesDownOnShortFinal)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto arrivalAirport = createArrivalAirport(host, "KBBB");
    auto world = WorldBuilder::assembleSampleWorld(host, { arrivalAirport });
    host->useWorld(world);

    auto plan = make_shared<FlightPlan>(0, 3600, "KBBB", "KBBB");
    plan->setArrivalRunway("09L");
    auto aircraft = make_shared<AIAircraft>(host, 910, "C750", "CGL", "N910", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 910, Flight::RulesType::IFR, "CGL", "910", "CGL 910", plan);
    flight->setAircraft(aircraft);
    flight->setPhase(Flight::Phase::Arrival);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 910, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    const auto& runwayEnd = arrivalAirport->getRunwayOrThrow("09L")->getEndOrThrow("09L");
    aircraft->setLocation(GeoMath::getPointAtDistance(
        runwayEnd.centerlinePoint().geo(),
        GeoMath::flipHeading(runwayEnd.heading()),
        0.7f * METERS_IN_1_NAUTICAL_MILE));
    aircraft->setAttitude(AircraftAttitude(runwayEnd.heading(), 0.0f, 0.0f));
    aircraft->setAltitude(Altitude::agl(180.0f));
    aircraft->setGroundSpeedKt(135.0);

    world->progressTo(chrono::seconds(200));
    pilot->m_finalReportedTimestamp = world->timestamp() - chrono::seconds(18);

    EXPECT_TRUE(pilot->shouldAutoGoAroundForLateLandingClearance());
}

TEST(AIPilotLandingSpeedTest, goAroundReadbackSkipsTransmissionWhenNoArrivalControllerExists)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto airport = createMinimalAirport(host, "KAAA");
    auto world = WorldBuilder::assembleSampleWorld(host, { airport });
    host->useWorld(world);

    auto plan = make_shared<FlightPlan>(0, 3600, "KAAA", "KAAA");
    auto aircraft = make_shared<AIAircraft>(host, 907, "A320", "DAL", "N907", Aircraft::Category::Jet);
    auto flight = make_shared<Flight>(host, 907, Flight::RulesType::IFR, "DAL", "907", "DAL 907", plan);
    flight->setAircraft(aircraft);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 907, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    auto request = make_shared<GoAroundRequest>(
        Clearance::Header{ 1, Clearance::Type::GoAroundRequest, world->timestamp(), nullptr, flight },
        "09L",
        DeclineReason::RunwayNotVacated);

    auto readback = pilot->maneuverGoAroundReadback(request);
    ASSERT_TRUE(!!readback);

    readback->progressTo(chrono::seconds(1));
    world->progressTo(chrono::seconds(1));

    EXPECT_TRUE(host->textToSpeechService()->transmissionHistory().empty());
}

TEST(AIPilotLandingSpeedTest, uncontrolledArrivalDoesNotAutoGoAroundForMissingLandingClearance)
{
    auto host = TestHostServices::create();
    auto intentFactory = make_shared<IntentFactory>(host);
    auto maneuverFactory = make_shared<ManeuverFactory>(host);
    host->services().use<IntentFactory>(intentFactory);

    auto departureAirport = createMinimalAirport(host, "KAAA");
    auto arrivalAirport = createArrivalAirport(host, "KBBB");
    auto world = WorldBuilder::assembleSampleWorld(host, { departureAirport, arrivalAirport });
    host->useWorld(world);

    auto plan = make_shared<FlightPlan>(0, 3600, "KAAA", "KBBB");
    plan->setArrivalRunway("09L");
    auto aircraft = make_shared<AIAircraft>(host, 908, "SR22", "SR2", "N908", Aircraft::Category::LightProp);
    auto flight = make_shared<Flight>(host, 908, Flight::RulesType::VFR, "SR2", "908", "SR2 908", plan);
    flight->setAircraft(aircraft);
    flight->setPhase(Flight::Phase::Arrival);
    world->addFlight(flight);

    auto pilot = make_shared<AIPilot>(host, 908, Actor::Gender::Male, flight, maneuverFactory, intentFactory);
    flight->setPilot(pilot);

    const auto& runwayEnd = arrivalAirport->getRunwayOrThrow("09L")->getEndOrThrow("09L");
    aircraft->setLocation(GeoMath::getPointAtDistance(
        runwayEnd.centerlinePoint().geo(),
        GeoMath::flipHeading(runwayEnd.heading()),
        1.5f * METERS_IN_1_NAUTICAL_MILE));
    aircraft->setAttitude(AircraftAttitude(runwayEnd.heading(), 0.0f, 0.0f));
    aircraft->setAltitude(Altitude::agl(700.0f));
    aircraft->setGroundSpeedKt(95.0);
    aircraft->setVerticalSpeedFpm(-500.0);
    flight->setArrivalRunway("09L");

    world->progressTo(chrono::seconds(200));
    pilot->m_finalReportedTimestamp = world->timestamp() - chrono::seconds(90);

    EXPECT_TRUE(pilot->shouldAutoGoAroundForLateLandingClearance());

    pilot->markUncontrolledArrivalLanding("test");
    pilot->ensureAutoGoAroundOnFinal(
        "runway_overshoot_before_reply",
        "weather_on_final",
        "landing_clearance_timeout_before_reply");

    EXPECT_TRUE(pilot->m_uncontrolledArrivalLanding);
    EXPECT_EQ(pilot->m_finalReportedTimestamp, chrono::microseconds(0));
    EXPECT_EQ(pilot->autoGoAroundTriggerOnFinal(), AIPilot::AutoGoAroundTrigger::None);
    EXPECT_FALSE(flight->tryFindClearance<GoAroundRequest>(Clearance::Type::GoAroundRequest));
}