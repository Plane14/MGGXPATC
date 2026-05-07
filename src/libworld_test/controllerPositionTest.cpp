// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#include <memory>

#include "gtest/gtest.h"
#include "libworld.h"
#include "libworld_test.h"
#include "intentTypes.hpp"

using namespace std;
using namespace world;

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
        const string& name2)
    {
        Runway::End end1(name1, 0.0f, 0.0f, UniPoint::fromGeo(host, p1));
        Runway::End end2(name2, 0.0f, 10.0f, UniPoint::fromGeo(host, p2));
        return shared_ptr<Runway>(new Runway(end1, end2, 50.0f));
    }

    shared_ptr<Airport> createAirport(
        shared_ptr<HostServices> host,
        const string& icao,
        const GeoPoint& datum,
        int groundKhz,
        int localKhz,
        const string& runwayName1,
        const string& runwayName2)
    {
        Airport::Header header(icao, icao + " Test", datum, 0);
        auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);

        vector<ControllerPosition::Structure> positions = {
            { ControllerPosition::Type::Ground, groundKhz, GeoPolygon::empty(), "" },
            { ControllerPosition::Type::Local, localKhz, GeoPolygon::empty(), "" }
        };

        auto runway = createRunway(host, datum, { datum.latitude, datum.longitude + 0.02 }, runwayName1, runwayName2);
        auto tower = WorldBuilder::assembleAirportTower(host, header, airspace, positions);
        return WorldBuilder::assembleAirport(host, header, { runway }, {}, {}, {}, tower, airspace);
    }
}

TEST(ControllerPositionTest, receivesOnlyAddressedControllerIntents)
{
    auto host = TestHostServices::create();
    host->services().use<PhraseologyService>(make_shared<NoopPhraseologyService>());
    auto airport = createAirport(host, "KAAA", { 30.0, 40.0 }, 121900, 118300, "03", "21");
    auto world = WorldBuilder::assembleSampleWorld(host, { airport });
    host->useWorld(world);

    shared_ptr<TestHostServices::TestAIController> localController;
    for (const auto& controller : host->getCreatedAIControllers())
    {
        if (controller &&
            controller->position() &&
            controller->position()->facility() &&
            controller->position()->facility()->airport() &&
            controller->position()->facility()->airport()->header().icao() == "KAAA" &&
            controller->position()->type() == ControllerPosition::Type::Local)
        {
            localController = controller;
            break;
        }
    }

    ASSERT_TRUE(localController);

    int receiveCount = 0;
    localController->onReceiveIntent([&receiveCount](shared_ptr<Intent>) {
        receiveCount++;
    });

    auto testFlight = host->addIfrFlight(701, "KAAA", "KAAA", GeoPoint(30.0, 40.0), Altitude::agl(2000));
    ASSERT_TRUE(testFlight.ptr);

    auto localPosition = localController->position();
    ASSERT_TRUE(localPosition);

    auto outboundTowerIntent = shared_ptr<Intent>(new TowerContinueApproachIntent(
        1,
        0,
        localPosition,
        testFlight.ptr,
        "03",
        1,
        {}));

    localPosition->frequency()->enqueueTransmission(outboundTowerIntent);
    localPosition->frequency()->progressTo(chrono::microseconds(1));
    localPosition->frequency()->progressTo(chrono::microseconds(2));

    EXPECT_EQ(receiveCount, 0);

    auto inboundPilotIntent = shared_ptr<Intent>(new PilotReportFinalIntent(
        2,
        testFlight.ptr,
        localPosition,
        "03"));

    localPosition->frequency()->enqueueTransmission(inboundPilotIntent);
    localPosition->frequency()->progressTo(chrono::microseconds(3));
    localPosition->frequency()->progressTo(chrono::microseconds(4));

    EXPECT_EQ(receiveCount, 1);
}

TEST(ControllerPositionTest, appliesDefaultControllerScopeBoundariesByPositionType)
{
    auto host = TestHostServices::create();
    Airport::Header header("KAAA", "Scoped Test", GeoPoint(30.0, 40.0), 0);
    auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);
    auto tower = WorldBuilder::assembleAirportTower(host, header, airspace, {
        { ControllerPosition::Type::Ground, 121900, GeoPolygon::empty(), "" },
        { ControllerPosition::Type::Local, 118300, GeoPolygon::empty(), "" },
        { ControllerPosition::Type::Approach, 124700, GeoPolygon::empty(), "" }
    });

    const GeoPoint nearAirport = header.datum();
    const GeoPoint localOnlyPoint = GeoMath::getPointAtDistance(header.datum(), 90.0f, 6.0f * 1852.0f);
    const GeoPoint approachOnlyPoint = GeoMath::getPointAtDistance(header.datum(), 90.0f, 20.0f * 1852.0f);
    const GeoPoint outsideApproach = GeoMath::getPointAtDistance(header.datum(), 90.0f, 55.0f * 1852.0f);

    ASSERT_TRUE(tower->tryFindPosition(ControllerPosition::Type::Ground, nearAirport));
    EXPECT_EQ(tower->tryFindPosition(ControllerPosition::Type::Ground, nearAirport)->type(), ControllerPosition::Type::Ground);
    EXPECT_EQ(tower->tryFindPosition(ControllerPosition::Type::Ground, localOnlyPoint), nullptr);

    ASSERT_TRUE(tower->tryFindPosition(ControllerPosition::Type::Local, localOnlyPoint));
    EXPECT_EQ(tower->tryFindPosition(ControllerPosition::Type::Local, localOnlyPoint)->type(), ControllerPosition::Type::Local);
    EXPECT_EQ(tower->tryFindPosition(ControllerPosition::Type::Local, approachOnlyPoint), nullptr);

    ASSERT_TRUE(tower->tryFindPosition(ControllerPosition::Type::Approach, approachOnlyPoint));
    EXPECT_EQ(tower->tryFindPosition(ControllerPosition::Type::Approach, approachOnlyPoint)->type(), ControllerPosition::Type::Approach);
    EXPECT_EQ(tower->tryFindPosition(ControllerPosition::Type::Approach, outsideApproach), nullptr);
}

TEST(ControllerPositionTest, clearanceDeliveryFallbackRespectsScopedFallbackPositions)
{
    auto host = TestHostServices::create();
    Airport::Header header("KAAA", "Fallback Scope Test", GeoPoint(30.0, 40.0), 0);
    auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);
    auto tower = WorldBuilder::assembleAirportTower(host, header, airspace, {
        { ControllerPosition::Type::Ground, 121900, GeoPolygon::empty(), "" },
        { ControllerPosition::Type::Local, 118300, GeoPolygon::empty(), "" }
    });

    const GeoPoint groundScopePoint = GeoMath::getPointAtDistance(header.datum(), 90.0f, 2.0f * 1852.0f);
    const GeoPoint localScopePoint = GeoMath::getPointAtDistance(header.datum(), 90.0f, 7.0f * 1852.0f);
    const GeoPoint outsideLocalScope = GeoMath::getPointAtDistance(header.datum(), 90.0f, 25.0f * 1852.0f);

    ASSERT_TRUE(tower->tryFindPosition(ControllerPosition::Type::ClearanceDelivery, groundScopePoint));
    EXPECT_EQ(
        tower->tryFindPosition(ControllerPosition::Type::ClearanceDelivery, groundScopePoint)->type(),
        ControllerPosition::Type::Ground);

    ASSERT_TRUE(tower->tryFindPosition(ControllerPosition::Type::ClearanceDelivery, localScopePoint));
    EXPECT_EQ(
        tower->tryFindPosition(ControllerPosition::Type::ClearanceDelivery, localScopePoint)->type(),
        ControllerPosition::Type::Local);

    EXPECT_EQ(tower->tryFindPosition(ControllerPosition::Type::ClearanceDelivery, outsideLocalScope), nullptr);
}

TEST(ControllerPositionTest, appliesVerticalAirspaceBoundsWhenSelectingPosition)
{
    auto host = TestHostServices::create();
    Airport::Header header("KAAA", "Vertical Scope Test", GeoPoint(30.0, 40.0), 0);
    auto airspace = WorldBuilder::assembleSampleAirportControlZone(header);
    auto tower = WorldBuilder::assembleAirportTower(host, header, airspace, {
        { ControllerPosition::Type::Approach, 124700, GeoPolygon::empty(), "" },
        { ControllerPosition::Type::Area, 133500, GeoPolygon::empty(), "" }
    });

    const GeoPoint probePoint = GeoMath::getPointAtDistance(header.datum(), 90.0f, 20.0f * 1852.0f);

    auto approachAtLowAltitude = tower->tryFindPosition(
        ControllerPosition::Type::Approach,
        probePoint,
        3000.0f);
    ASSERT_TRUE(approachAtLowAltitude);
    EXPECT_EQ(approachAtLowAltitude->type(), ControllerPosition::Type::Approach);

    auto approachAtHighAltitude = tower->tryFindPosition(
        ControllerPosition::Type::Approach,
        probePoint,
        30000.0f);
    EXPECT_EQ(approachAtHighAltitude, nullptr);

    auto areaAtHighAltitude = tower->tryFindPosition(
        ControllerPosition::Type::Area,
        probePoint,
        30000.0f);
    ASSERT_TRUE(areaAtHighAltitude);
    EXPECT_EQ(areaAtHighAltitude->type(), ControllerPosition::Type::Area);
}
