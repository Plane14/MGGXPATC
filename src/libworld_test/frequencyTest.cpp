// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#include <memory>

#include "gtest/gtest.h"
#include "libworld.h"
#include "libworld_test.h"

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

    class TestControllerContinueIntent : public Intent
    {
    public:
        static constexpr int IntentCode = 99010;
    public:
        TestControllerContinueIntent(
            uint64_t id,
            shared_ptr<ControllerPosition> subjectControl,
            shared_ptr<Flight> subjectFlight) : Intent(
                id,
                0,
                Direction::ControllerToPilot,
                Type::Information,
                IntentCode,
                ConversationState::Continue,
                subjectControl,
                subjectFlight)
        {
        }
    };

    class TestPilotReplyIntent : public Intent
    {
    public:
        static constexpr int IntentCode = 99011;
    public:
        TestPilotReplyIntent(
            uint64_t id,
            uint64_t replyToId,
            shared_ptr<Flight> subjectFlight,
            shared_ptr<ControllerPosition> subjectControl) : Intent(
                id,
                replyToId,
                Direction::PilotToController,
                Type::Affirmation,
                IntentCode,
                ConversationState::End,
                subjectControl,
                subjectFlight)
        {
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

TEST(FrequencyTest, replyCanMatchEarlierOpenConversationWhileAnotherConversationIsAlsoOpen)
{
    auto host = TestHostServices::create();
    host->services().use<PhraseologyService>(make_shared<NoopPhraseologyService>());
    auto airport = createAirport(host, "KAAA", { 30.0, 40.0 }, 121900, 118300, "03", "21");
    auto world = WorldBuilder::assembleSampleWorld(host, { airport });
    host->useWorld(world);

    auto flight1 = host->addIfrFlight(701, "KAAA", "KAAA", GeoPoint(30.0, 40.0), Altitude::agl(2000));
    auto flight2 = host->addIfrFlight(702, "KAAA", "KAAA", GeoPoint(30.0, 40.0), Altitude::agl(2500));

    auto localPosition = airport->tower()->findPositionOrThrow(ControllerPosition::Type::Local, airport->header().datum());
    ASSERT_TRUE(localPosition);

    auto frequency = localPosition->frequency();
    ASSERT_TRUE(frequency);

    auto firstConversation = make_shared<TestControllerContinueIntent>(1, localPosition, flight1.ptr);
    auto secondConversation = make_shared<TestControllerContinueIntent>(2, localPosition, flight2.ptr);
    auto firstReply = make_shared<TestPilotReplyIntent>(3, 1, flight1.ptr, localPosition);

    frequency->enqueueTransmission(firstConversation);
    world->progressTo(chrono::microseconds(1));

    frequency->enqueueTransmission(secondConversation);
    frequency->enqueuePushToTalk(chrono::milliseconds(0), firstReply);

    world->progressTo(chrono::microseconds(2));
    world->progressTo(chrono::microseconds(3));

    const auto& history = host->textToSpeechService()->transmissionHistory();
    ASSERT_EQ(history.size(), 3u);
    EXPECT_EQ(history[0]->intent()->id(), 1u);
    EXPECT_EQ(history[1]->intent()->id(), 2u);
    EXPECT_EQ(history[2]->intent()->id(), 3u);
    EXPECT_EQ(history[2]->intent()->replyToId(), 1u);
}