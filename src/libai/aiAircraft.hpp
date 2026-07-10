//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once

#include <chrono>
#include <sstream>
#include <iomanip>
#include <cctype>
#include <cmath>

#include "libworld.h"
#include "worldHelper.hpp"
#include "basicManeuverTypes.hpp"
#include "maneuverFactory.hpp"
#include "intentTypes.hpp"
#include "intentFactory.hpp"
#include "aircraftPerformanceTable.hpp"

#define METERS_IN_NAUTICAL_MILE 1852.0
#define MICROSECONDS_IN_HOUR 3600000000.0
#define MICROSECONDS_IN_MINUTE 60000000.0

// the highest elevation airport - Qamdo Bamda, China
#define WORLD_MAX_RUNWAY_ELEVATION 14219.0

using namespace std;
using namespace world;

namespace ai
{
    class AIAircraft : public world::Aircraft
    {
    public:
        enum class MissionProfile
        {
            None = 0,
            Training = 1,
            Patrol = 2,
            LowLevel = 3
        };

        enum class FormationRole
        {
            None = 0,
            Leader = 1,
            Wingman = 2
        };

    private:
        static string normalizeWaypointName(string value)
        {
            value.erase(remove_if(value.begin(), value.end(), [](unsigned char c) {
                return !isalnum(c);
            }), value.end());

            transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
                return static_cast<char>(toupper(c));
            });

            return value;
        }

        static bool tryFindWaypointLocation(const shared_ptr<FlightPlan>& plan, const string& waypointName, GeoPoint& location)
        {
            if (!plan || waypointName.empty())
            {
                return false;
            }

            const string normalizedWaypointName = normalizeWaypointName(waypointName);
            for (const auto& routeWaypoint : plan->knownWaypoints())
            {
                if (routeWaypoint.location == GeoPoint::empty)
                {
                    continue;
                }

                if (normalizeWaypointName(routeWaypoint.name) == normalizedWaypointName)
                {
                    location = routeWaypoint.location;
                    return true;
                }
            }

            return false;
        }

        void separateArrivalSpawn(GeoPoint& arrivalStartLocation, float arrivalHeading, float& arrivalAltitudeFeet, float arrivalGroundSpeedKt)
        {
            auto world = host()->getWorld();
            auto flightPtr = flight().lock();
            if (!world || !flightPtr || arrivalStartLocation == GeoPoint::empty)
            {
                return;
            }

            const auto inferWakeClass = [](const shared_ptr<Aircraft>& aircraft) {
                if (!aircraft)
                {
                    return 1; // medium
                }

                const auto category = aircraft->category();
                // Super wake class for very large aircraft (A380, AN225).
                if (category == Aircraft::Category::Heavy)
                {
                    const string& icao = aircraft->modelIcao();
                    if (icao == "A388" || icao == "A380" || icao == "AN25" || icao == "A225")
                    {
                        return 3; // Super
                    }
                    return 2; // Heavy
                }
                if (category == Aircraft::Category::LightProp ||
                    category == Aircraft::Category::Prop ||
                    category == Aircraft::Category::Helicopter)
                {
                    return 0;
                }

                return 1;
            };

            const auto finalApproachSpacingNm = [](int leaderWakeClass, int followerWakeClass) {
                // ICAO Doc 4444 wake turbulence separation matrix (NM on final approach).
                // Leader classes: 0=Light, 1=Medium, 2=Heavy, 3=Super (A380/AN225).
                // Rows = leader, Cols = follower [Light, Medium, Heavy, Super].
                static const float spacingMatrix[4][4] = {
                    /*Light  behind:*/ { 3.0f, 3.0f, 3.0f, 3.0f },  // Light leader
                    /*Medium behind:*/ { 5.0f, 3.0f, 3.0f, 3.0f },  // Medium leader
                    /*Heavy  behind:*/ { 6.0f, 5.0f, 4.0f, 4.0f },  // Heavy leader
                    /*Super  behind:*/ { 8.0f, 7.0f, 6.0f, 6.0f },  // Super leader
                };
                const int lr = max(0, min(3, leaderWakeClass));
                const int fr = max(0, min(3, followerWakeClass));
                return spacingMatrix[lr][fr];
            };

            const int followerWakeClass = inferWakeClass(flightPtr->aircraft());
            const float speedBasedSpacingNm = max(4.0f, min(9.0f, m_performanceProfile.approachSpeedKt / 30.0f));
            const float spawnSpeedKt = max(90.0f, arrivalGroundSpeedKt);

            for (int attempt = 0; attempt < 8; ++attempt)
            {
                float requiredShiftNm = 0.0f;
                float requiredAltitudeFeet = arrivalAltitudeFeet;

                for (const auto& otherFlight : world->flights())
                {
                    if (!otherFlight || otherFlight == flightPtr || !otherFlight->aircraft())
                    {
                        continue;
                    }

                    auto otherAircraft = otherFlight->aircraft();
                    if (otherAircraft->location() == GeoPoint::empty)
                    {
                        continue;
                    }

                    if (otherAircraft->altitude().isGround() && otherAircraft->groundSpeedKt() < 5.0)
                    {
                        continue;
                    }

                    const int leaderWakeClass = inferWakeClass(otherAircraft);
                    const float minimumSpacingNm = max(
                        speedBasedSpacingNm,
                        finalApproachSpacingNm(leaderWakeClass, followerWakeClass));

                    const float lateralDistanceNm = static_cast<float>(
                        GeoMath::getDistanceMeters(arrivalStartLocation, otherAircraft->location()) / METERS_IN_NAUTICAL_MILE);
                    if (lateralDistanceNm >= minimumSpacingNm)
                    {
                        continue;
                    }

                    requiredShiftNm = max(requiredShiftNm, minimumSpacingNm - lateralDistanceNm + 0.5f);

                    // Preserve at least 1000 ft vertical spacing around busy final/arrival airspace.
                    if (lateralDistanceNm < 15.0f)
                    {
                        float otherAltitudeFeet = static_cast<float>(otherAircraft->altitude().feet());
                        if (otherAircraft->altitude().type() == Altitude::Type::AGL)
                        {
                            otherAltitudeFeet += host()->getWorld()->queryTerrainElevationAt(otherAircraft->location());
                        }

                        if (fabs(arrivalAltitudeFeet - otherAltitudeFeet) < 1000.0f)
                        {
                            requiredAltitudeFeet = max(requiredAltitudeFeet, otherAltitudeFeet + 1000.0f);
                        }
                    }
                }

                if (requiredShiftNm <= 0.0f)
                {
                    arrivalAltitudeFeet = max(arrivalAltitudeFeet, requiredAltitudeFeet);
                    break;
                }

                arrivalStartLocation = GeoMath::getPointAtDistance(
                    arrivalStartLocation,
                    GeoMath::flipHeading(arrivalHeading),
                    requiredShiftNm * METERS_IN_NAUTICAL_MILE);

                const float addedMinutes = requiredShiftNm * 60.0f / spawnSpeedKt;
                arrivalAltitudeFeet += addedMinutes * m_performanceProfile.descentRateFpm;
                arrivalAltitudeFeet = max(arrivalAltitudeFeet, requiredAltitudeFeet);
            }
        }

        void configureArrivalStart(const Runway::End& runwayEnd, bool preferApproachOnly, bool assignFinalManeuver)
        {
            const float minutesToThreshold = m_performanceProfile.minutesToThreshold;
            const float descentSpeedFpm = m_performanceProfile.descentRateFpm;
            const float groundSpeedKt = m_performanceProfile.approachSpeedKt;
            const float flareBufferFeet = m_performanceProfile.flareBufferFeet;

            auto flightPtr = flight().lock();
            auto plan = flightPtr ? flightPtr->plan() : nullptr;
            if (!flightPtr || !plan)
            {
                throw runtime_error("AIAircraft::configureArrivalStart failed: flight plan was not available");
            }

            shared_ptr<Runway> landingRunway;
            string landingRunwayName = runwayEnd.name();
            const bool hasLandingRunway = tryGetRunwayForPhase(Flight::Phase::Arrival, landingRunway, landingRunwayName);

            GeoPoint arrivalStartLocation;
            float arrivalHeading = runwayEnd.heading();
            float arrivalGroundSpeedKt = groundSpeedKt;
            float arrivalAltitudeFeet = runwayEnd.elevationFeet() + minutesToThreshold * descentSpeedFpm + flareBufferFeet;
            bool useProcedureStart = false;

            FlightPlan::Cursor arrivalCursor(plan);
            shared_ptr<FlightPlan::Leg> arrivalStartLeg;
            if (preferApproachOnly)
            {
                if (arrivalCursor.activateNextLegOfType(FlightPlan::LegType::Approach) && arrivalCursor.activeLeg())
                {
                    arrivalStartLeg = arrivalCursor.activeLeg();
                }
                else if (arrivalCursor.activateNextLegOfType(FlightPlan::LegType::Star) && arrivalCursor.activeLeg())
                {
                    arrivalStartLeg = arrivalCursor.activeLeg();
                }
            }
            else
            {
                if (arrivalCursor.activateNextLegOfType(FlightPlan::LegType::Star) && arrivalCursor.activeLeg())
                {
                    arrivalStartLeg = arrivalCursor.activeLeg();
                }
                else if (arrivalCursor.activateNextLegOfType(FlightPlan::LegType::Approach) && arrivalCursor.activeLeg())
                {
                    arrivalStartLeg = arrivalCursor.activeLeg();
                }
            }

            if (arrivalStartLeg)
            {
                GeoPoint startPoint;
                GeoPoint nextPoint;
                if (tryFindWaypointLocation(plan, arrivalStartLeg->fromNavaid(), startPoint))
                {
                    arrivalStartLocation = startPoint;
                    useProcedureStart = true;
                    if (arrivalStartLeg->targetAltitude() > 0.0f)
                    {
                        arrivalAltitudeFeet = max(arrivalAltitudeFeet, arrivalStartLeg->targetAltitude());
                    }

                    if (tryFindWaypointLocation(plan, arrivalStartLeg->toNavaid(), nextPoint))
                    {
                        arrivalHeading = GeoMath::getHeadingFromPoints(arrivalStartLocation, nextPoint);
                    }

                    if (arrivalStartLeg->targetSpeed() > 0.0f)
                    {
                        arrivalGroundSpeedKt = min(arrivalGroundSpeedKt, arrivalStartLeg->targetSpeed());
                    }
                    else if (arrivalStartLeg->type() == FlightPlan::LegType::Star)
                    {
                        // STAR entry is typically 30-80 NM from threshold — use a higher
                        // descent speed instead of approach speed so aircraft don't crawl
                        // through the entire arrival procedure unrealistically slowly.
                        const float starSpeedKt = min(
                            max(groundSpeedKt, m_performanceProfile.takeoffInitialClimbSpeedKt * 1.15f),
                            static_cast<float>(m_performanceProfile.ceilingFl > 250 ? 280.0f : 220.0f));
                        arrivalGroundSpeedKt = starSpeedKt;
                    }
                }
            }

            GeoPoint finalStartLocation = GeoPoint::empty;
            if (!useProcedureStart)
            {
                const float finalDistanceMeters = minutesToThreshold * groundSpeedKt / 60.0f * METERS_IN_NAUTICAL_MILE;
                GeoPoint aimingPoint = runwayEnd.centerlinePoint().geo();
                if (hasLandingRunway && landingRunway)
                {
                    aimingPoint = landingTouchdownPointForRunway(runwayEnd, *landingRunway);
                }

                finalStartLocation = GeoMath::getPointAtDistance(
                    aimingPoint,
                    GeoMath::flipHeading(runwayEnd.heading()),
                    max(50.0f, finalDistanceMeters - 50.0f));
                arrivalStartLocation = finalStartLocation;
            }

            separateArrivalSpawn(arrivalStartLocation, arrivalHeading, arrivalAltitudeFeet, arrivalGroundSpeedKt);
            if (!useProcedureStart)
            {
                finalStartLocation = arrivalStartLocation;
            }

            const GeoPoint& altitudeReferenceLocation = useProcedureStart ? arrivalStartLocation : finalStartLocation;
            if (altitudeReferenceLocation != GeoPoint::empty)
            {
                const float terrainElevationFeet = host()->getWorld()->queryTerrainElevationAt(altitudeReferenceLocation);
                arrivalAltitudeFeet = max(arrivalAltitudeFeet, terrainElevationFeet + MaxAltitudeAGL);
            }

            setAltitude(Altitude::msl(arrivalAltitudeFeet));
            setGroundSpeedKt(arrivalGroundSpeedKt);
            setVerticalSpeedFpm(-descentSpeedFpm);
            setFlapState(0);
            setGearState(0);
            setLights(LightBits::BeaconLandingNavStrobe);
            // Derive pitch from descent geometry: pitch ≈ -atan(VS_fpm / (GS_kt * 101.3))
            const float effectiveGsKt = max(90.0f, arrivalGroundSpeedKt);
            const float pitchRad = atan2(descentSpeedFpm, effectiveGsKt * 101.3f);
            const float arrivalPitchDeg = max(-6.0f, min(-1.0f, -pitchRad * 57.2958f));
            // Preserve current pitch and let the flight loop smoothly transition to target.
            setAttitude(AircraftAttitude(arrivalHeading, m_attitude.pitch(), 0));
            m_targetPitchDeg = arrivalPitchDeg;

            flightPtr->setPhase(Flight::Phase::Arrival);

            m_locationTimestamp = host()->getWorld()->timestamp();

            if (useProcedureStart)
            {
                setLocation(arrivalStartLocation);
                host()->writeLog(
                    "AIPILO|flight[%s] starting arrival at procedure waypoint[%s] runway[%s] retry[%d]",
                    flightPtr->callSign().c_str(),
                    arrivalStartLeg->fromNavaid().c_str(),
                    runwayEnd.name().c_str(),
                    preferApproachOnly ? 1 : 0);
            }
            else
            {
                setLocation(finalStartLocation);
            }

            if (assignFinalManeuver)
            {
                setManeuver(flightPtr->pilot()->getFinalToGate(runwayEnd));
            }
        }

        bool tryGetRunwayForPhase(Flight::Phase phase, shared_ptr<Runway>& runway, string& runwayName) const
        {
            auto flightPtr = flight().lock();
            auto plan = flightPtr ? flightPtr->plan() : nullptr;
            auto world = host() ? host()->getWorld() : nullptr;
            if (!plan || !world)
            {
                return false;
            }

            string airportIcao;
            switch (phase)
            {
            case Flight::Phase::Departure:
                airportIcao = plan->departureAirportIcao();
                runwayName = plan->departureRunway();
                break;
            case Flight::Phase::Arrival:
                airportIcao = plan->arrivalAirportIcao();
                runwayName = plan->arrivalRunway();
                break;
            default:
                return false;
            }

            if (airportIcao.empty() || runwayName.empty())
            {
                return false;
            }

            try
            {
                runway = world->getRunway(airportIcao, runwayName);
                return !!runway;
            }
            catch (const exception&)
            {
                return false;
            }
        }

        tuple<double, double> runwayAlignmentMeters(const Runway::End& runwayEnd, const GeoPoint& point) const
        {
            const GeoPoint threshold = runwayEnd.centerlinePoint().geo();
            const double distanceMeters = GeoMath::getDistanceMeters(threshold, point);
            const double headingToPoint = GeoMath::getHeadingFromPoints(threshold, point);
            const double headingOffsetRad = GeoMath::degreesToRadians(
                GeoMath::getTurnDegrees(runwayEnd.heading(), static_cast<float>(headingToPoint)));
            return make_tuple(
                cos(headingOffsetRad) * distanceMeters,
                sin(headingOffsetRad) * distanceMeters);
        }

        bool canTouchDownAtCurrentArrivalLocation() const
        {
            shared_ptr<Runway> runway;
            string runwayName;
            if (!tryGetRunwayForPhase(Flight::Phase::Arrival, runway, runwayName) || !runway)
            {
                return true;
            }

            const auto& runwayEnd = runway->getEndOrThrow(runwayName);
            const auto alignment = runwayAlignmentMeters(runwayEnd, m_location);
            const double alongTrackMeters = get<0>(alignment);
            const double crossTrackMeters = fabs(get<1>(alignment));
            const double touchdownDistanceMeters = landingTouchdownDistanceMetersForRunway(*runway) - 5.0;
            const double lateralToleranceMeters = max(8.0, static_cast<double>(runway->widthMeters()) * 0.4);

            return alongTrackMeters >= touchdownDistanceMeters && crossTrackMeters <= lateralToleranceMeters;
        }

        void constrainToActiveRunwayCenterline()
        {
            auto flightPtr = flight().lock();
            if (!flightPtr)
            {
                return;
            }

            // Helicopters should not be forcibly snapped to runway centerline.
            // They often hover, sidestep, and depart/arrive offset from runway axis.
            if (category() == Aircraft::Category::Helicopter)
            {
                return;
            }

            const bool departurePhase = flightPtr->phase() == Flight::Phase::Departure;
            const bool arrivalPhase = flightPtr->phase() == Flight::Phase::Arrival;
            if (!departurePhase && !arrivalPhase)
            {
                return;
            }

            shared_ptr<Runway> runway;
            string runwayName;
            if (!tryGetRunwayForPhase(departurePhase ? Flight::Phase::Departure : Flight::Phase::Arrival, runway, runwayName) || !runway)
            {
                return;
            }

            const auto& runwayEnd = runway->getEndOrThrow(runwayName);
            const auto alignment = runwayAlignmentMeters(runwayEnd, m_location);
            const double alongTrackMeters = get<0>(alignment);
            const double crossTrackMeters = get<1>(alignment);
            const bool nearRunway = runway->bounds().contains(m_location) ||
                GeoMath::getDistanceMeters(m_location, runwayEnd.centerlinePoint().geo()) <= 900.0;

            if (!nearRunway)
            {
                return;
            }

            if (arrivalPhase && alongTrackMeters < 0.0)
            {
                return;
            }

            if (departurePhase && alongTrackMeters < -20.0)
            {
                return;
            }

            if (alongTrackMeters > runway->lengthMeters() + 60.0)
            {
                return;
            }

            if (arrivalPhase && m_altitude.type() == Altitude::Type::Ground &&
                m_groundSpeedKt <= m_performanceProfile.landingExitSpeedKt + 5.0f)
            {
                return;
            }

            // During departure, stop constraining once the aircraft has rotated.
            // A pitch > 3 deg indicates the aircraft is airborne or lifting off.
            if (departurePhase && m_attitude.pitch() > 3.0f)
            {
                return;
            }

            // During arrival, ease off centerline constraint during flare
            // (below ~30 ft AGL) so the aircraft can drift naturally.
            const bool inFlare = arrivalPhase && !m_altitude.isGroundBased() &&
                m_altitude.type() == Altitude::Type::AGL && m_altitude.feet() < 30.0f;

            if (fabs(crossTrackMeters) <= 1.5)
            {
                return;
            }

            // During departure, do not clamp to runway length. Clamping to the physical
            // runway end can pin the aircraft in place once it rotates near threshold,
            // producing a visual "hover" before it finally escapes the snap envelope.
            const double minAlongTrackMeters = departurePhase ? -40.0 : 0.0;
            const double maxAlongTrackMeters = departurePhase
                ? max(static_cast<double>(runway->lengthMeters()) + 400.0, alongTrackMeters)
                : static_cast<double>(runway->lengthMeters());
            const double snappedAlongTrackMeters = max(minAlongTrackMeters, min(maxAlongTrackMeters, alongTrackMeters));
            GeoPoint centerlinePoint = GeoMath::getPointAtDistance(
                runwayEnd.centerlinePoint().geo(),
                runwayEnd.heading(),
                static_cast<float>(snappedAlongTrackMeters));
            centerlinePoint.altitude = m_location.altitude;

            const double correctionDistanceMeters = GeoMath::getDistanceMeters(m_location, centerlinePoint);
            // Reduce max correction in flare for natural touchdown drift.
            const double maxCorrectionMeters = inFlare
                ? max(0.3, min(1.5, abs(m_groundSpeedKt) * 0.03))
                : max(1.0, min(4.0, abs(m_groundSpeedKt) * 0.05));

            GeoPoint correctedLocation = centerlinePoint;
            if (correctionDistanceMeters > maxCorrectionMeters)
            {
                correctedLocation = GeoMath::getPointAtDistance(
                    m_location,
                    GeoMath::getHeadingFromPoints(m_location, centerlinePoint),
                    static_cast<float>(maxCorrectionMeters));
                correctedLocation.altitude = m_location.altitude;
            }

            m_inKinematicMove = true;
            setLocation(correctedLocation);
            // Only force heading alignment when significantly off centerline.
            if (fabs(crossTrackMeters) > 6.0)
            {
                setAttitude(m_attitude.withHeading(runwayEnd.heading()), TrackSyncMode::SyncToHeading);
            }
            m_inKinematicMove = false;
        }

        GeoPoint m_location;
        chrono::microseconds m_locationTimestamp;
        chrono::microseconds m_touchdownTimestamp;
        AircraftAttitude m_attitude;
        Altitude m_altitude;
        double m_track;
        double m_targetTrack;
        double m_groundSpeedKt;
        double m_targetGroundSpeedKt;
        double m_verticalSpeedFpm;
        double m_targetVerticalSpeedFpm;
        double m_targetPitchDeg;
        string m_squawk;
        LightBits m_lights;
        float m_gearState;
        float m_flapState;
        float m_spoilerState;
        AircraftPerformanceProfile m_performanceProfile;
        shared_ptr<Maneuver> m_maneuver;
        bool m_inTimedProgress;
        bool m_inKinematicMove;
        bool m_locationExplicitlyControlledThisTick;
        bool m_altitudeExplicitlyControlledThisTick;
        MissionProfile m_missionProfile;
        FormationRole m_formationRole;
        weak_ptr<world::Aircraft> m_formationLeader;
        double m_formationTrailOffsetNm;
        double m_formationLateralOffsetNm;
        double m_formationVerticalOffsetFt;
    public:
        AIAircraft(
            shared_ptr<HostServices> _host,
            int _id,
            const string& _modelIcao,
            const string& _airlineIcao,
            const string& _tailNo,
            Category _category
        ) : Aircraft(
                _host,
                _id,
                Actor::Nature::AI,
                _modelIcao,
                _airlineIcao,
                _tailNo,
                _category
            ),
            m_location(0, 0),
            m_attitude({ 0, 0, 0 }),
            m_track(0),
            m_targetTrack(0),
            m_groundSpeedKt(0),
            m_targetGroundSpeedKt(0),
            m_verticalSpeedFpm(0),
            m_targetVerticalSpeedFpm(0),
            m_targetPitchDeg(0.0),
            m_gearState(1.0f),
            m_flapState(0),
            m_spoilerState(0),
            m_locationTimestamp(chrono::seconds(-1)),
            m_touchdownTimestamp(chrono::seconds(-1)),
            m_altitude(Altitude::ground()),
            m_lights(LightBits::None),
            m_inTimedProgress(false),
            m_inKinematicMove(false),
            m_locationExplicitlyControlledThisTick(false),
            m_altitudeExplicitlyControlledThisTick(false),
            m_missionProfile(MissionProfile::None),
            m_formationRole(FormationRole::None),
            m_formationTrailOffsetNm(0.35),
            m_formationLateralOffsetNm(-0.18),
            m_formationVerticalOffsetFt(-40.0)
        {
            m_performanceProfile = AircraftPerformanceTable::lookup(_host, _modelIcao, _category);
        }

        const GeoPoint& location() const override { return m_location; }
        chrono::microseconds locationTimestamp() const { return m_locationTimestamp; }
        const AircraftAttitude& attitude() const override { return m_attitude; }
        double track() const override { return m_track; }
        const Altitude& altitude() const override { return m_altitude; }
        double groundSpeedKt() const override { return m_groundSpeedKt; }
        double verticalSpeedFpm() const override { return m_verticalSpeedFpm; }
        const string& squawk() const override { return m_squawk; }
        LightBits lights() const override { return m_lights; }
        float gearState() const override { return m_gearState; }
        float flapState() const override { return m_flapState; }
        float spoilerState() const override { return m_spoilerState; }
        const AircraftPerformanceProfile& performanceProfile() const { return m_performanceProfile; }
        MissionProfile missionProfile() const { return m_missionProfile; }
        FormationRole formationRole() const { return m_formationRole; }
        bool isFormationLeader() const { return m_formationRole == FormationRole::Leader; }
        bool isFormationWingman() const { return m_formationRole == FormationRole::Wingman && !m_formationLeader.expired(); }
        bool isLowLevelMission() const { return m_missionProfile == MissionProfile::LowLevel; }

        float landingTouchdownDistanceMetersForRunway(const Runway& runway) const
        {
            const float profileDistanceMeters = max(15.0f, m_performanceProfile.landingTouchdownDistanceMeters);
            const float maxUsableDistanceMeters = runway.lengthMeters() > 80.0f
                ? max(15.0f, runway.lengthMeters() - 60.0f)
                : profileDistanceMeters;
            return min(profileDistanceMeters, maxUsableDistanceMeters);
        }

        GeoPoint landingTouchdownPointForRunway(const Runway::End& runwayEnd, const Runway& runway) const
        {
            return GeoMath::getPointAtDistance(
                runwayEnd.centerlinePoint().geo(),
                runwayEnd.heading(),
                landingTouchdownDistanceMetersForRunway(runway));
        }

        bool isLightsOn(LightBits bits) const override
        {
            return ((m_lights & bits) == bits);
        }

        void setMissionProfile(MissionProfile profile)
        {
            m_missionProfile = profile;
        }

        void setFormationRole(FormationRole role)
        {
            m_formationRole = role;
            if (role != FormationRole::Wingman)
            {
                m_formationLeader.reset();
            }
        }

        void setFormationLeader(
            const shared_ptr<world::Aircraft>& leader,
            double trailOffsetNm = 0.35,
            double lateralOffsetNm = -0.18,
            double verticalOffsetFt = -40.0)
        {
            m_formationLeader = leader;
            m_formationRole = leader ? FormationRole::Wingman : FormationRole::None;
            m_formationTrailOffsetNm = max(0.05, trailOffsetNm);
            m_formationLateralOffsetNm = lateralOffsetNm;
            m_formationVerticalOffsetFt = verticalOffsetFt;
        }

        void clearFormation()
        {
            m_formationLeader.reset();
            m_formationRole = FormationRole::None;
        }

        shared_ptr<world::Aircraft> formationLeaderAircraft() const
        {
            return m_formationLeader.lock();
        }

        void park(shared_ptr<ParkingStand> parkingStand) override
        {
            m_location = GeoMath::getPointAtDistance(parkingStand->location().geo(), GeoMath::flipHeading(parkingStand->heading()), 13);
            m_attitude = AircraftAttitude({ parkingStand->heading(), 0, 0 });
            m_altitude = Altitude::ground();
            m_groundSpeedKt = 0;
            m_verticalSpeedFpm = 0;
            m_gearState = 1.0f;
            m_flapState = 0.0f;
            m_spoilerState = 0.0f;
            m_lights = LightBits::None;

            if (auto flightPtr = flight().lock())
            {
                flightPtr->setPhase(Flight::Phase::TurnAround);
            }

            setManeuver(flight().lock()->pilot()->getFlightCycle());
        }

        void setOnFinal(const Runway::End& runwayEnd) override
        {
            configureArrivalStart(runwayEnd, false, true);
        }

        void prepareForApproachRetry(const Runway::End& runwayEnd)
        {
            configureArrivalStart(runwayEnd, true, false);
        }

        void setOnHelipadFinal(shared_ptr<ParkingStand> parkingStand)
        {
            if (!parkingStand)
            {
                throw runtime_error("AIAircraft::setOnHelipadFinal failed: parking stand was not available");
            }

            auto flightPtr = flight().lock();
            if (!flightPtr)
            {
                throw runtime_error("AIAircraft::setOnHelipadFinal failed: flight was not available");
            }

            const GeoPoint touchdownPoint = parkingStand->location().geo();
            const float approachHeading = normalizeHeading(parkingStand->heading());
            const float terrainElevationFeet = host()->getWorld()->queryTerrainElevationAt(touchdownPoint);
            // Scale approach distance and altitude by approach speed (lighter helis need less room).
            const float approachSpeedKt = max(35.0f, min(80.0f, m_performanceProfile.approachSpeedKt));
            const float finalDistanceNm = max(0.3f, min(0.8f, approachSpeedKt / 120.0f));
            const float finalDistanceMeters = finalDistanceNm * METERS_IN_NAUTICAL_MILE;
            const float approachAltitudeFt = max(150.0f, min(400.0f, finalDistanceNm * 500.0f));
            GeoPoint finalStartLocation = GeoMath::getPointAtDistance(
                touchdownPoint,
                GeoMath::flipHeading(approachHeading),
                finalDistanceMeters);

            setLocation(finalStartLocation);
            setAltitude(Altitude::msl(terrainElevationFeet + approachAltitudeFt));
            setGroundSpeedKt(approachSpeedKt);
            setVerticalSpeedFpm(-max(180.0f, m_performanceProfile.descentRateFpm * 0.45f));
            setFlapState(0.0f);
            setGearState(1.0f);
            setSpoilerState(0.0f);
            setLights(LightBits::BeaconLandingNavStrobe);
            setAttitude(AircraftAttitude(approachHeading, -1.0f, 0.0f));

            flightPtr->setPhase(Flight::Phase::Arrival);
            m_locationTimestamp = host()->getWorld()->timestamp();
        }

        void progressTo(chrono::microseconds timestamp) override
        {
            chrono::microseconds lastTimestamp = m_locationTimestamp;
            if (lastTimestamp.count() < 0)
            {
                lastTimestamp = chrono::microseconds(0);
                try
                {
                    lastTimestamp = host()->getWorld()->timestamp();
                }
                catch (const exception&)
                {
                }

                m_locationTimestamp = lastTimestamp;
            }

            m_locationExplicitlyControlledThisTick = false;
            m_altitudeExplicitlyControlledThisTick = false;
            m_inTimedProgress = true;

            if (m_maneuver)
            {
                //m_host->writeLog("Aircraft[%d]: maneuver->progressTo(%lld)", m_id, timestamp.count());
                m_maneuver->progressTo(timestamp);
            }

            bool touchedDown = false;
            int64_t elapsedMicroseconds = max<int64_t>(0, (timestamp - lastTimestamp).count());

            moveFor(elapsedMicroseconds, touchedDown);
            m_inTimedProgress = false;

            m_locationTimestamp = timestamp;
            if (touchedDown)
            {
                m_touchdownTimestamp = timestamp;
            }
        }

        void setLocation(const GeoPoint& _location)
        {
            //m_host->writeLog("Aircraft[%d]::setLocation(lat=%.10f,lon=%.10f,alt=%f)", m_id, _location.latitude, _location.longitude, _location.altitude);
            if (m_inTimedProgress && !m_inKinematicMove)
            {
                m_locationExplicitlyControlledThisTick = true;
            }
            m_location = _location;
            notifyChanges();
        }

        void setAttitude(const AircraftAttitude& _attitude, TrackSyncMode trackSync = TrackSyncMode::SyncToHeading)
        {
            //m_host->writeLog("Aircraft[%d]::setAttitude(hdg=%f)", m_id, _attitude.heading());
            m_attitude = _attitude;
            m_targetPitchDeg = _attitude.pitch();

            if (trackSync == TrackSyncMode::SyncToHeading)
            {
                setTrack(m_attitude.heading());
            }

            notifyChanges();
        }

        void setAltitude(const Altitude& _altitude)
        {
            // m_host->writeLog(
            //     "Aircraft[%d]::setAltitude(%f %s)",
            //     m_id,
            //     _altitude.feet(),
            //     _altitude.isGround() ? "GND" : _altitude.type() == Altitude::Type::AGL ? "AGL" : "MSL");

            if (m_inTimedProgress && !m_inKinematicMove)
            {
                m_altitudeExplicitlyControlledThisTick = true;
            }
            m_altitude = _altitude;
            notifyChanges();
        }

        void setTrack(double _track)
        {
            m_targetTrack = normalizeHeading(_track);
            if (!m_inTimedProgress || abs(m_groundSpeedKt) < 1.0)
            {
                m_track = m_targetTrack;
            }
        }

        // Immediately stops the aircraft and syncs track/heading to the given runway heading.
        // Bypasses the target/slew system; safe to call during timed progress.
        void hardStop(float headingDegrees)
        {
            m_groundSpeedKt = 0.0;
            m_targetGroundSpeedKt = 0.0;
            m_track = normalizeHeading(headingDegrees);
            m_targetTrack = m_track;
            m_attitude = m_attitude.withHeading(headingDegrees).withRoll(0.0f).withPitch(0.0f);
            m_targetPitchDeg = 0.0;
            notifyChanges();
        }

        void setGroundSpeedKt(double kt)
        {
            m_targetGroundSpeedKt = clampTargetGroundSpeedKt(kt);
            if (!m_inTimedProgress)
            {
                m_groundSpeedKt = m_targetGroundSpeedKt;
            }
        }

        void setVerticalSpeedFpm(double fpm)
        {
            m_targetVerticalSpeedFpm = clampTargetVerticalSpeedFpm(fpm);
            if (!m_inTimedProgress)
            {
                m_verticalSpeedFpm = m_targetVerticalSpeedFpm;
            }
        }

        void setGearState(float ratio)
        {
            m_gearState = ratio;
            notifyChanges();
        }

        void setSpoilerState(float ratio)
        {
            m_spoilerState = ratio;
            notifyChanges();
        }

        void setFlapState(float ratio)
        {
            m_flapState = ratio;
            notifyChanges();
        }

        void setSquawk(const string& _squawk)
        {
            m_squawk = _squawk;
            notifyChanges();
        }

        void setLights(LightBits _lights)
        {
            m_lights = _lights;
            notifyChanges();
        }

        void setManeuver(shared_ptr<Maneuver> _maneuver)
        {
            m_maneuver = _maneuver;
        }

        string getStatusString() override
        {
            return m_maneuver ? m_maneuver->getStatusString() : "N/A";
        }

        void notifyChanges() override
        {
            getWorldChangeSet()->mutableFlights().updated(flight().lock());
        }

        void moveFor(int64_t elapsedMicroseconds, bool& touchedDown)
        {
            updateDynamicFlightModel(elapsedMicroseconds);

            if (!m_locationExplicitlyControlledThisTick && abs(m_groundSpeedKt) > 0.00001)
            {
                double elapsedHours = elapsedMicroseconds / MICROSECONDS_IN_HOUR;
                GeoPoint nextLocation = GeoMath::getPointAtDistance(
                    m_location,
                    m_track,
                    m_groundSpeedKt * elapsedHours * METERS_IN_NAUTICAL_MILE);
                m_inKinematicMove = true;
                setLocation(nextLocation);
                m_inKinematicMove = false;
                constrainToActiveRunwayCenterline();
            }

            if (!m_altitudeExplicitlyControlledThisTick && abs(m_verticalSpeedFpm) > 0.00001)
            {
                double elapsedMinutes = elapsedMicroseconds / MICROSECONDS_IN_MINUTE;
                float nextFeet = m_altitude.feet() + m_verticalSpeedFpm * elapsedMinutes;
                Altitude nextAltitude  = getNextAltitude(nextFeet);
                touchedDown = (
                    m_altitude.type() != Altitude::Type::Ground &&
                    nextAltitude.type() == Altitude::Type::Ground);
                m_inKinematicMove = true;
                setAltitude(nextAltitude);
                m_inKinematicMove = false;
            }
        }

        tuple<double, double> getGroundSpeedEnvelopeKt() const
        {
            auto flightPtr = flight().lock();
            const float approach = max(70.0f, m_performanceProfile.approachSpeedKt);
            const bool helicopter = category() == Aircraft::Category::Helicopter;

            if (helicopter)
            {
                if (m_altitude.type() == Altitude::Type::Ground)
                {
                    if (flightPtr && flightPtr->phase() == Flight::Phase::Departure)
                    {
                        return make_tuple(0.0, 35.0);
                    }

                    return make_tuple(0.0, 25.0);
                }

                const double minSpeed = 0.0;
                const double maxSpeed = (flightPtr && flightPtr->phase() == Flight::Phase::Arrival)
                    ? max(120.0, static_cast<double>(m_performanceProfile.approachSpeedKt) * 1.35)
                    : max(145.0, static_cast<double>(m_performanceProfile.takeoffInitialClimbSpeedKt) * 1.35);
                return make_tuple(minSpeed, maxSpeed);
            }

            if (m_altitude.type() == Altitude::Type::Ground)
            {
                if (flightPtr)
                {
                    switch (flightPtr->phase())
                    {
                    case Flight::Phase::Departure:
                        // While still ground-based in departure phase, allow takeoff roll
                        // but keep taxi/lineup from unrealistically exploding in speed.
                        return make_tuple(0.0, max(150.0, static_cast<double>(approach) * 1.45));
                    case Flight::Phase::Arrival:
                        return make_tuple(0.0, max(130.0, static_cast<double>(approach) * 1.25));
                    case Flight::Phase::TurnAround:
                    default:
                        return make_tuple(0.0, 40.0);
                    }
                }

                return make_tuple(0.0, 40.0);
            }

            const double minSpeed = max(80.0, static_cast<double>(approach) * 0.72);
            double maxSpeed = max(210.0, static_cast<double>(approach) * 2.7);
            if (flightPtr && flightPtr->phase() == Flight::Phase::Arrival)
            {
                maxSpeed = max(170.0, static_cast<double>(approach) * 1.8);
            }

            return make_tuple(minSpeed, maxSpeed);
        }

        tuple<double, double> getVerticalSpeedEnvelopeFpm() const
        {
            const auto flightPtr = flight().lock();
            const bool onGround = m_altitude.type() == Altitude::Type::Ground;
            const bool departurePhase = flightPtr && flightPtr->phase() == Flight::Phase::Departure;
            const bool arrivalPhase = flightPtr && flightPtr->phase() == Flight::Phase::Arrival;
            const bool helicopter = category() == Aircraft::Category::Helicopter;

            if (helicopter)
            {
                if (onGround)
                {
                    return make_tuple(0.0, departurePhase ? 900.0 : 250.0);
                }

                const double climbMax = max(900.0, min(3800.0, static_cast<double>(m_performanceProfile.initialClimbRocFpm) * 1.05));
                const double descentMin = -max(900.0, static_cast<double>(m_performanceProfile.descentRateFpm) * (arrivalPhase ? 1.8 : 1.5));
                return make_tuple(descentMin, climbMax);
            }

            if (onGround)
            {
                // Avoid runway/taxi vertical jitter; liftoff maneuvers explicitly command VS.
                return make_tuple(-120.0, departurePhase ? 4200.0 : 500.0);
            }

            const double unrestrictedClimbMax = max(1200.0, static_cast<double>(m_performanceProfile.initialClimbRocFpm) * 0.85);
            const double nominalClimbMax = max(1000.0, static_cast<double>(m_performanceProfile.initialClimbRocFpm) * 0.65);
            const double climbMax = m_performanceProfile.canPerformUnrestrictedClimbout() ? unrestrictedClimbMax : nominalClimbMax;

            const double descentFactor = arrivalPhase ? 2.8 : 2.4;
            const double descentMin = -max(700.0, static_cast<double>(m_performanceProfile.descentRateFpm) * descentFactor);
            return make_tuple(descentMin, climbMax);
        }

        static double normalizeHeading(double heading)
        {
            double normalized = fmod(heading, 360.0);
            if (normalized < 0.0)
            {
                normalized += 360.0;
            }
            return normalized;
        }

        static double approachValue(double current, double target, double delta)
        {
            if (current < target)
            {
                return min(target, current + delta);
            }
            return max(target, current - delta);
        }

        double clampTargetGroundSpeedKt(double kt) const
        {
            auto envelope = getGroundSpeedEnvelopeKt();
            return min(get<1>(envelope), max(get<0>(envelope), kt));
        }

        double clampTargetVerticalSpeedFpm(double fpm) const
        {
            auto envelope = getVerticalSpeedEnvelopeFpm();
            return min(get<1>(envelope), max(get<0>(envelope), fpm));
        }

        void updateDynamicFlightModel(int64_t elapsedMicroseconds)
        {
            const double elapsedSeconds = max(0.0, elapsedMicroseconds / 1000000.0);
            if (elapsedSeconds <= 0.0)
            {
                return;
            }

            updateMissionGuidance(elapsedSeconds);

            // Speed response constrained by envelope.
            m_targetGroundSpeedKt = clampTargetGroundSpeedKt(m_targetGroundSpeedKt);
            const bool onGround = m_altitude.type() == Altitude::Type::Ground;
            // Use aircraft-rated acceleration/deceleration during ground rolls.
            // Takeoff roll (Departure on ground): rated takeoff acceleration.
            // Landing roll (Arrival on ground): rated rollout deceleration.
            // All other ground movement and airborne: generic envelopes.
            auto flightPtr = flight().lock();
            const bool isTakeoffRoll = onGround && flightPtr && flightPtr->phase() == Flight::Phase::Departure;
            const bool isLandingRoll = onGround && flightPtr && flightPtr->phase() == Flight::Phase::Arrival;
            const double accelKtPerSecond = isTakeoffRoll
                ? max(3.0, static_cast<double>(m_performanceProfile.takeoffAccelerationKtPerSecond))
                : (onGround ? 3.0
                    : max(1.5, static_cast<double>(m_performanceProfile.takeoffAccelerationKtPerSecond) * 0.55));
            const double decelKtPerSecond = isLandingRoll
                ? max(4.5, static_cast<double>(m_performanceProfile.landingRolloutDecelerationKtPerSecond))
                : (isTakeoffRoll
                    ? max(4.5, static_cast<double>(m_performanceProfile.takeoffAccelerationKtPerSecond) * 1.5)
                    : (onGround ? 4.5
                        : max(2.0, static_cast<double>(m_performanceProfile.landingRolloutDecelerationKtPerSecond) * 0.7)));
            const double maxSpeedDelta = (m_targetGroundSpeedKt >= m_groundSpeedKt ? accelKtPerSecond : decelKtPerSecond) * elapsedSeconds;
            m_groundSpeedKt = approachValue(m_groundSpeedKt, m_targetGroundSpeedKt, maxSpeedDelta);

            // Vertical speed response constrained by envelope.
            m_targetVerticalSpeedFpm = clampTargetVerticalSpeedFpm(m_targetVerticalSpeedFpm);
            // Use climb-rate or descent-rate based response depending on direction of change,
            // rather than unrestricted-climbout capability which is unrelated to descent.
            const bool increasingVs = m_targetVerticalSpeedFpm > m_verticalSpeedFpm;
            const double verticalResponseFpmPerSecond = increasingVs
                ? max(700.0, static_cast<double>(m_performanceProfile.initialClimbRocFpm) * 0.45)
                : max(500.0, static_cast<double>(m_performanceProfile.descentRateFpm) * 0.65);
            const double maxVerticalDelta = verticalResponseFpmPerSecond * elapsedSeconds;
            m_verticalSpeedFpm = approachValue(m_verticalSpeedFpm, m_targetVerticalSpeedFpm, maxVerticalDelta);

            m_targetTrack = normalizeHeading(m_targetTrack);
            m_track = normalizeHeading(m_track);

            // Smooth pitch transition toward target (max ~3 deg/s).
            const double maxPitchDelta = 3.0 * elapsedSeconds;
            const double newPitch = approachValue(m_attitude.pitch(), m_targetPitchDeg, maxPitchDelta);
            if (abs(newPitch - m_attitude.pitch()) > 0.001)
            {
                m_attitude = m_attitude.withPitch(newPitch);
            }

            const double turnDegrees = GeoMath::getTurnDegrees(static_cast<float>(m_track), static_cast<float>(m_targetTrack));
            const bool helicopter = category() == Aircraft::Category::Helicopter;
            const bool fighter = category() == Aircraft::Category::Fighter;
            // Physics-based turn rate: rate = g·tan(bank)/V ≈ 1091·tan(bank)/V_kt (deg/s).
            double turnRateDegreesPerSecond;
            if (onGround) {
                // Realistic taxi-turn rates: large aircraft turn slowly on the ground.
                turnRateDegreesPerSecond = helicopter ? 8.0 : 5.0;
            } else {
                // Calculate required bank angle for desired turn rate
                // Standard rate turn: 3 deg/s = bank angle of ~15-25 deg depending on speed
                // Rate = g * tan(bank) / V, so bank = atan(rate * V / g)
                const double speedKt = max(80.0, abs(m_groundSpeedKt));
                const double speedMps = speedKt * 0.514444; // knots to m/s
                
                // Target turn rate based on aircraft category and speed
                double targetTurnRateDegPerSec;
                if (helicopter) {
                    targetTurnRateDegPerSec = min(18.0, max(6.0, 1091.0 * tan(20.0 * M_PI / 180.0) / speedKt));
                } else if (fighter) {
                    // Fighters can sustain higher bank angles (up to 60-70 deg)
                    targetTurnRateDegPerSec = min(20.0, max(4.0, 1091.0 * tan(45.0 * M_PI / 180.0) / speedKt));
                } else {
                    // Transport category: max 25-30 deg bank, standard rate ~3 deg/s at cruise
                    // At low speed, rate is higher for same bank
                    const double maxBankDeg = 30.0;
                    targetTurnRateDegPerSec = min(8.0, max(1.5, 1091.0 * tan(maxBankDeg * M_PI / 180.0) / speedKt));
                }
                
                // Current bank angle determines actual turn rate
                const double absRoll = max(1.0, abs(m_attitude.roll()));
                const double bankRad = absRoll * M_PI / 180.0;
                const double physicsRate = 1091.0 * tan(bankRad) / speedKt;
                
                // Limit by both physics and target rate
                turnRateDegreesPerSecond = min(physicsRate, targetTurnRateDegPerSec);
                
                // Auto-bank: if we need to turn, calculate required bank
                if (abs(turnDegrees) > 0.5) {
                    const double requiredRate = min(targetTurnRateDegPerSec, abs(turnDegrees) / elapsedSeconds);
                    const double requiredBankRad = atan(requiredRate * speedMps / 9.81);
                    const double requiredBankDeg = requiredBankRad * 180.0 / M_PI;
                    const double maxBankDeg = helicopter ? 20.0 : (fighter ? 60.0 : 30.0);
                    const double clampedBankDeg = min(maxBankDeg, max(5.0, requiredBankDeg));
                    const double bankSign = turnDegrees > 0.0 ? 1.0 : -1.0;
                    m_attitude = m_attitude.withRoll(clampedBankDeg * bankSign);
                } else if (abs(m_attitude.roll()) > 1.0) {
                    // Roll out smoothly when aligned
                    const double rollRate = 10.0 * elapsedSeconds; // 10 deg/s roll rate
                    const double currentRoll = m_attitude.roll();
                    const double newRoll = approachValue(currentRoll, 0.0, rollRate);
                    m_attitude = m_attitude.withRoll(newRoll);
                }
            }
            const double maxTurnDelta = turnRateDegreesPerSecond * elapsedSeconds;
            if (abs(turnDegrees) <= maxTurnDelta)
            {
                m_track = m_targetTrack;
            }
            else
            {
                m_track = normalizeHeading(m_track + (turnDegrees > 0.0 ? maxTurnDelta : -maxTurnDelta));
            }
        }

        double altitudeMslFeet(const Altitude& altitude, const GeoPoint& location) const
        {
            switch (altitude.type())
            {
            case Altitude::Type::Ground:
                return host()->getWorld()->queryTerrainElevationAt(location);
            case Altitude::Type::AGL:
                return altitude.feet() + host()->getWorld()->queryTerrainElevationAt(location);
            case Altitude::Type::MSL:
            default:
                return altitude.feet();
            }
        }

        void updateMissionGuidance(double elapsedSeconds)
        {
            (void)elapsedSeconds;

            auto flightPtr = flight().lock();
            if (!flightPtr)
            {
                return;
            }

            const bool airborne = !m_altitude.isGroundBased();
            if (!airborne || flightPtr->phase() == Flight::Phase::Arrival || flightPtr->phase() == Flight::Phase::TurnAround)
            {
                return;
            }

            if (m_missionProfile == MissionProfile::LowLevel)
            {
                const double terrainFeet = host()->getWorld()->queryTerrainElevationAt(m_location);
                const double desiredAglFeet = (category() == Aircraft::Category::Fighter) ? 1800.0 : 2500.0;
                const double desiredAltitudeFeet = terrainFeet + desiredAglFeet;
                const double currentAltitudeFeet = altitudeMslFeet(m_altitude, m_location);
                const double altitudeDeltaFeet = desiredAltitudeFeet - currentAltitudeFeet;
                if (abs(altitudeDeltaFeet) > 120.0)
                {
                    const double lowLevelVerticalTarget = clampTargetVerticalSpeedFpm(altitudeDeltaFeet * 3.2);
                    m_targetVerticalSpeedFpm = lowLevelVerticalTarget;
                }
            }

            if (m_formationRole != FormationRole::Wingman)
            {
                return;
            }

            auto leader = m_formationLeader.lock();
            if (!leader || leader.get() == this || leader->altitude().isGroundBased())
            {
                return;
            }

            const float leaderHeading = static_cast<float>(leader->track());
            GeoPoint desiredPoint = GeoMath::getPointAtDistance(
                leader->location(),
                GeoMath::flipHeading(leaderHeading),
                m_formationTrailOffsetNm * METERS_IN_NAUTICAL_MILE);
            if (abs(m_formationLateralOffsetNm) > 0.001)
            {
                desiredPoint = GeoMath::getPointAtDistance(
                    desiredPoint,
                    GeoMath::addTurnToHeading(leaderHeading, m_formationLateralOffsetNm >= 0.0 ? 90.0f : 270.0f),
                    abs(m_formationLateralOffsetNm) * METERS_IN_NAUTICAL_MILE);
            }

            const double distanceMeters = GeoMath::getDistanceMeters(m_location, desiredPoint);
            if (distanceMeters > 25.0)
            {
                m_targetTrack = GeoMath::getHeadingFromPoints(m_location, desiredPoint);
            }
            else
            {
                m_targetTrack = leader->track();
            }

            const double leaderSpeedKt = max(120.0, abs(leader->groundSpeedKt()));
            const double spacingErrorNm = distanceMeters / METERS_IN_NAUTICAL_MILE;
            // PD controller: proportional + derivative damping to reduce oscillation.
            const double closureRateKt = m_groundSpeedKt - leaderSpeedKt;
            const double proportionalGain = 80.0;
            const double dampingGain = 1.8;
            const double closureAdjustmentKt = min(45.0, max(-20.0, spacingErrorNm * proportionalGain - closureRateKt * dampingGain));
            m_targetGroundSpeedKt = clampTargetGroundSpeedKt(leaderSpeedKt + closureAdjustmentKt);

            const double currentAltitudeFeet = altitudeMslFeet(m_altitude, m_location);
            const double desiredAltitudeFeet = altitudeMslFeet(leader->altitude(), leader->location()) + m_formationVerticalOffsetFt;
            const double altitudeErrorFeet = desiredAltitudeFeet - currentAltitudeFeet;
            if (abs(altitudeErrorFeet) > 40.0)
            {
                // Damped altitude correction to prevent vertical oscillation.
                const double vsErrorFpm = m_verticalSpeedFpm - leader->verticalSpeedFpm();
                m_targetVerticalSpeedFpm = clampTargetVerticalSpeedFpm(altitudeErrorFeet * 3.5 - vsErrorFpm * 1.2);
            }
        }

        Altitude getNextAltitude(float nextFeet)
        {
            auto flightPtr = flight().lock();
            const bool departurePhase = flightPtr && flightPtr->phase() == Flight::Phase::Departure;
            const bool arrivalPhase = flightPtr && flightPtr->phase() == Flight::Phase::Arrival;

            switch (m_altitude.type())
            {
            case Altitude::Type::Ground:
            case Altitude::Type::AGL:
                if (nextFeet <= 0)
                {
                    if (arrivalPhase && !canTouchDownAtCurrentArrivalLocation())
                    {
                        return Altitude::agl(1.0f);
                    }

                    return Altitude::ground();
                }

                if (departurePhase)
                {
                    return Altitude::msl(nextFeet + host()->getWorld()->queryTerrainElevationAt(m_location));
                }

                return nextFeet > MaxAltitudeAGL
                    ? Altitude::msl(nextFeet + host()->getWorld()->queryTerrainElevationAt(m_location))
                    : Altitude::agl(nextFeet);
            case Altitude::Type::MSL:
                if (flightPtr)
                {
                    const float landingRunwayElevationFeet = flightPtr->landingRunwayElevationFeet();
                    const float terrainElevationFeet = arrivalPhase
                        ? max(landingRunwayElevationFeet, host()->getWorld()->queryTerrainElevationAt(m_location))
                        : host()->getWorld()->queryTerrainElevationAt(m_location);

                    if (nextFeet <= terrainElevationFeet)
                    {
                        return Altitude::ground();
                    }

                    if (arrivalPhase && nextFeet <= terrainElevationFeet + MaxAltitudeAGL)
                    {
                        return Altitude::agl(nextFeet - terrainElevationFeet);
                    }

                    return Altitude::msl(nextFeet);
                }
                return Altitude::ground();
            }

            throw runtime_error(
                "Aircraft id=" + to_string(id()) + " invalid altitude type=" + to_string((int)m_altitude.type()));
        }

        bool justTouchedDown(chrono::microseconds timestamp) override
        {
            auto microsecondsSinceTouchdown = (timestamp -  m_touchdownTimestamp);
            bool wasTouchDown = microsecondsSinceTouchdown.count() < 500000; // < 0.5s
            m_touchdownTimestamp = timestamp - chrono::seconds(1);
            return wasTouchDown;
        }
    };
}
