// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#pragma once
#define _USE_MATH_DEFINES

#include <cstdarg>
#include <cstring>
#include <string>
#include <memory>
#include <tuple>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <list>
#include <queue>
#include <functional>
#include <chrono>
#include "stlhelpers.h"


// lowest elevation in the world is -1355 MSL at Dead Sea
#define ALTITUDE_GROUND -10000
#define ALTITUDE_UNASSIGNED -11000
#define FREQUENCY_UNICOM_1228 122800
#define FREQUENCY_UNICOM_1227 122700
#define FREQUENCY_UNICOM_1230 123000
#define FEET_IN_1_METER 3.28084
#define KNOT_IN_1_METER_PER_SEC 1.9438444942
#define METERS_IN_1_NAUTICAL_MILE 1852

namespace world
{
    class HostServices;
    struct GeoPoint;
    struct LocalPoint;
    struct UniPoint;
    struct GeoPolygon;
    struct Vector3d;
    struct AircraftAttitude;
    struct Altitude;

    struct GeoPoint
    {
    public:
        double latitude;
        double longitude;
        double altitude; //to be removed
    public:
        GeoPoint() :
            latitude(0), longitude(0), altitude(0)
        {
        }
        GeoPoint(double _latitude, double _longitude, double _altitude = 0) :
            latitude(_latitude), longitude(_longitude), altitude(_altitude)
        {
        }
    public:
        static const GeoPoint empty;
    public:
        friend bool operator== (const GeoPoint& p1, const GeoPoint& p2);
        friend bool operator!= (const GeoPoint& p1, const GeoPoint& p2);    
    };

    struct GeoVector
    {
    public:
        GeoPoint p1;
        GeoPoint p2;
        double latitude;
        double longitude;
    public:
        GeoVector() :
            GeoVector({ 0, 0 }, {0, 0})
        {
        }
        GeoVector(const GeoPoint& _p1, const GeoPoint& _p2) :
            p1(_p1), p2(_p2)
        {
            longitude = p2.longitude - p1.longitude;
            latitude = p2.latitude - p1.latitude;
        }
    public:
        static const GeoVector empty;
    public:
        friend bool operator== (const GeoVector& u, const GeoVector& v);
        friend bool operator!= (const GeoVector& u, const GeoVector& v);
        friend double operator* (const GeoVector& u, const GeoVector& v);
    };

    struct LocalPoint
    {
        float x;
        float y;
        float z;
    };

    class UniPoint
    {
    private:
        enum class Type
        {
            local,
            geo
        };
    private:
        const std::shared_ptr<HostServices> m_services;
        const Type m_assignedType;
        LocalPoint m_local;
        GeoPoint m_geo;
    public:
        UniPoint(const GeoPoint& _geo);
        UniPoint(std::shared_ptr<HostServices> _services, const LocalPoint& _local);
        UniPoint(std::shared_ptr<HostServices> _services, const GeoPoint& _geo);
    public:
        // void moveByLocal(const LocalPoint& delta);
        // void moveByGeo(const GeoPoint& delta);
        const LocalPoint& local() const { return m_local; }
        const GeoPoint& geo() const { return m_geo; }
        double latitude() const { return m_geo.latitude; }
        double longitude() const { return m_geo.longitude; }
        double altitude() const { return m_geo.altitude; }
        float x() const { return m_local.x; }
        float y() const { return m_local.y; }
        float z() const { return m_local.z; }
    public:
        static UniPoint fromLocal(std::shared_ptr<HostServices> _services, const LocalPoint& _local);
        static UniPoint fromLocal(std::shared_ptr<HostServices> _services, float _x, float _y, float _z);
        static UniPoint fromGeo(std::shared_ptr<HostServices> _services, const GeoPoint& _geo);
        static UniPoint fromGeo(
            std::shared_ptr<HostServices> _services, 
            double _latitude, 
            double _longitude, 
            double _altitude);
    };

    struct Vector3d
    {
        const double latitude;
        const double longitude;
        const double altitude;
    };

    class GeoPolygon
    {
    public:
        enum class GeoEdgeType
        {
            Unknown = 0,
            ArcByEdge = 1,
            Circle = 2,
            GreatCircle = 3,
            RhumbLine = 4,
            ClockwiseArc = 5,
            CounterClockwiseArc = 6
        };
    public:
        struct GeoEdge 
        {
        public:
            const GeoEdgeType type;
            const GeoPoint fromPoint;
            const GeoPoint arcOrigin;
            const float arcDistance;
            const float arcBearing;
        };
    public:
        const std::vector<GeoEdge> edges;
    public:
        GeoPolygon(const std::vector<GeoEdge>& _edges) :
            edges(_edges)
        {
        }
    public:
        bool isEmpty() const { 
            return edges.size() == 0; 
        }
    public:
        static GeoPolygon empty()
        {
            return GeoPolygon({});
        }

        static GeoEdge circleEdge(const GeoPoint& arcOrigin, const float arcDistance)
        {
            return { GeoEdgeType::Circle, GeoPoint::empty, arcOrigin, arcDistance, 0 };
        }
    };

    class GeoMath
    {
    public:
        struct TurnData
        {
        public:
            GeoPoint e1p0;
            GeoPoint e1p1;
            double e1HeadingRad;
            GeoPoint e2p0;
            GeoPoint e2p1;
            double e2HeadingRad;
            float radius;
        };
        struct TurnArc
        {
        public:
            GeoPoint p0;
            GeoPoint p1;
            float heading0;
            float heading1;
            GeoPoint arcCenter;
            double arcStartAngle;
            double arcEndAngle;
            double arcDeltaAngle;
            double arcRadius;
            float arcLengthMeters;
            bool arcClockwise;
        };
    public:
        static double pi();
        static double twoPi();
        static double degreesToRadians(double degrees);
        static double radiansToDegrees(double degrees);
        static double headingToAngleDegrees(double headingDegrees);
        static double headingToAngleRadians(double headingDegrees);
        static double radiansToHeading(double radians);
        static GeoPoint getPointAtDistance(const GeoPoint& origin, float headingDegrees, float distanceMeters);
        static float getHeadingFromPoints(const GeoPoint& origin, const GeoPoint& destination);
        static double getRadiansFromPoints(const GeoPoint& origin, const GeoPoint& destination);
        static float getDistanceMeters(const GeoPoint& p1, const GeoPoint& p2);
        static double distanceToRadians(float distanceMeters);
        static float flipHeading(float headingDegrees);
        static void calculateTurn(const GeoMath::TurnData& input, GeoMath::TurnArc& output, std::shared_ptr<HostServices> host);
        static float getTurnDegrees(float fromHeading, float toHeading);
        static float addTurnToHeading(float heading, float turnDegrees);
        static bool isPointInRectangle(const GeoPoint& p, const GeoPoint& A, const GeoPoint& B, const GeoPoint& C, const GeoPoint& D);
        static double hypotenuse(double side);
    };

    template<class TKey>
    class HaveKey
    {
    public:
        virtual const TKey& getKey() = 0;
    };

    struct AircraftAttitude
    {
    private:
        double m_heading;
        double m_pitch;
        double m_roll;
    public:
        AircraftAttitude(double _heading, double _pitch, double _roll) :
            m_heading(_heading),
            m_pitch(_pitch),
            m_roll(_roll)
        {
        }
    public:
        const double heading() const { return m_heading; }
        const double pitch() const { return m_pitch; }
        const double roll() const { return m_roll; }
    public:
        AircraftAttitude withHeading(double newHeading) const { return AircraftAttitude(newHeading, m_pitch, m_roll); }
        AircraftAttitude withPitch(double newPitch) const { return AircraftAttitude(m_heading, newPitch, m_roll); }
        AircraftAttitude withRoll(double newRoll) const { return AircraftAttitude(m_heading, m_pitch, newRoll); }
    };

    struct Altitude
    {
    public:
        enum class Type
        {
            Ground = 0,
            AGL = 1,
            MSL = 2
        };
    private:
        float m_feet;
        Type m_type;
    private:
        Altitude(float _feet, Type _type) :
            m_feet(_feet),
            m_type(_type)
        {
        }
    public:
        const float feet() const { return m_feet; }
        Type type() const { return m_type; }
        bool isGround() const { return m_type == Type::Ground; }
        bool isGroundBased() const { return m_type == Type::Ground || m_type == Type::AGL; }
        std::string toString() const;
    public:
        static Altitude ground() { return Altitude(0, Type::Ground); }
        static Altitude agl(float feet) { return Altitude(feet, Type::AGL); }
        static Altitude msl(float feet) { return Altitude(feet, Type::MSL); }
    };

    /*
    struct Distance
    {
    private:
        double m_value;
        int m_ratio;
    public:
        Distance(double _value, int _ratio) :
            m_value(_value),
            m_ratio(_ratio)
        {
        }
    public:
        double value() { return m_value; }
        int ratio() { return m_ratio; }
    public:
        static Distance meters(double count) { return Distance(count, 10000); }
        static Distance feet(double count) { return Distance(count, 3048); }
        static Distance nauticalMiles(double count) { return Distance(count, 18520000); }
    };

    class Velocity
    { // a = (v1^2 - v0^2) / (2 s), s=distance
    private:
        Distance m_distanceUnit;
        std::chrono::microseconds m_timeUnit;
        double m_momentary;
        double m_acceleration;
        std::chrono::microseconds m_timestamp;
    public:
        Velocity(
            Distance _distanceUnit, 
            std::chrono::microseconds _timeUnit, 
            double _momentary, 
            std::chrono::microseconds _timestamp
        ) : m_distanceUnit(_distanceUnit),
            m_timeUnit(_timeUnit),
            m_momentary(_momentary),
            m_timestamp(_timestamp),
            m_acceleration(0.0)
        {
        }
    public:
        double momentary() const { return m_momentary; }
        double acceleration() const { return m_acceleration; }
        std::chrono::microseconds timestamp() const { return m_timestamp; }
    public:
        void progressTo(std::chrono::microseconds futureTimestamp);
        double calcDisplacement(std::chrono::microseconds futureTimestamp);
    public:
        static Velocity knotsAt(double value, std::chrono::microseconds timestamp) { 
            return Velocity(Distance::nauticalMiles(1), std::chrono::hours(1), value, timestamp);
        }
    };
    */

    template<class TKey, class TEntity>
    class EntityRef
    {
    private:
        friend class WorldBuilder;
    private:
        const TKey m_key;
        std::shared_ptr<TEntity> m_entity;
    public:
        EntityRef(const TKey& _key) : m_key(_key)
        {
        }
    public:
        const TKey& key() const
        {
            return m_key;
        }
        const std::shared_ptr<TEntity>& entity() const
        {
            return m_entity;
        }
        bool isResolved() const
        {
            return !!m_entity;
        }
        TEntity* operator->() const 
        {
            if (!m_entity)
            {
                throw std::runtime_error("EntityRef not resolved, cannot dereference");
            }
            return m_entity.get();
        }
    private:
        void resolve(std::shared_ptr<TEntity> _entity)
        {
            m_entity = _entity;
        }
    };


} // namespace world
