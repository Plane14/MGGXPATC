// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#include <cmath>
#include <iomanip>
#include <sstream>
#include "libworld.h"

using namespace std;

namespace world
{
    namespace
    {
        struct FlatPoint
        {
            double x = 0.0;
            double y = 0.0;
        };

        FlatPoint projectToFlatPlane(const GeoPoint& origin, const GeoPoint& point)
        {
            const double originLatitudeRad = GeoMath::degreesToRadians(origin.latitude);
            const double metersPerDegreeLatitude = 111320.0;
            const double metersPerDegreeLongitude = cos(originLatitudeRad) * metersPerDegreeLatitude;
            return {
                (point.longitude - origin.longitude) * metersPerDegreeLongitude,
                (point.latitude - origin.latitude) * metersPerDegreeLatitude
            };
        }

        vector<GeoPoint> collectPolygonVertices(const GeoPolygon& polygon)
        {
            vector<GeoPoint> vertices;
            vertices.reserve(polygon.edges.size());

            for (const auto& edge : polygon.edges)
            {
                if (edge.fromPoint == GeoPoint::empty)
                {
                    continue;
                }

                if (vertices.empty() || vertices.back() != edge.fromPoint)
                {
                    vertices.push_back(edge.fromPoint);
                }
            }

            if (vertices.size() >= 2 && vertices.front() == vertices.back())
            {
                vertices.pop_back();
            }

            return vertices;
        }

        bool pointInPolygon(const vector<GeoPoint>& vertices, const GeoPoint& point)
        {
            if (vertices.size() < 3)
            {
                return true;
            }

            bool inside = false;
            const GeoPoint origin = vertices.front();
            const FlatPoint testPoint = projectToFlatPlane(origin, point);
            const size_t vertexCount = vertices.size();

            for (size_t i = 0, j = vertexCount - 1; i < vertexCount; j = i++)
            {
                const FlatPoint vertexI = projectToFlatPlane(origin, vertices[i]);
                const FlatPoint vertexJ = projectToFlatPlane(origin, vertices[j]);
                const bool crossesScanline =
                    ((vertexI.y > testPoint.y) != (vertexJ.y > testPoint.y)) &&
                    (testPoint.x < (vertexJ.x - vertexI.x) * (testPoint.y - vertexI.y) / ((vertexJ.y - vertexI.y) + 1e-9) + vertexI.x);

                if (crossesScanline)
                {
                    inside = !inside;
                }
            }

            return inside;
        }

        bool polygonContainsLocation(const GeoPolygon& polygon, const GeoPoint& location)
        {
            if (polygon.isEmpty())
            {
                return true;
            }

            if (location == GeoPoint::empty)
            {
                return false;
            }

            if (polygon.edges.size() == 1 && polygon.edges.front().type == GeoPolygon::GeoEdgeType::Circle)
            {
                const auto& circle = polygon.edges.front();
                const double radiusMeters = circle.arcDistance * 1852.0;
                return GeoMath::getDistanceMeters(circle.arcOrigin, location) <= radiusMeters + 1.0;
            }

            return pointInPolygon(collectPolygonVertices(polygon), location);
        }

        int positionMatchScore(const shared_ptr<ControllerPosition>& position, const GeoPoint& location)
        {
            if (!position)
            {
                return 0;
            }

            if (location == GeoPoint::empty)
            {
                return 1;
            }

            const auto radarScope = position->radarScope();
            if (radarScope && !radarScope->scopeLimit().isEmpty())
            {
                return polygonContainsLocation(radarScope->scopeLimit(), location) ? 3 : 0;
            }

            const auto airspace = radarScope ? radarScope->airspace() : nullptr;
            const auto geometry = airspace ? airspace->geometry() : nullptr;
            if (geometry && !geometry->lateralBounds().isEmpty())
            {
                return polygonContainsLocation(geometry->lateralBounds(), location) ? 2 : 0;
            }

            return 1;
        }

        shared_ptr<ControllerPosition> tryFindBestPosition(
            const vector<shared_ptr<ControllerPosition>>& positions,
            ControllerPosition::Type type,
            const GeoPoint& location)
        {
            shared_ptr<ControllerPosition> bestPosition;
            int bestScore = 0;

            for (const auto& position : positions)
            {
                if (!position || position->type() != type)
                {
                    continue;
                }

                const int score = positionMatchScore(position, location);
                if (score > bestScore)
                {
                    bestScore = score;
                    bestPosition = position;
                }
            }

            return bestPosition;
        }
    }

    shared_ptr<ControllerPosition> ControlFacility::tryFindPosition(ControllerPosition::Type type, const GeoPoint& location) const
    {
        auto exactMatch = tryFindBestPosition(m_positions, type, location);

        if (exactMatch)
        {
            return exactMatch;
        }

        if (type == ControllerPosition::Type::ClearanceDelivery)
        {
            auto groundFallback = tryFindBestPosition(m_positions, ControllerPosition::Type::Ground, location);
            if (groundFallback)
            {
                return groundFallback;
            }

            auto localFallback = tryFindBestPosition(m_positions, ControllerPosition::Type::Local, location);
            if (localFallback)
            {
                return localFallback;
            }
        }

        return nullptr;
    }

    shared_ptr<ControllerPosition> ControlFacility::findPositionOrThrow(ControllerPosition::Type type, const GeoPoint& location) const 
    {
        auto positionOrNull = tryFindPosition(type, location);
        if (positionOrNull)
        {
            return positionOrNull;
        }

        stringstream errorMessage;
        errorMessage << setprecision(11)
                     << "Controller position type ["
                     << (int)type
                     << "] not found at facility ["
                     << m_callSign
                     << "] for location (" << location.latitude << "," << location.longitude << "). "
                     << m_positions.size() << " position(s) exist: ";
        for (const auto position : m_positions)
        {
            errorMessage << "[type=" << (int)position->type()
                         << "|" << position->frequency()->khz() << "|"
                         << position->callSign() << "] ";
        }

        throw runtime_error(errorMessage.str());
    }

    void ControlFacility::progressTo(chrono::microseconds timestamp)
    {
        for (const auto position : m_positions)
        {
            position->progressTo(timestamp);
        }
    }

    void ControlFacility::clearFlights()
    {
        for (const auto& position : m_positions)
        {
            position->clearFlights();
        }
    }

    // shared_ptr<ControllerPosition> ControlFacility::tryFindPosition(
    //     ControllerPosition::Type type, 
    //     const GeoPoint& location) const
    // {

    // }
}