// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <iostream>
#include <cctype>
#include <utility>
#include "stlhelpers.h"
#include "libworld.h"
#include "libdataxp.h"

using namespace world;
using namespace std;

static const unordered_map<string, ParkingStand::Type> parkingStandTypeLookup = {
    {"gate", ParkingStand::Type::Gate},
    {"hangar", ParkingStand::Type::Hangar},
    {"tie_down", ParkingStand::Type::Remote},
    {"misc", ParkingStand::Type::Unknown},
};

static const unordered_map<string, Aircraft::Category> aircraftCategoryLookup = {
    {"heavy", Aircraft::Category::Heavy},
    {"jets", Aircraft::Category::Jet},
    {"turboprops", Aircraft::Category::Turboprop},
    {"props", Aircraft::Category::Prop},
    {"helos", Aircraft::Category::Helicopter},
    {"fighters", Aircraft::Category::Fighter},
    {"all", Aircraft::Category::All},
};

static const unordered_map<string, Aircraft::OperationType> aircraftOperationTypeLookup = {
    {"none", Aircraft::OperationType::None},
    {"general_aviation", Aircraft::OperationType::GA},
    {"airline", Aircraft::OperationType::Airline},
    {"cargo", Aircraft::OperationType::Cargo},
    {"military", Aircraft::OperationType::Military},
};

static constexpr int DATUM_UNSPECIFIED = -10000;

static void parseSeparatedList(
    const string& listText, 
    const string& delimiters, 
    function<void(const string& item)> parseItem)
{
    int lastDelimiterIndex = -1;
    
    for (int i = 0 ; i < listText.length(); i++)
    {
        char c = listText[i];
        bool isDelimiter = (delimiters.find(c) != string::npos);
        if (isDelimiter)
        {
            if (i > lastDelimiterIndex + 1)
            {
                string itemText = listText.substr(lastDelimiterIndex + 1, i - lastDelimiterIndex - 1);
                parseItem(itemText);
            }
            lastDelimiterIndex = i;
        }
    }

    if (lastDelimiterIndex < (int)listText.length() - 1)
    {
        string itemText = listText.substr(lastDelimiterIndex + 1, listText.length() - lastDelimiterIndex - 1);
        parseItem(itemText);
    }
}

static string trimCopy(const string& value)
{
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == string::npos)
    {
        return "";
    }

    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

static string toUpperCopy(string value);

static void assignTaxiNodeUsage(shared_ptr<TaxiNode> node, const string& usage)
{
    const auto usageType = toUpperCopy(trimCopy(usage));

    if (usageType == "BOTH")
    {
        node->setRouteStart(true);
        node->setRouteEnd(true);
    }
    else if (usageType == "INIT" || usageType == "START")
    {
        node->setRouteStart(true);
    }
    else if (usageType == "END" || usageType == "DEST" || usageType == "DESTINATION")
    {
        node->setRouteEnd(true);
    }
    else if (usageType == "JUNC" || usageType == "JUNCTION")
    {
        node->setJunction(true);
    }
}

struct AtcPolygonDefinition
{
    bool hasLowerBound = false;
    float lowerBoundFeet = 0.0f;
    bool hasUpperBound = false;
    float upperBoundFeet = 0.0f;
    vector<GeoPoint> points;
};

struct AtcControllerBlock
{
    string name;
    string facilityId;
    string icao;
    string role;
    string airspaceClass;
    vector<int> frequenciesKhz;
    vector<AtcPolygonDefinition> polygons;
};

struct AtcNavCache
{
    unordered_map<string, vector<AtcControllerBlock>> blocksByAirport;
};

struct AtcRoleAggregate
{
    bool present = false;
    string role;
    string airspaceClass;
    vector<int> frequenciesKhz;
    vector<AtcPolygonDefinition> polygons;
};

struct NamedAirspaceRecord
{
    string categoryCode;
    string name;
    string normalizedName;
    string canonicalName;
    string canonicalNormalizedName;
    bool hasLowerBound = false;
    float lowerBoundFeet = 0.0f;
    bool hasUpperBound = false;
    float upperBoundFeet = 0.0f;
    shared_ptr<GeoPolygon> polygon;
};

struct AirspaceTxtCache
{
    vector<NamedAirspaceRecord> records;
    unordered_map<string, vector<size_t>> recordIndicesByToken;
    unordered_map<string, vector<size_t>> recordIndicesByExactName;
};

struct NamedAirspaceMatch
{
    string displayName;
    bool hasLowerBound = false;
    float lowerBoundFeet = 0.0f;
    bool hasUpperBound = false;
    float upperBoundFeet = 0.0f;
    const AirspaceClass* classification = nullptr;
};

struct ProjectedGeoPoint
{
    double x = 0.0;
    double y = 0.0;
    GeoPoint geo = GeoPoint::empty;
};

struct FlatGeoPoint
{
    double x = 0.0;
    double y = 0.0;
};

static string toUpperCopy(string value)
{
    transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(toupper(c));
    });
    return value;
}

static bool equalsIgnoreCase(const string& left, const string& right)
{
    return toUpperCopy(left) == toUpperCopy(right);
}

static bool tokenListContains(const string& listText, const string& token)
{
    const string expected = toUpperCopy(token);
    bool found = false;
    parseSeparatedList(listText, ",;:| \t", [&found, &expected](const string& item) {
        if (toUpperCopy(trimCopy(item)) == expected)
        {
            found = true;
        }
    });
    return found;
}

static vector<string> splitTokens(const string& lineText)
{
    vector<string> tokens;
    parseSeparatedList(lineText, " \t", [&tokens](const string& item) {
        const string trimmed = trimCopy(item);
        if (!trimmed.empty())
        {
            tokens.push_back(trimmed);
        }
    });
    return tokens;
}

static void appendUniqueText(vector<string>& items, const string& value)
{
    if (!value.empty() && find(items.begin(), items.end(), value) == items.end())
    {
        items.push_back(value);
    }
}

static string normalizeLookupText(const string& value)
{
    string normalized;
    normalized.reserve(value.size());

    bool previousWasSpace = true;
    for (const auto ch : value)
    {
        if (isalnum(static_cast<unsigned char>(ch)))
        {
            normalized.push_back(static_cast<char>(toupper(static_cast<unsigned char>(ch))));
            previousWasSpace = false;
        }
        else if (!previousWasSpace)
        {
            normalized.push_back(' ');
            previousWasSpace = true;
        }
    }

    while (!normalized.empty() && normalized.back() == ' ')
    {
        normalized.pop_back();
    }

    return normalized;
}

static bool hasCtrMarker(const string& normalizedName);
static bool hasTerminalMarker(const string& normalizedName);
static bool hasControlAreaMarker(const string& normalizedName);
static bool hasFirMarker(const string& normalizedName);
static bool hasOceanicMarker(const string& normalizedName);

static string canonicalizeAirspaceName(const string& name)
{
    const string trimmed = trimCopy(name);
    const string upper = toUpperCopy(trimmed);

    size_t cutIndex = string::npos;
    const auto considerCut = [&](const string& marker) {
        const auto markerIndex = upper.find(marker);
        if (markerIndex == string::npos)
        {
            return;
        }

        const string prefixNormalized = normalizeLookupText(trimmed.substr(0, markerIndex));
        const bool prefixLooksTypedAirspace =
            hasCtrMarker(prefixNormalized)
            || hasTerminalMarker(prefixNormalized)
            || hasControlAreaMarker(prefixNormalized)
            || hasFirMarker(prefixNormalized)
            || hasOceanicMarker(prefixNormalized);

        if (prefixLooksTypedAirspace && (cutIndex == string::npos || markerIndex < cutIndex))
        {
            cutIndex = markerIndex;
        }
    };

    considerCut(" AREA ");
    considerCut(" SECTOR ");

    return cutIndex == string::npos
        ? trimmed
        : trimCopy(trimmed.substr(0, cutIndex));
}

static bool hasAreaOrSectorDecoration(const string& name)
{
    const string upper = toUpperCopy(trimCopy(name));
    return upper.find(" AREA ") != string::npos || upper.find(" SECTOR ") != string::npos;
}

static bool containsLookupPhrase(const string& normalizedText, const string& normalizedPhrase)
{
    if (normalizedText.empty() || normalizedPhrase.empty())
    {
        return false;
    }

    const string paddedText = " " + normalizedText + " ";
    const string paddedPhrase = " " + normalizedPhrase + " ";
    return paddedText.find(paddedPhrase) != string::npos;
}

static bool hasCtrMarker(const string& normalizedName)
{
    return containsLookupPhrase(normalizedName, "CTR");
}

static bool hasTerminalMarker(const string& normalizedName)
{
    return containsLookupPhrase(normalizedName, "TMA")
        || containsLookupPhrase(normalizedName, "TMAD");
}

static bool hasControlAreaMarker(const string& normalizedName)
{
    return containsLookupPhrase(normalizedName, "CTA");
}

static bool hasFirMarker(const string& normalizedName)
{
    return containsLookupPhrase(normalizedName, "FIR");
}

static bool hasOceanicMarker(const string& normalizedName)
{
    return containsLookupPhrase(normalizedName, "OCEANIC")
        || containsLookupPhrase(normalizedName, "OCA");
}

static bool isLookupNoiseToken(const string& token)
{
    static const unordered_set<string> s_noiseTokens = {
        "A", "AN", "AC", "CTR", "TMA", "TMAD", "CTA", "FIR", "OCEANIC", "OCA", "AREA", "SECTOR", "CLASS", "AIRSPACE"
    };

    return token.size() <= 1 || s_noiseTokens.find(token) != s_noiseTokens.end();
}

static vector<string> extractLookupTokens(const string& value)
{
    vector<string> tokens;
    const string normalized = normalizeLookupText(value);
    string token;
    stringstream parser(normalized);
    while (parser >> token)
    {
        if (!isLookupNoiseToken(token))
        {
            appendUniqueText(tokens, token);
        }
    }

    return tokens;
}

static bool tryParseAirspaceAltitudeFeet(const string& text, bool& hasBound, float& boundFeet)
{
    hasBound = false;
    boundFeet = 0.0f;

    const string normalized = normalizeLookupText(text);
    if (normalized.empty())
    {
        return false;
    }

    if (normalized == "GND" || normalized == "SFC" || normalized == "SURFACE")
    {
        hasBound = true;
        boundFeet = 0.0f;
        return true;
    }

    string firstToken;
    stringstream parser(normalized);
    parser >> firstToken;
    if (firstToken.empty())
    {
        return false;
    }

    if (firstToken == "UNLTD" || firstToken == "UNLIMITED")
    {
        return true;
    }

    try
    {
        if (stringStartsWith(firstToken, "FL") && firstToken.length() > 2)
        {
            hasBound = true;
            boundFeet = stof(firstToken.substr(2)) * 100.0f;
            return true;
        }

        hasBound = true;
        boundFeet = stof(firstToken);
        return true;
    }
    catch (const exception&)
    {
        return false;
    }
}

static bool tryParseDmsCoordinate(const string& dmsText, char hemisphere, double& value)
{
    vector<string> parts;
    string current;

    for (const auto ch : dmsText)
    {
        if (ch == ':')
        {
            parts.push_back(current);
            current.clear();
        }
        else
        {
            current.push_back(ch);
        }
    }

    parts.push_back(current);
    if (parts.size() != 3)
    {
        return false;
    }

    try
    {
        const double degrees = stod(parts.at(0));
        const double minutes = stod(parts.at(1));
        const double seconds = stod(parts.at(2));

        value = degrees + minutes / 60.0 + seconds / 3600.0;
        const char upperHemisphere = static_cast<char>(toupper(static_cast<unsigned char>(hemisphere)));
        if (upperHemisphere == 'S' || upperHemisphere == 'W')
        {
            value *= -1.0;
        }

        return true;
    }
    catch (const exception&)
    {
        return false;
    }
}

static bool tryParseAirspacePoint(const string& pointText, GeoPoint& point)
{
    string latitudeDms;
    string latitudeHemisphere;
    string longitudeDms;
    string longitudeHemisphere;
    stringstream parser(pointText);

    if (!(parser >> latitudeDms >> latitudeHemisphere >> longitudeDms >> longitudeHemisphere)
        || latitudeHemisphere.empty()
        || longitudeHemisphere.empty())
    {
        return false;
    }

    double latitude = 0.0;
    double longitude = 0.0;
    if (!tryParseDmsCoordinate(latitudeDms, latitudeHemisphere.front(), latitude)
        || !tryParseDmsCoordinate(longitudeDms, longitudeHemisphere.front(), longitude))
    {
        return false;
    }

    point = GeoPoint(latitude, longitude);
    point.altitude = 0.0;
    return true;
}

static FlatGeoPoint projectToFlatPlane(const GeoPoint& origin, const GeoPoint& point)
{
    const double originLatitudeRad = GeoMath::degreesToRadians(origin.latitude);
    const double metersPerDegreeLatitude = 111320.0;
    const double metersPerDegreeLongitude = cos(originLatitudeRad) * metersPerDegreeLatitude;
    return {
        (point.longitude - origin.longitude) * metersPerDegreeLongitude,
        (point.latitude - origin.latitude) * metersPerDegreeLatitude
    };
}

static vector<GeoPoint> collectPolygonVertices(const GeoPolygon& polygon)
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

static bool pointInPolygon(const vector<GeoPoint>& vertices, const GeoPoint& point)
{
    if (vertices.size() < 3)
    {
        return true;
    }

    bool inside = false;
    const GeoPoint origin = vertices.front();
    const FlatGeoPoint testPoint = projectToFlatPlane(origin, point);

    for (size_t i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++)
    {
        const FlatGeoPoint vertexI = projectToFlatPlane(origin, vertices.at(i));
        const FlatGeoPoint vertexJ = projectToFlatPlane(origin, vertices.at(j));
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

static bool polygonContainsLocation(const GeoPolygon& polygon, const GeoPoint& location)
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

static string preferredAirspaceSuffix(ControlledAirspace::Type type)
{
    switch (type)
    {
    case ControlledAirspace::Type::ControlZone:
        return "CTR";
    case ControlledAirspace::Type::TerminalControlArea:
        return "TMA";
    case ControlledAirspace::Type::ControlArea:
        return "CTA";
    case ControlledAirspace::Type::AreaFIR:
    case ControlledAirspace::Type::OceanicFIR:
        return "FIR";
    default:
        return "";
    }
}

static bool isCompatibleNamedAirspaceRecord(
    const NamedAirspaceRecord& record,
    ControlledAirspace::Type baseType)
{
    const string& normalizedName = record.canonicalNormalizedName;
    switch (baseType)
    {
    case ControlledAirspace::Type::ControlZone:
        return !hasTerminalMarker(normalizedName)
            && !hasControlAreaMarker(normalizedName)
            && !hasFirMarker(normalizedName)
            && !hasOceanicMarker(normalizedName);

    case ControlledAirspace::Type::TerminalControlArea:
        return !hasCtrMarker(normalizedName)
            && !hasFirMarker(normalizedName)
            && !hasOceanicMarker(normalizedName);

    case ControlledAirspace::Type::ControlArea:
        return !hasCtrMarker(normalizedName)
            && !hasFirMarker(normalizedName)
            && !hasOceanicMarker(normalizedName);

    case ControlledAirspace::Type::AreaFIR:
    case ControlledAirspace::Type::OceanicFIR:
        return !hasCtrMarker(normalizedName)
            && !hasTerminalMarker(normalizedName)
            && !hasControlAreaMarker(normalizedName);

    default:
        return true;
    }
}

static int scoreAirspaceAliasMatch(
    const string& canonicalNormalizedName,
    const vector<string>& aliases,
    ControlledAirspace::Type baseType)
{
    int bestScore = 0;
    const string preferredSuffix = preferredAirspaceSuffix(baseType);

    for (const auto& alias : aliases)
    {
        if (alias.empty())
        {
            continue;
        }

        if (!preferredSuffix.empty() && canonicalNormalizedName == alias + " " + preferredSuffix)
        {
            bestScore = max(bestScore, 420);
        }
        if (canonicalNormalizedName == alias)
        {
            bestScore = max(bestScore, 380);
        }
        if (stringStartsWith(canonicalNormalizedName, alias + " "))
        {
            bestScore = max(bestScore, 320);
        }
        if (stringStartsWith(canonicalNormalizedName, alias))
        {
            bestScore = max(bestScore, 260);
        }
        if (containsLookupPhrase(canonicalNormalizedName, alias))
        {
            bestScore = max(bestScore, 220);
        }
    }

    return bestScore;
}

static const AirspaceClass& toAirspaceClass(
    const string& airspaceClassText,
    ControlledAirspace::Type fallbackType);

static const AirspaceClass& resolveNamedAirspaceClass(
    const NamedAirspaceRecord& record,
    ControlledAirspace::Type fallbackType,
    const AirspaceClass& fallbackClass)
{
    const string category = toUpperCopy(trimCopy(record.categoryCode));
    if (category == "A")
    {
        return AirspaceClass::ClassA;
    }
    if (category == "B")
    {
        return AirspaceClass::ClassB;
    }
    if (category == "C")
    {
        return AirspaceClass::ClassC;
    }
    if (category == "D")
    {
        return AirspaceClass::ClassD;
    }
    if (category == "E")
    {
        return AirspaceClass::ClassE;
    }
    if (category == "G")
    {
        return AirspaceClass::ClassG;
    }

    if (!category.empty() && category != "CTR")
    {
        return toAirspaceClass(category, fallbackType);
    }

    return fallbackClass;
}

static int normalizeAtcFrequencyKhz(int rawFrequency)
{
    return rawFrequency < 100000
        ? rawFrequency * 10
        : rawFrequency;
}

static bool hasControllerPositionType(
    const vector<ControllerPosition::Structure>& positions,
    ControllerPosition::Type type)
{
    return any_of(positions.begin(), positions.end(), [type](const ControllerPosition::Structure& position) {
        return position.type == type;
    });
}

static vector<GeoPoint> normalizePolygonPoints(const vector<GeoPoint>& points)
{
    vector<GeoPoint> normalized;
    normalized.reserve(points.size());

    for (const auto& point : points)
    {
        if (point == GeoPoint::empty)
        {
            continue;
        }

        if (normalized.empty() || normalized.back() != point)
        {
            normalized.push_back(point);
        }
    }

    if (normalized.size() >= 2 && normalized.front() == normalized.back())
    {
        normalized.pop_back();
    }

    return normalized;
}

static GeoPolygon buildPolygonFromPoints(const vector<GeoPoint>& points)
{
    const auto normalized = normalizePolygonPoints(points);
    if (normalized.size() < 3)
    {
        return GeoPolygon::empty();
    }

    vector<GeoPolygon::GeoEdge> edges;
    edges.reserve(normalized.size());

    for (const auto& point : normalized)
    {
        edges.push_back({
            GeoPolygon::GeoEdgeType::GreatCircle,
            point,
            GeoPoint::empty,
            0.0f,
            0.0f
        });
    }

    return GeoPolygon(edges);
}

static vector<GeoPoint> buildConvexHull(vector<GeoPoint> points)
{
    points = normalizePolygonPoints(points);
    sort(points.begin(), points.end(), [](const GeoPoint& left, const GeoPoint& right) {
        return left.longitude < right.longitude
            || (left.longitude == right.longitude && left.latitude < right.latitude);
    });
    points.erase(unique(points.begin(), points.end(), [](const GeoPoint& left, const GeoPoint& right) {
        return left == right;
    }), points.end());

    if (points.size() <= 3)
    {
        return points;
    }

    double averageLatitude = 0.0;
    for (const auto& point : points)
    {
        averageLatitude += point.latitude;
    }
    averageLatitude /= static_cast<double>(points.size());

    double longitudeScale = cos(GeoMath::degreesToRadians(averageLatitude));
    if (fabs(longitudeScale) < 1e-6)
    {
        longitudeScale = 1.0;
    }

    vector<ProjectedGeoPoint> projected;
    projected.reserve(points.size());
    for (const auto& point : points)
    {
        projected.push_back({
            point.longitude * longitudeScale,
            point.latitude,
            point
        });
    }

    const auto cross = [](const ProjectedGeoPoint& origin, const ProjectedGeoPoint& a, const ProjectedGeoPoint& b) {
        return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
    };

    vector<ProjectedGeoPoint> hull(projected.size() * 2);
    size_t hullSize = 0;

    for (const auto& point : projected)
    {
        while (hullSize >= 2 && cross(hull[hullSize - 2], hull[hullSize - 1], point) <= 0.0)
        {
            hullSize--;
        }

        hull[hullSize++] = point;
    }

    for (size_t index = projected.size() - 1, lowerSize = hullSize + 1; index > 0; --index)
    {
        const auto& point = projected[index - 1];
        while (hullSize >= lowerSize && cross(hull[hullSize - 2], hull[hullSize - 1], point) <= 0.0)
        {
            hullSize--;
        }

        hull[hullSize++] = point;
    }

    if (hullSize > 1)
    {
        hullSize--;
    }

    vector<GeoPoint> result;
    result.reserve(hullSize);
    for (size_t index = 0; index < hullSize; ++index)
    {
        result.push_back(hull[index].geo);
    }

    if (result.size() < 3)
    {
        return points;
    }

    return result;
}

static GeoPolygon buildMergedScopePolygon(const vector<AtcPolygonDefinition>& polygons)
{
    vector<GeoPoint> mergedPoints;
    size_t nonEmptyPolygonCount = 0;

    for (const auto& polygon : polygons)
    {
        const auto normalizedPoints = normalizePolygonPoints(polygon.points);
        if (normalizedPoints.size() < 3)
        {
            continue;
        }

        nonEmptyPolygonCount++;
        if (polygons.size() == 1)
        {
            return buildPolygonFromPoints(normalizedPoints);
        }

        mergedPoints.insert(mergedPoints.end(), normalizedPoints.begin(), normalizedPoints.end());
    }

    if (nonEmptyPolygonCount == 0)
    {
        return GeoPolygon::empty();
    }

    if (nonEmptyPolygonCount == 1)
    {
        for (const auto& polygon : polygons)
        {
            const auto normalizedPoints = normalizePolygonPoints(polygon.points);
            if (normalizedPoints.size() >= 3)
            {
                return buildPolygonFromPoints(normalizedPoints);
            }
        }
    }

    return buildPolygonFromPoints(buildConvexHull(mergedPoints));
}

static void appendUniqueFrequency(vector<int>& frequenciesKhz, int frequencyKhz)
{
    if (frequencyKhz <= 0)
    {
        return;
    }

    if (find(frequenciesKhz.begin(), frequenciesKhz.end(), frequencyKhz) == frequenciesKhz.end())
    {
        frequenciesKhz.push_back(frequencyKhz);
    }
}

static vector<string> getAirportKeys(const AtcControllerBlock& block)
{
    vector<string> keys;
    const auto appendKey = [&](const string& keyText) {
        const string key = toUpperCopy(trimCopy(keyText));
        if (!key.empty() && find(keys.begin(), keys.end(), key) == keys.end())
        {
            keys.push_back(key);
        }
    };

    appendKey(block.facilityId);
    appendKey(block.icao);
    return keys;
}

static void mergeRoleAggregate(AtcRoleAggregate& aggregate, const AtcControllerBlock& block)
{
    aggregate.present = true;
    if (aggregate.role.empty())
    {
        aggregate.role = block.role;
    }
    if (aggregate.airspaceClass.empty() && !block.airspaceClass.empty())
    {
        aggregate.airspaceClass = block.airspaceClass;
    }

    for (const auto frequencyKhz : block.frequenciesKhz)
    {
        appendUniqueFrequency(aggregate.frequenciesKhz, frequencyKhz);
    }

    for (const auto& polygon : block.polygons)
    {
        if (!polygon.points.empty())
        {
            aggregate.polygons.push_back(polygon);
        }
    }
}

static void collectAtcRoleAggregates(
    const vector<AtcControllerBlock>& blocks,
    AtcRoleAggregate& towerAggregate,
    AtcRoleAggregate& terminalAggregate,
    AtcRoleAggregate& approachAggregate,
    AtcRoleAggregate& departureAggregate,
    AtcRoleAggregate& centerAggregate)
{
    for (const auto& block : blocks)
    {
        const string role = toUpperCopy(block.role);
        if (role == "TWR")
        {
            mergeRoleAggregate(towerAggregate, block);
        }
        else if (role == "TRACON")
        {
            mergeRoleAggregate(terminalAggregate, block);
            mergeRoleAggregate(approachAggregate, block);
            mergeRoleAggregate(departureAggregate, block);
        }
        else if (role == "APP" || role == "APPROACH")
        {
            mergeRoleAggregate(terminalAggregate, block);
            mergeRoleAggregate(approachAggregate, block);
        }
        else if (role == "DEP" || role == "DEPARTURE")
        {
            mergeRoleAggregate(terminalAggregate, block);
            mergeRoleAggregate(departureAggregate, block);
        }
        else if (role == "CTR")
        {
            mergeRoleAggregate(centerAggregate, block);
        }
    }
}

static int selectAtcFrequency(const vector<int>& frequenciesKhz, size_t preferredIndex = 0)
{
    if (frequenciesKhz.empty())
    {
        return 0;
    }

    return frequenciesKhz.at(min(preferredIndex, frequenciesKhz.size() - 1));
}

static const AirspaceClass& toAirspaceClass(
    const string& airspaceClassText,
    ControlledAirspace::Type fallbackType)
{
    const string classification = toUpperCopy(trimCopy(airspaceClassText));
    if (classification == "A")
    {
        return AirspaceClass::ClassA;
    }
    if (classification == "B")
    {
        return AirspaceClass::ClassB;
    }
    if (classification == "C")
    {
        return AirspaceClass::ClassC;
    }
    if (classification == "D")
    {
        return AirspaceClass::ClassD;
    }
    if (classification == "E")
    {
        return AirspaceClass::ClassE;
    }

    switch (fallbackType)
    {
    case ControlledAirspace::Type::TerminalControlArea:
        return AirspaceClass::ClassC;
    case ControlledAirspace::Type::ControlZone:
        return AirspaceClass::ClassD;
    case ControlledAirspace::Type::AreaFIR:
    case ControlledAirspace::Type::ControlArea:
        return AirspaceClass::ClassE;
    default:
        return AirspaceClass::ClassG;
    }
}

static void appendPolygons(
    const vector<AtcPolygonDefinition>& source,
    vector<AtcPolygonDefinition>& target)
{
    target.insert(target.end(), source.begin(), source.end());
}

static shared_ptr<AtcNavCache> getAtcNavCache(shared_ptr<HostServices> host)
{
    static unordered_map<string, shared_ptr<AtcNavCache>> s_cacheByPath;

    const string atcDataPath = host->getHostFilePath({ "Custom Data", "1200 atc data", "Earth nav data", "atc.dat" });
    const auto found = s_cacheByPath.find(atcDataPath);
    if (found != s_cacheByPath.end())
    {
        return found->second;
    }

    auto cache = make_shared<AtcNavCache>();

    try
    {
        auto input = host->openFileForRead(atcDataPath);

        string line;
        bool inControllerBlock = false;
        bool inAirspacePolygon = false;
        AtcControllerBlock block;
        AtcPolygonDefinition polygon;

        const auto flushPolygon = [&]() {
            polygon.points = normalizePolygonPoints(polygon.points);
            if (polygon.points.size() >= 3)
            {
                block.polygons.push_back(polygon);
            }

            polygon = AtcPolygonDefinition();
        };

        const auto flushBlock = [&]() {
            if (inAirspacePolygon)
            {
                flushPolygon();
                inAirspacePolygon = false;
            }

            if (!block.role.empty())
            {
                for (const auto& key : getAirportKeys(block))
                {
                    cache->blocksByAirport[key].push_back(block);
                }
            }

            block = AtcControllerBlock();
        };

        while (getline(*input, line))
        {
            const string trimmed = trimCopy(line);
            if (trimmed.empty())
            {
                continue;
            }

            if (trimmed == "CONTROLLER")
            {
                if (inControllerBlock)
                {
                    flushBlock();
                }

                inControllerBlock = true;
                block = AtcControllerBlock();
                continue;
            }

            if (!inControllerBlock)
            {
                continue;
            }

            if (trimmed == "CONTROLLER_END")
            {
                flushBlock();
                inControllerBlock = false;
                continue;
            }

            if (trimmed == "AIRSPACE_POLYGON_END")
            {
                if (inAirspacePolygon)
                {
                    flushPolygon();
                    inAirspacePolygon = false;
                }
                continue;
            }

            if (stringStartsWith(trimmed, "AIRSPACE_POLYGON_BEGIN"))
            {
                if (inAirspacePolygon)
                {
                    flushPolygon();
                }

                polygon = AtcPolygonDefinition();
                inAirspacePolygon = true;

                stringstream parser(trimmed.substr(23));
                float lowerBoundFeet = 0.0f;
                float upperBoundFeet = 0.0f;
                if (parser >> lowerBoundFeet >> upperBoundFeet)
                {
                    polygon.hasLowerBound = true;
                    polygon.lowerBoundFeet = lowerBoundFeet;
                    polygon.hasUpperBound = true;
                    polygon.upperBoundFeet = upperBoundFeet;
                }

                continue;
            }

            if (inAirspacePolygon && stringStartsWith(trimmed, "POINT "))
            {
                GeoPoint point = GeoPoint::empty;
                stringstream parser(trimmed.substr(6));
                if (parser >> point.latitude >> point.longitude)
                {
                    point.altitude = 0.0;
                    polygon.points.push_back(point);
                }
                continue;
            }

            if (stringStartsWith(trimmed, "NAME "))
            {
                block.name = trimCopy(trimmed.substr(5));
                continue;
            }

            if (stringStartsWith(trimmed, "FACILITY_ID "))
            {
                block.facilityId = trimCopy(trimmed.substr(12));
                continue;
            }

            if (stringStartsWith(trimmed, "ICAO "))
            {
                block.icao = trimCopy(trimmed.substr(5));
                continue;
            }

            if (stringStartsWith(trimmed, "ROLE "))
            {
                block.role = trimCopy(trimmed.substr(5));
                continue;
            }

            if (stringStartsWith(trimmed, "CLASS "))
            {
                block.airspaceClass = trimCopy(trimmed.substr(6));
                continue;
            }

            if (stringStartsWith(trimmed, "FREQ ") || stringStartsWith(trimmed, "CHAN "))
            {
                const size_t valueStart = trimmed.find(' ') + 1;
                const string valueText = trimCopy(trimmed.substr(valueStart));
                try
                {
                    appendUniqueFrequency(block.frequenciesKhz, normalizeAtcFrequencyKhz(stoi(valueText)));
                }
                catch (const exception&)
                {
                }
            }
        }

        if (inControllerBlock)
        {
            flushBlock();
        }

        host->writeLog(
            "ATCNAV|Loaded controller sector cache for [%d] airport(s)",
            static_cast<int>(cache->blocksByAirport.size()));
    }
    catch (const exception& e)
    {
        host->writeLog(
            "ATCNAV|Failed to build controller sector cache from [%s]: %s",
            atcDataPath.c_str(),
            e.what());
    }

    s_cacheByPath.emplace(atcDataPath, cache);
    return cache;
}

static shared_ptr<AirspaceTxtCache> getAirspaceTxtCache(shared_ptr<HostServices> host)
{
    static unordered_map<string, shared_ptr<AirspaceTxtCache>> s_cacheByPath;

    const string airspaceDataPath = host->getHostFilePath({ "Custom Data", "airspaces", "airspace.txt" });
    const auto found = s_cacheByPath.find(airspaceDataPath);
    if (found != s_cacheByPath.end())
    {
        return found->second;
    }

    auto cache = make_shared<AirspaceTxtCache>();

    try
    {
        auto input = host->openFileForRead(airspaceDataPath);
        if (input)
        {
            string line;
            bool inRecord = false;
            NamedAirspaceRecord record;
            vector<GeoPoint> polygonPoints;

            const auto flushRecord = [&]() {
                if (!inRecord)
                {
                    return;
                }

                const GeoPolygon polygon = buildPolygonFromPoints(polygonPoints);
                record.normalizedName = normalizeLookupText(record.name);
                record.canonicalName = canonicalizeAirspaceName(record.name);
                record.canonicalNormalizedName = normalizeLookupText(record.canonicalName);

                if (!record.canonicalNormalizedName.empty() && !polygon.isEmpty())
                {
                    record.polygon = make_shared<GeoPolygon>(polygon);
                    const auto index = cache->records.size();
                    cache->records.push_back(record);
                    cache->recordIndicesByExactName[record.canonicalNormalizedName].push_back(index);

                    for (const auto& token : extractLookupTokens(record.canonicalNormalizedName))
                    {
                        cache->recordIndicesByToken[token].push_back(index);
                    }
                }

                record = NamedAirspaceRecord();
                polygonPoints.clear();
                inRecord = false;
            };

            while (getline(*input, line))
            {
                const string trimmed = trimCopy(line);
                if (trimmed.empty())
                {
                    continue;
                }

                if (stringStartsWith(trimmed, "AC "))
                {
                    flushRecord();
                    inRecord = true;
                    record = NamedAirspaceRecord();
                    record.categoryCode = trimCopy(trimmed.substr(3));
                    continue;
                }

                if (!inRecord)
                {
                    continue;
                }

                if (stringStartsWith(trimmed, "AN "))
                {
                    record.name = trimCopy(trimmed.substr(3));
                    continue;
                }

                if (stringStartsWith(trimmed, "AL "))
                {
                    tryParseAirspaceAltitudeFeet(trimCopy(trimmed.substr(3)), record.hasLowerBound, record.lowerBoundFeet);
                    continue;
                }

                if (stringStartsWith(trimmed, "AH "))
                {
                    tryParseAirspaceAltitudeFeet(trimCopy(trimmed.substr(3)), record.hasUpperBound, record.upperBoundFeet);
                    continue;
                }

                if (stringStartsWith(trimmed, "DP "))
                {
                    GeoPoint point = GeoPoint::empty;
                    if (tryParseAirspacePoint(trimCopy(trimmed.substr(3)), point))
                    {
                        polygonPoints.push_back(point);
                    }
                    continue;
                }
            }

            flushRecord();

            host->writeLog(
                "ATCNAV|Loaded named airspace cache for [%d] entry(s)",
                static_cast<int>(cache->records.size()));
        }
    }
    catch (const exception& e)
    {
        host->writeLog(
            "ATCNAV|Failed to build named airspace cache from [%s]: %s",
            airspaceDataPath.c_str(),
            e.what());
    }

    s_cacheByPath.emplace(airspaceDataPath, cache);
    return cache;
}

static bool tryResolveNamedAirspaceMatch(
    shared_ptr<HostServices> host,
    const Airport::Header& header,
    ControlledAirspace::Type baseType,
    const AirspaceClass& baseClassification,
    NamedAirspaceMatch& match)
{
    const auto airspaceCache = getAirspaceTxtCache(host);
    if (!airspaceCache || airspaceCache->records.empty())
    {
        return false;
    }

    vector<string> aliases;
    const auto addAlias = [&](const string& text) {
        appendUniqueText(aliases, normalizeLookupText(canonicalizeAirspaceName(text)));
    };

    addAlias(header.name());

    const auto atcCache = getAtcNavCache(host);
    if (atcCache)
    {
        const auto blocksFound = atcCache->blocksByAirport.find(toUpperCopy(header.icao()));
        if (blocksFound != atcCache->blocksByAirport.end())
        {
            for (const auto& block : blocksFound->second)
            {
                addAlias(block.name);
            }
        }
    }

    if (aliases.empty())
    {
        return false;
    }

    unordered_set<size_t> candidateIndices;
    for (const auto& alias : aliases)
    {
        const auto exactFound = airspaceCache->recordIndicesByExactName.find(alias);
        if (exactFound != airspaceCache->recordIndicesByExactName.end())
        {
            candidateIndices.insert(exactFound->second.begin(), exactFound->second.end());
        }

        for (const auto& token : extractLookupTokens(alias))
        {
            const auto tokenFound = airspaceCache->recordIndicesByToken.find(token);
            if (tokenFound != airspaceCache->recordIndicesByToken.end())
            {
                candidateIndices.insert(tokenFound->second.begin(), tokenFound->second.end());
            }
        }
    }

    if (candidateIndices.empty())
    {
        return false;
    }

    bool foundMatch = false;
    size_t bestIndex = 0;
    int bestScore = numeric_limits<int>::min();
    float bestLowerBound = numeric_limits<float>::max();
    float bestUpperBound = -numeric_limits<float>::max();
    size_t bestNameLength = numeric_limits<size_t>::max();

    for (const auto candidateIndex : candidateIndices)
    {
        const auto& record = airspaceCache->records.at(candidateIndex);
        const int aliasScore = scoreAirspaceAliasMatch(record.canonicalNormalizedName, aliases, baseType);
        if (aliasScore <= 0)
        {
            continue;
        }

        if (!isCompatibleNamedAirspaceRecord(record, baseType)
            || !record.polygon
            || !polygonContainsLocation(*record.polygon, header.datum()))
        {
            continue;
        }

        int score = 1000 + aliasScore;
        const string preferredSuffix = preferredAirspaceSuffix(baseType);
        if (!preferredSuffix.empty() && containsLookupPhrase(record.canonicalNormalizedName, preferredSuffix))
        {
            score += 30;
        }
        if (!hasAreaOrSectorDecoration(record.name))
        {
            score += 10;
        }

        const float lowerBound = record.hasLowerBound
            ? record.lowerBoundFeet
            : numeric_limits<float>::max();
        const float upperBound = record.hasUpperBound
            ? record.upperBoundFeet
            : -numeric_limits<float>::max();
        const size_t nameLength = record.canonicalName.size();

        if (!foundMatch
            || score > bestScore
            || (score == bestScore && lowerBound < bestLowerBound)
            || (score == bestScore && lowerBound == bestLowerBound && upperBound > bestUpperBound)
            || (score == bestScore && lowerBound == bestLowerBound && upperBound == bestUpperBound && nameLength < bestNameLength))
        {
            foundMatch = true;
            bestIndex = candidateIndex;
            bestScore = score;
            bestLowerBound = lowerBound;
            bestUpperBound = upperBound;
            bestNameLength = nameLength;
        }
    }

    if (!foundMatch)
    {
        return false;
    }

    const auto& bestRecord = airspaceCache->records.at(bestIndex);
    match.displayName = bestRecord.canonicalName;
    match.hasLowerBound = bestRecord.hasLowerBound;
    match.lowerBoundFeet = bestRecord.lowerBoundFeet;
    match.hasUpperBound = bestRecord.hasUpperBound;
    match.upperBoundFeet = bestRecord.upperBoundFeet;
    match.classification = &resolveNamedAirspaceClass(bestRecord, baseType, baseClassification);

    host->writeLog(
        "ATCNAV|Matched airspace.txt for airport[%s] as [%s]",
        header.icao().c_str(),
        match.displayName.c_str());

    return true;
}

void XPAtcNavData::appendControllerPositions(
    shared_ptr<HostServices> host,
    const string& airportIcao,
    vector<ControllerPosition::Structure>& positions)
{
    const auto cache = getAtcNavCache(host);
    if (!cache)
    {
        return;
    }

    const string airportKey = toUpperCopy(airportIcao);
    const auto found = cache->blocksByAirport.find(airportKey);
    if (found == cache->blocksByAirport.end())
    {
        return;
    }

    AtcRoleAggregate towerAggregate;
    AtcRoleAggregate terminalAggregate;
    AtcRoleAggregate approachAggregate;
    AtcRoleAggregate departureAggregate;
    AtcRoleAggregate centerAggregate;
    collectAtcRoleAggregates(
        found->second,
        towerAggregate,
        terminalAggregate,
        approachAggregate,
        departureAggregate,
        centerAggregate);

    struct NavControllerSpec
    {
        ControllerPosition::Type type;
        int frequencyKhz;
        GeoPolygon scopeLimit;
    };

    vector<NavControllerSpec> navSpecs = {
        {
            ControllerPosition::Type::Local,
            selectAtcFrequency(towerAggregate.frequenciesKhz),
            buildMergedScopePolygon(towerAggregate.polygons)
        },
        {
            ControllerPosition::Type::Approach,
            selectAtcFrequency(approachAggregate.frequenciesKhz),
            buildMergedScopePolygon(approachAggregate.polygons)
        },
        {
            ControllerPosition::Type::Departure,
            selectAtcFrequency(departureAggregate.frequenciesKhz, 1),
            buildMergedScopePolygon(departureAggregate.polygons)
        },
        {
            ControllerPosition::Type::Area,
            selectAtcFrequency(
                centerAggregate.present
                    ? centerAggregate.frequenciesKhz
                    : terminalAggregate.frequenciesKhz),
            buildMergedScopePolygon(
                centerAggregate.present
                    ? centerAggregate.polygons
                    : terminalAggregate.polygons)
        }
    };

    int updatedCount = 0;
    int addedCount = 0;

    vector<ControllerPosition::Structure> mergedPositions;
    mergedPositions.reserve(positions.size() + navSpecs.size());

    for (const auto& position : positions)
    {
        const auto matchingNavSpec = find_if(
            navSpecs.begin(),
            navSpecs.end(),
            [&](const NavControllerSpec& navSpec) {
                return navSpec.type == position.type;
            });

        if (matchingNavSpec != navSpecs.end()
            && position.scopeLimit.isEmpty()
            && !matchingNavSpec->scopeLimit.isEmpty())
        {
            mergedPositions.push_back({
                position.type,
                position.frequencyKhz,
                matchingNavSpec->scopeLimit,
                position.callSign
            });
            updatedCount++;
            continue;
        }

        mergedPositions.push_back(position);
    }

    for (const auto& navSpec : navSpecs)
    {
        if (navSpec.frequencyKhz <= 0 || hasControllerPositionType(mergedPositions, navSpec.type))
        {
            continue;
        }

        mergedPositions.push_back({ navSpec.type, navSpec.frequencyKhz, navSpec.scopeLimit, "" });
        addedCount++;
    }

    positions = move(mergedPositions);

    if (addedCount > 0 || updatedCount > 0)
    {
        host->writeLog(
            "ATCNAV|Applied navdata sectors for airport[%s]: added[%d] updated[%d]",
            airportIcao.c_str(),
            addedCount,
            updatedCount);
    }
}

shared_ptr<ControlledAirspace> XPAtcNavData::queryAirportAirspace(
    shared_ptr<HostServices> host,
    const Airport::Header& header)
{
    const auto cache = getAtcNavCache(host);
    if (!cache)
    {
        return nullptr;
    }

    const string airportKey = toUpperCopy(header.icao());
    const auto found = cache->blocksByAirport.find(airportKey);
    if (found == cache->blocksByAirport.end())
    {
        return nullptr;
    }

    AtcRoleAggregate towerAggregate;
    AtcRoleAggregate terminalAggregate;
    AtcRoleAggregate approachAggregate;
    AtcRoleAggregate departureAggregate;
    AtcRoleAggregate centerAggregate;
    collectAtcRoleAggregates(
        found->second,
        towerAggregate,
        terminalAggregate,
        approachAggregate,
        departureAggregate,
        centerAggregate);

    vector<AtcPolygonDefinition> facilityPolygons;
    string classificationText;
    ControlledAirspace::Type airspaceType = ControlledAirspace::Type::ControlZone;

    if (terminalAggregate.present)
    {
        airspaceType = ControlledAirspace::Type::TerminalControlArea;
        classificationText = terminalAggregate.airspaceClass;
        appendPolygons(terminalAggregate.polygons, facilityPolygons);
        appendPolygons(towerAggregate.polygons, facilityPolygons);
    }
    else if (towerAggregate.present)
    {
        airspaceType = ControlledAirspace::Type::ControlZone;
        classificationText = towerAggregate.airspaceClass;
        appendPolygons(towerAggregate.polygons, facilityPolygons);
    }
    else if (centerAggregate.present)
    {
        airspaceType = ControlledAirspace::Type::AreaFIR;
        classificationText = centerAggregate.airspaceClass;
        appendPolygons(centerAggregate.polygons, facilityPolygons);
    }

    const GeoPolygon lateralBounds = buildMergedScopePolygon(facilityPolygons);
    if (lateralBounds.isEmpty())
    {
        return nullptr;
    }

    bool hasLowerBound = false;
    float lowerBoundFeet = 0.0f;
    bool hasUpperBound = false;
    float upperBoundFeet = 0.0f;

    for (const auto& polygon : facilityPolygons)
    {
        if (polygon.points.empty())
        {
            continue;
        }

        if (polygon.hasLowerBound)
        {
            lowerBoundFeet = hasLowerBound
                ? min(lowerBoundFeet, polygon.lowerBoundFeet)
                : polygon.lowerBoundFeet;
            hasLowerBound = true;
        }

        if (polygon.hasUpperBound)
        {
            upperBoundFeet = hasUpperBound
                ? max(upperBoundFeet, polygon.upperBoundFeet)
                : polygon.upperBoundFeet;
            hasUpperBound = true;
        }
    }

    if (!hasUpperBound)
    {
        hasUpperBound = true;
        upperBoundFeet = 18000.0f;
    }

    const AirspaceClass& baseClassification = toAirspaceClass(classificationText, airspaceType);

    string name = header.icao();
    if (airspaceType == ControlledAirspace::Type::TerminalControlArea)
    {
        name += " TMA";
    }
    else if (airspaceType == ControlledAirspace::Type::ControlZone)
    {
        name += " CTR";
    }
    else if (airspaceType == ControlledAirspace::Type::AreaFIR)
    {
        name += " FIR";
    }

    string centerName = header.name();
    const AirspaceClass* classification = &baseClassification;

    NamedAirspaceMatch namedAirspaceMatch;
    if (tryResolveNamedAirspaceMatch(host, header, airspaceType, baseClassification, namedAirspaceMatch))
    {
        if (!namedAirspaceMatch.displayName.empty())
        {
            name = namedAirspaceMatch.displayName;
            centerName = namedAirspaceMatch.displayName;
        }

        if (namedAirspaceMatch.classification)
        {
            classification = namedAirspaceMatch.classification;
        }

        if (namedAirspaceMatch.hasLowerBound)
        {
            hasLowerBound = true;
            lowerBoundFeet = namedAirspaceMatch.lowerBoundFeet;
        }

        if (namedAirspaceMatch.hasUpperBound)
        {
            hasUpperBound = true;
            upperBoundFeet = namedAirspaceMatch.upperBoundFeet;
        }
    }

    auto geometry = shared_ptr<AirspaceGeometry>(new AirspaceGeometry(
        lateralBounds,
        hasLowerBound,
        lowerBoundFeet,
        hasUpperBound,
        upperBoundFeet));

    return shared_ptr<ControlledAirspace>(new ControlledAirspace(
        1,
        header.icao(),
        header.icao(),
        centerName,
        name,
        airspaceType,
        *classification,
        geometry));
}

XPAirportReader::XPAirportReader(
    shared_ptr<HostServices> _host,
    int _unparsedLineCode,
    QueryAirspaceCallback _onQueryAirspace,
    FilterAirportCallback _onFilterAirport,
    bool _logSkippedAirportFilterHits
) : m_host(std::move(_host)),
    m_onQueryAirspace(std::move(_onQueryAirspace)),
    m_onFilterAirport(std::move(_onFilterAirport)),
    m_logSkippedAirportFilterHits(_logSkippedAirportFilterHits),
    m_unparsedLineCode(_unparsedLineCode),
    m_nextEdgeId(1001),
    m_nextParkingStandId(301),
    m_datumLatitude(DATUM_UNSPECIFIED),
    m_datumLongitude(DATUM_UNSPECIFIED),
    m_elevation(0),
    m_skippingAirport(false),
    m_headerWasRead(false),
    m_filterWasQueried(false)
{
}

void XPAirportReader::readAirport(istream& input)
{
    readAptDatInContext(input, [&](int lineCode) {
        return rootContextParser(lineCode, input);
    });
}

bool XPAirportReader::validate(vector<string>& diagnostics)
{
    return true;
}

shared_ptr<Airport> XPAirportReader::getAirport()
{
    if (!m_skippingAirport)
    {
        try
        {
            return assembleAirportOrThrow();
        }
        catch (const exception &e)
        {
            m_host->writeLog("APTDAT|FAILED to assemble airport [%s]: %s", m_icao.c_str(), e.what());
        }
    }

    return nullptr;
}

shared_ptr<Airport> XPAirportReader::assembleAirportOrThrow()
{
    GeoPoint datum(
        m_datumLatitude != DATUM_UNSPECIFIED ? m_datumLatitude : 0,
        m_datumLongitude != DATUM_UNSPECIFIED ? m_datumLongitude : 0);

    Airport::Header header(m_icao, m_name, datum, m_elevation, m_iata);
    m_airspace = m_onQueryAirspace(header);
    vector<ControllerPosition::Structure> controllerPositions = m_controllerPositions;
    XPAtcNavData::appendControllerPositions(m_host, m_icao, controllerPositions);
    
    shared_ptr<ControlFacility> tower = (!controllerPositions.empty() || m_airspace)
        ? WorldBuilder::assembleAirportTower(m_host, header, m_airspace, controllerPositions)
        : nullptr;

    auto airport = WorldBuilder::assembleAirport(
        m_host,
        header,
        m_runways, 
        m_parkingStands, 
        m_taxiNodes, 
        m_taxiEdges,
        tower, 
        m_airspace,
        m_trafficFlows);

    return airport;
}

void XPAirportReader::readAptDatInContext(istream& input, ContextualParser parser)
{   
    while (!input.eof() && !input.bad())
    {
        int saveLineCode = m_unparsedLineCode;
        streampos saveInputPosition = input.tellg();

        try
        {
            if (!readAptDatLineInContext(input, parser))
            {
                break;
            }
        }
        catch (const exception& e)
        {
            m_host->writeLog(
                "APTDAT|FAILED to read apt.dat: airport[%s] error [%s] lineCode[%d]",
                m_icao.c_str(),
                e.what(),
                saveLineCode);
            m_skippingAirport = true;

            const auto oldExceptions = input.exceptions();
            try
            {
                input.exceptions(ios::goodbit);
                input.clear();
                skipToNextLine(input);
            }
            catch(const exception&)
            {
            }

            try
            {
                input.exceptions(oldExceptions);
            }
            catch(const exception&)
            {
            }
        }
    }
}

bool XPAirportReader::readAptDatLineInContext(istream &input, XPAirportReader::ContextualParser parser)
{
    int lineCode = m_unparsedLineCode >= 0
       ? m_unparsedLineCode
       : extractNextLineCode(input);

    if (lineCode < 0)
    {
        return false;
    }

    m_unparsedLineCode = -1;

    bool accepted = parser(lineCode);
    if (!accepted)
    {
        m_unparsedLineCode = lineCode;
        return false;
    }

    return true;
} 

bool XPAirportReader::rootContextParser(int lineCode, istream& input)
{
    if (m_skippingAirport)
    {
        if (lineCode == 1)
        {
            return false;
        }
        skipToNextLine(input);
        return true;
    }

    if (m_headerWasRead)
    {
        if (lineCode == 1)
        {
            return false; // we're at the beginning of the next airport
        }
        if (lineCode != 1302 && !m_filterWasQueried)
        {
            m_skippingAirport = !invokeFilterCallback();
            m_filterWasQueried = true;
            if (m_skippingAirport && m_logSkippedAirportFilterHits)
            {
                m_host->writeLog("APTDAT|will skip airport [%s] according to filter", m_icao.c_str());
            }
        }
    }

    switch (lineCode)
    {
    case 1:
        parseHeader1(input);
        m_headerWasRead = true;
        break;
    case 100:
        parseRunway100(input);
        break;
    case 1201:
        parseTaxiNode1201(input);
        break;
    case 1202:
        parseTaxiEdge1202(input);
        break;
    case 1206:
        parseGroundEdge1206(input);
        break;
    case 1300:
        parseStartupLocation1300(input);
        break;
    case 1302:
        parseMetadata1302(input);
        break;
    case 1000:
        if (m_headerWasRead)
        {
            parseTrafficFlow1000(input);
        }
        else
        {
            skipToNextLine(input);
        }
        break;
    default:
        if (isControlFrequencyLine(lineCode))
        {
            parseControlFrequency(lineCode, input);
        }
        else
        {
            skipToNextLine(input);
        }
        break;
    }

    return true;
}

void XPAirportReader::parseHeader1(istream &input)
{
    int deprecated;
    input >> m_elevation >> deprecated >> deprecated >> m_icao;
    m_name = readToEndOfLine(input);
}

void XPAirportReader::parseRunway100(istream& input)
{
    const auto parseEnd = [this,&input](){
        string name;
        float displasedThresholdMeters;
        float overrunAreaMeters;
        GeoPoint centerlinePoint = {0,0,0};
        int unusedInt;

        input >> name >> centerlinePoint.latitude >> centerlinePoint.longitude;
        input >> displasedThresholdMeters >> overrunAreaMeters;
        input >> unusedInt >> unusedInt >> unusedInt >> unusedInt;

        
        return Runway::End(
            name, 
            displasedThresholdMeters, 
            overrunAreaMeters, 
            UniPoint(m_host, centerlinePoint));
    };

    float widthMeters;
    int unusedInt;
    float unusedFloat;
    input >> widthMeters >> unusedInt >> unusedInt;
    input >> unusedFloat >> unusedInt >> unusedInt >> unusedInt;
    
    auto end1 = parseEnd();
    auto end2 = parseEnd();
    auto runway = make_shared<Runway>(end1, end2, widthMeters);
    
    m_runways.push_back(runway);
}

void XPAirportReader::parseTaxiNode1201(istream& input)
{
    double latitude;
    double longitude;
    string usage;
    int id;
    string name;

    input.precision(11);
    input >> latitude >> longitude >> usage >> id;
    name = readToEndOfLine(input);

    UniPoint location(m_host, GeoPoint({latitude, longitude, 0}));
    auto node = make_shared<TaxiNode>(id, location);
    assignTaxiNodeUsage(node, usage);
    
    m_taxiNodes.push_back(node);
    m_taxiNodeById.insert({ id, node });
}

void XPAirportReader::parseTaxiEdge1202(istream& input)
{
    int nodeId1;
    int nodeId2;
    string direction;
    string typeString;
    string name;

    input >> nodeId1 >> nodeId2 >> direction >> typeString;
    name = readToEndOfLine(input);

    bool isOneWay = (direction.compare("oneway") == 0);
    TaxiEdge::Type type = TaxiEdge::Type::Taxiway;
    if (stringStartsWith(typeString, "runway"))
    {
        type = TaxiEdge::Type::Runway;
    }
    else if (stringStartsWith(typeString, "hold"))
    {
        type = TaxiEdge::Type::HoldShort;
    }

    int edgeId = m_nextEdgeId++;
    auto edge = shared_ptr<TaxiEdge>(new TaxiEdge(
        edgeId, 
        name, 
        nodeId1, 
        nodeId2, 
        type, 
        {},
        isOneWay));
    
    m_taxiEdges.push_back(edge);

    readAptDatInContext(input, [this, &input, edge](int lineCode) {
        if (lineCode == 1204)
        {
            parseRunwayActiveZone1204(input, edge);
            return true;
        }
        return false;
    });
}

void XPAirportReader::parseGroundEdge1206(istream &input)
{
    int nodeId1;
    int nodeId2;
    string direction;
    string name;

    input >> nodeId1 >> nodeId2 >> direction;
    name = readToEndOfLine(input);

    bool isOneWay = (direction.compare("oneway") == 0);
    int edgeId = m_nextEdgeId++;

    auto edge = shared_ptr<TaxiEdge>(new TaxiEdge(
        edgeId, 
        name, 
        nodeId1, 
        nodeId2, 
        TaxiEdge::Type::Groundway, 
        {},
        isOneWay));
    
    m_taxiEdges.push_back(edge);
}

void XPAirportReader::parseRunwayActiveZone1204(istream& input, shared_ptr<TaxiEdge> edge)
{
    string classification;
    string runwayIdList;

    input >> classification >> runwayIdList;
    
    bool isDeparture = (classification.compare("departure") == 0);
    bool isArrival = (classification.compare("arrival") == 0);
    bool isIls = (classification.compare("ils") == 0);

    parseSeparatedList(runwayIdList, ",", [edge, isDeparture, isArrival, isIls](const string& runwayId) {
        const string trimmedRunwayId = trimCopy(runwayId);
        if (!trimmedRunwayId.empty())
        {
            WorldBuilder::addActiveZone(edge, trimmedRunwayId, isDeparture, isArrival, isIls);
        }
    });
}

void XPAirportReader::parseStartupLocation1300(istream &input)
{
    double latitude;
    double longitude;
    float heading;
    string typeText;
    string categoriesText;
    string operationTypesText;
    string name;
    string widthCode;
    string airlinesText;

    input >> latitude >> longitude >> heading >> typeText >> categoriesText;
    name = readToEndOfLine(input);

    readAptDatInContext(input, [&](int lineCode){
        if (lineCode == 1301)
        {
            input >> widthCode >> operationTypesText;
            airlinesText = readToEndOfLine(input);
            return true;
        }
        return false;
    });

    UniPoint location = UniPoint::fromGeo(m_host, latitude, longitude, 0);
    ParkingStand::Type type = getValueOrThrow(parkingStandTypeLookup, typeText);
    Aircraft::Category categories = Aircraft::Category::None;
    Aircraft::OperationType operationTypes = Aircraft::OperationType::None;
    vector<string> airlines;

    parseSeparatedList(airlinesText, ",;:| \t", [&airlines](const string& item) {
        airlines.push_back(item);
    });
    parseSeparatedList(categoriesText, ",;:| \t", [&categories](const string& item) {
        categories = categories | getValueOrThrow(aircraftCategoryLookup, item);
    });
    parseSeparatedList(operationTypesText, ",;:| \t", [&operationTypes](const string& item) {
        operationTypes = operationTypes | getValueOrThrow(aircraftOperationTypeLookup, item);
    });

    auto parkingStand = shared_ptr<ParkingStand>(new ParkingStand(
        m_nextParkingStandId++, name, type, location, heading, widthCode, categories, operationTypes, airlines));

    m_parkingStands.push_back(parkingStand);
}

void XPAirportReader::parseMetadata1302(istream &input)
{
    string fieldName;
    input >> fieldName;

    if (fieldName.compare("datum_lat") == 0)
    {
        input >> m_datumLatitude;
    }
    else if (fieldName.compare("datum_lon") == 0)
    {
        input >> m_datumLongitude;
    }
    else if (fieldName.compare("icao_code") == 0)
    {
        string newIcao;
        input >> newIcao;
        // The 1302 metadata value is the authoritative ICAO code and may
        // correct an earlier header value.
        if (!newIcao.empty())
        {
            m_icao = newIcao;
        }
    }
    else if (fieldName.compare("iata_code") == 0)
    {
        input >> m_iata;
    }
    else if (fieldName.compare("faa_code") == 0 && m_iata.empty())
    {
        input >> m_iata;
    }
    else
    {
        readToEndOfLine(input);
    }
}

bool XPAirportReader::isControlFrequencyLine(int lineCode)
{
    return ((lineCode >= 50 && lineCode <= 56) || (lineCode >= 1050 && lineCode <= 1056));
}

void XPAirportReader::parseControlFrequency(int lineCode, istream &input)
{
    if (hasKey(m_parsedFrequencyLineCodes, lineCode))
    {
        return;
    }

    const auto getPositionType = [lineCode]() {
        switch (lineCode)
        {
            case 52:
            case 1052:
                return ControllerPosition::Type::ClearanceDelivery;
            case 53:
            case 1053:
                return ControllerPosition::Type::Ground;
            case 54:
            case 1054:
                return ControllerPosition::Type::Local;
            case 55:
            case 1055:
                return ControllerPosition::Type::Approach;
            case 56:
            case 1056:
                return ControllerPosition::Type::Departure;
            default:
                return ControllerPosition::Type::Unknown;
        }
    };

    const auto positionType = getPositionType();
    if (positionType != ControllerPosition::Type::Unknown)
    {
        int khz;
        string callSign;
        input >> khz >> callSign;
        
        if (tryInsertKey(m_parsedFrequencyKhz, khz))
        {
            ControllerPosition::Structure position = { positionType, khz, GeoPolygon::empty(), callSign };
            m_controllerPositions.push_back(position);
            m_parsedFrequencyLineCodes.insert(lineCode);
        }
    }
}

string XPAirportReader::readFirstToken(istream& input)
{
    bool isAtLeadingSpace = true;

    string s;
    s.reserve(16);
    
    while (!input.eof() && input.peek() > -1)
    {
        char c = input.peek();
        bool isAtWhitespace = (c <= 0x20);
        bool isAtEndOfLine = (c == '\n' || c == '\r');
    
        if (isAtEndOfLine)
        {
            break;
        }
        
        if (isAtLeadingSpace)
        {
            if (isAtWhitespace)
            {
                input.get();
            }
            else 
            {
                isAtLeadingSpace = false;
            }
        }
        else        
        {
            if (!isAtWhitespace)
            {
                s.push_back(input.get());
            }
            else 
            {
                break;
            }
        }
    }

    return s;
}

string XPAirportReader::readToEndOfLine(istream& input)
{
    const int stateLeadingSpace = 0;
    const int stateContents = 1;
    const int stateMaybeTrailingSpace = 2;
    const int stateEndOfLine = 3;
    const int stateStop = 4;

    int state = stateLeadingSpace;
    string s;
    s.reserve(16);

    while (!input.eof() && input.peek() > -1 && state != stateStop)
    {
        char c = input.peek();
        bool isWhitespace = (c <= 0x20);
        bool isEndOfLine = (c == '\n' || c == '\r');
        if (isEndOfLine)
        {
            state = stateEndOfLine;
        }

        switch (state)
        {
        case stateLeadingSpace:
            if (isWhitespace)
            {
                input.get();
            }
            else 
            {
                state = stateContents;
            }
            break;
        case stateContents:
            if (!isWhitespace)
            {
                s.push_back(input.get());
            }
            else 
            {
                state = stateMaybeTrailingSpace;
            }
            break;
        case stateMaybeTrailingSpace:
            if (isWhitespace)
            {
                input.get();
            }
            else 
            {
                s.push_back(' ');
                state = stateContents;
            }
            break;
        case stateEndOfLine:
            if (isEndOfLine)
            {
                input.get();
            }
            else 
            {
                state = stateStop;
            }
            break;
        }
    }

    return s;
}

int XPAirportReader::extractNextLineCode(istream &input)
{
    while (!input.eof() && input.peek() >= 0)
    {
        string firstToken = readFirstToken(input);
        if (firstToken.length() == 0 || firstToken[0] < '0' || firstToken[0] > '9')
        {
            getline(input, firstToken);
            continue;
        }
        return stoi(firstToken);
    }
    return -1;
}

string XPAirportReader::formatErrorMessage(istream &input, const streampos& position, int extractedLineCode, const char *what)
{
    stringstream message;
    message << "FAILED to read apt.dat: airport[" << m_icao << "] error [" << what << "] line [";
    if (extractedLineCode >= 0)
    {
        message << "code[" << extractedLineCode << "] > ";
    }

    const auto oldExceptions = input.exceptions();
    const auto oldState = input.rdstate();

    try
    {
        input.exceptions(ios::goodbit);
        input.clear();
        input.seekg(position);

        string line;
        getline(input, line);
        message << line;
    }
    catch(const exception&)
    {
        message << "<unavailable>";
    }

    try
    {
        input.clear(oldState);
        input.exceptions(oldExceptions);
    }
    catch(const exception&)
    {
    }

    message << ']';

    return message.str();
}

void XPAirportReader::skipToNextLine(istream& input)
{
    bool atEndOfLine = false;

    while (!input.eof() && input.peek() > -1)
    {
        char c = input.peek();
        bool isEolChar = (c == '\r' || c == '\n');

        if (atEndOfLine && !isEolChar)
        {
            break;
        }

        input.get();

        if (isEolChar)
        {
            atEndOfLine = true;
        }
    }
}

shared_ptr<ControlledAirspace> XPAirportReader::noopQueryAirspace(const Airport::Header& header)
{
    return nullptr;
}

bool XPAirportReader::noopFilterAirport(const Airport::Header &header)
{
    return true;
}

bool XPAirportReader::invokeFilterCallback()
{
    Airport::Header header(m_icao, m_name, GeoPoint(m_datumLatitude, m_datumLongitude), m_elevation, m_iata);
    return m_onFilterAirport(header);
}

XPAptDatReader::XPAptDatReader(shared_ptr<HostServices> _host) :
    m_host(std::move(_host))
{
}

void XPAptDatReader::readAptDat(
    istream &input,
    const XPAirportReader::QueryAirspaceCallback& onQueryAirspace,
    const XPAirportReader::FilterAirportCallback& onFilterAirport,
    const XPAptDatReader::AirportLoadedCallback& onAirportLoaded,
    bool logSkippedAirports)
{
    int loadedCount = 0;
    int skippedCount = 0;
    int unparsedLineCode = -1;

    do {
        XPAirportReader airportReader(m_host, unparsedLineCode, onQueryAirspace, onFilterAirport, logSkippedAirports);
        airportReader.readAirport(input);
        unparsedLineCode = airportReader.unparsedLineCode();

        auto airport = airportReader.getAirport();
        if (airport)
        {
            //m_host->writeLog("Airport loaded: %s", airport->header().icao().c_str());
            onAirportLoaded(airport);
            loadedCount++;
        }
        else
        {
            if (logSkippedAirports)
            {
                m_host->writeLog("APTDAT|skipped airport [%s]", airportReader.icao().c_str());
            }
            skippedCount++;
        }
    } while (unparsedLineCode == 1);

    m_host->writeLog("APTDAT|done loading airports, %d loaded, %d skipped.", loadedCount, skippedCount);
}

void XPAirportReader::parseTrafficFlow1000(istream& input)
{
    string flowName = readToEndOfLine(input);
    if (flowName.empty())
    {
        flowName = "unnamed flow";
    }

    auto flow = make_shared<TrafficFlow>(flowName, m_icao, 0.0f, 359.0f, -1.0f, -1.0f);
    m_currentFlow = flow;
    m_trafficFlows.push_back(flow);

    readAptDatInContext(input, [this, &input, flow](int lineCode) {
        switch (lineCode)
        {
        case 1001:
            parseTrafficFlowWindRule1001(input, flow);
            return true;
        case 1002:
            parseTrafficFlowCeilingRule1002(input, flow);
            return true;
        case 1003:
            parseTrafficFlowVisibilityRule1003(input, flow);
            return true;
        case 1004:
            skipToNextLine(input);
            return true;
        case 1100:
        case 1110:
            parseRunwayInUseRule(input, flow);
            return true;
        case 1101:
            skipToNextLine(input);
            return true;
        default:
            return false;
        }
    });
}

void XPAirportReader::parseTrafficFlowWindRule1001(istream& input, shared_ptr<TrafficFlow> flow)
{
    const auto tokens = splitTokens(readToEndOfLine(input));
    if (tokens.size() < 4)
    {
        return;
    }

    const string reportingStationIcao = tokens.at(0);
    const float windFrom = stof(tokens.at(1));
    const float windTo = stof(tokens.at(2));
    const float maximumWindSpeedKt = stof(tokens.at(3));

    if (flow && windFrom >= 0.0f && windTo >= 0.0f)
    {
        flow->addWindRule(reportingStationIcao, windFrom, windTo, maximumWindSpeedKt);
        if (flow->windRules().size() == 1)
        {
            flow->setWindRange(windFrom, windTo);
        }
    }
}

void XPAirportReader::parseTrafficFlowCeilingRule1002(istream& input, shared_ptr<TrafficFlow> flow)
{
    const auto tokens = splitTokens(readToEndOfLine(input));
    if (tokens.size() < 2)
    {
        return;
    }

    const float minimumCeilingFeetAgl = stof(tokens.at(1));

    if (flow)
    {
        flow->setMinimumCeilingFeetAgl(minimumCeilingFeetAgl);
    }
}

void XPAirportReader::parseTrafficFlowVisibilityRule1003(istream& input, shared_ptr<TrafficFlow> flow)
{
    const auto tokens = splitTokens(readToEndOfLine(input));
    if (tokens.size() < 2)
    {
        return;
    }

    const float minimumVisibilityStatuteMiles = stof(tokens.at(1));

    if (flow)
    {
        flow->setMinimumVisibilityStatuteMiles(minimumVisibilityStatuteMiles);
    }
}

void XPAirportReader::parseRunwayInUseRule(istream& input, shared_ptr<TrafficFlow> flow)
{
    const auto tokens = splitTokens(readToEndOfLine(input));
    if (tokens.size() < 3)
    {
        return;
    }

    const string runwayName = tokens.at(0);
    const string operations = tokens.at(2);
    const bool arrival = tokenListContains(operations, "arrivals") ||
        tokenListContains(operations, "arrival") ||
        tokenListContains(operations, "both");
    const bool departure = tokenListContains(operations, "departures") ||
        tokenListContains(operations, "departure") ||
        tokenListContains(operations, "both");

    if (flow && (arrival || departure))
    {
        flow->addRunwayUse(runwayName, arrival, departure);
    }
}
