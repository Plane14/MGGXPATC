// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../libworld/libworld.h"
#include "xpCifpReader.hpp"

using namespace std;
using namespace world;

class XPProcedureValidator
{
public:
    enum class ValidationSeverity
    {
        Info,
        Warning,
        Error
    };

    struct ValidationIssue
    {
        ValidationSeverity severity = ValidationSeverity::Info;
        string message;
        int sequence = -1;
        string waypoint;

        ValidationIssue() = default;
        ValidationIssue(ValidationSeverity sev, const string& msg, int seq = -1, const string& wp = "")
            : severity(sev), message(msg), sequence(seq), waypoint(wp) {}
    };

    struct ValidationResult
    {
        bool isValid = true;
        vector<ValidationIssue> issues;

        void addIssue(ValidationSeverity severity, const string& message, int sequence = -1, const string& waypoint = "")
        {
            issues.emplace_back(severity, message, sequence, waypoint);
            if (severity == ValidationSeverity::Error)
            {
                isValid = false;
            }
        }

        string toString() const
        {
            ostringstream out;
            for (const auto& issue : issues)
            {
                switch (issue.severity)
                {
                case ValidationSeverity::Error:
                    out << "ERROR";
                    break;
                case ValidationSeverity::Warning:
                    out << "WARNING";
                    break;
                case ValidationSeverity::Info:
                    out << "INFO";
                    break;
                }

                out << " [seq=" << issue.sequence;
                if (!issue.waypoint.empty())
                {
                    out << ", wp=" << issue.waypoint;
                }
                out << "]: " << issue.message << "\n";
            }

            if (isValid)
            {
                out << "Procedure validation passed.\n";
            }
            else
            {
                out << "Procedure validation FAILED.\n";
            }

            return out.str();
        }
    };

    explicit XPProcedureValidator(shared_ptr<HostServices> host) :
        m_host(std::move(host))
    {
    }

    // Validate a complete procedure track
    ValidationResult validateProcedureTrack(
        const vector<XPCifpReader::WaypointWithLocation>& waypoints,
        const vector<XPCifpReader::WaypointWithLocation>& missedWaypoints,
        const string& procedureType,
        const string& procedureName,
        float aircraftSpeedKnots = 250.0f,
        float aircraftAltitudeFeet = 0.0f) const
    {
        ValidationResult result;

        if (waypoints.empty())
        {
            result.addIssue(ValidationSeverity::Error, "Procedure track is empty");
            return result;
        }

        // Validate sequence continuity
        {
            auto seqResult = validateSequenceContinuity(waypoints);
            for (const auto& issue : seqResult.issues)
            {
                result.issues.push_back(issue);
            }
            if (!seqResult.isValid)
            {
                result.isValid = false;
            }
        }

        // Validate altitude constraints
        {
            auto altResult = validateAltitudeConstraints(waypoints, aircraftAltitudeFeet, aircraftSpeedKnots);
            for (const auto& issue : altResult.issues)
            {
                result.issues.push_back(issue);
            }
            if (!altResult.isValid)
            {
                result.isValid = false;
            }
        }

        // Validate speed constraints
        {
            auto spdResult = validateSpeedConstraints(waypoints, aircraftSpeedKnots);
            for (const auto& issue : spdResult.issues)
            {
                result.issues.push_back(issue);
            }
            if (!spdResult.isValid)
            {
                result.isValid = false;
            }
        }

        // Validate path terminators
        {
            auto pathResult = validatePathTerminators(waypoints, procedureType);
            for (const auto& issue : pathResult.issues)
            {
                result.issues.push_back(issue);
            }
            if (!pathResult.isValid)
            {
                result.isValid = false;
            }
        }

        // Validate RNAV/RNP procedures
        {
            auto rnavResult = validateRnavRnp(waypoints, procedureType, procedureName);
            for (const auto& issue : rnavResult.issues)
            {
                result.issues.push_back(issue);
            }
            if (!rnavResult.isValid)
            {
                result.isValid = false;
            }
        }

        // Validate missed approach (if applicable)
        if (procedureType == "APPCH")
        {
            auto maResult = validateMissedApproach(missedWaypoints);
            for (const auto& issue : maResult.issues)
            {
                result.issues.push_back(issue);
            }
            if (!maResult.isValid)
            {
                result.isValid = false;
            }
        }

        return result;
    }

    // Validate altitude constraints against aircraft capabilities
    ValidationResult validateAltitudeConstraints(
        const vector<XPCifpReader::WaypointWithLocation>& waypoints,
        float aircraftAltitudeFeet,
        float aircraftSpeedKnots,
        float aircraftRateOfClimbFpm = 1000.0f) const
    {
        ValidationResult result;

        if (waypoints.empty())
        {
            return result;
        }

        float currentAltitude = aircraftAltitudeFeet;
        float previousLatitude = 0.0f;
        float previousLongitude = 0.0f;
        bool hasPreviousLocation = false;

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            const auto& waypoint = waypoints[i];

            // Check altitude constraint
            if (waypoint.altitudeConstraint > 0.0f)
            {
                const float constraintAltitude = waypoint.altitudeConstraint;
                const char constraintType = waypoint.altitudeConstraintType;

                switch (constraintType)
                {
                case '+':  // At or above
                    if (currentAltitude < constraintAltitude - 50.0f)
                    {
                        result.addIssue(
                            ValidationSeverity::Warning,
                            "Altitude constraint 'at or above " + to_string(static_cast<int>(constraintAltitude)) +
                            "' may not be achievable",
                            static_cast<int>(i),
                            waypoint.name);
                    }
                    break;

                case '-':  // At or below
                    if (currentAltitude > constraintAltitude + 50.0f)
                    {
                        result.addIssue(
                            ValidationSeverity::Warning,
                            "Altitude constraint 'at or below " + to_string(static_cast<int>(constraintAltitude)) +
                            "' may not be achievable",
                            static_cast<int>(i),
                            waypoint.name);
                    }
                    break;

                case ' ':  // Exactly at
                default:
                    if (fabs(currentAltitude - constraintAltitude) > 100.0f)
                    {
                        result.addIssue(
                            ValidationSeverity::Info,
                            "Altitude change to " + to_string(static_cast<int>(constraintAltitude)) +
                            " required at this waypoint",
                            static_cast<int>(i),
                            waypoint.name);
                    }
                    break;
                }

                // Update current altitude for next leg
                currentAltitude = constraintAltitude;
            }

            // Check for altitude changes between waypoints
            if (hasPreviousLocation && waypoint.hasLocation)
            {
                const float distanceNm = calculateDistanceNm(
                    previousLatitude, previousLongitude,
                    waypoint.latitude, waypoint.longitude);

                if (distanceNm > 0.0f && waypoint.altitudeConstraint > 0.0f)
                {
                    // Estimate time to cover distance at aircraft speed
                    const float timeMinutes = (distanceNm / aircraftSpeedKnots) * 60.0f;
                    const float altitudeChange = fabs(currentAltitude - waypoint.altitudeConstraint);
                    const float requiredRate = altitudeChange / timeMinutes;  // ft/min

                    if (requiredRate > aircraftRateOfClimbFpm * 1.5f)
                    {
                        result.addIssue(
                            ValidationSeverity::Warning,
                            "Altitude change of " + to_string(static_cast<int>(altitudeChange)) +
                            " feet over " + to_string(static_cast<int>(distanceNm)) +
                            " NM may require excessive climb/descent rate",
                            static_cast<int>(i),
                            waypoint.name);
                    }
                }
            }

            if (waypoint.hasLocation)
            {
                previousLatitude = waypoint.latitude;
                previousLongitude = waypoint.longitude;
                hasPreviousLocation = true;
            }
        }

        return result;
    }

    // Validate speed constraints
    ValidationResult validateSpeedConstraints(
        const vector<XPCifpReader::WaypointWithLocation>& waypoints,
        float aircraftSpeedKnots) const
    {
        ValidationResult result;

        if (waypoints.empty())
        {
            return result;
        }

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            const auto& waypoint = waypoints[i];

            if (waypoint.speedConstraint > 0.0f)
            {
                const float constraintSpeed = waypoint.speedConstraint;
                const char constraintType = waypoint.speedConstraintType;

                switch (constraintType)
                {
                case '+':  // Minimum speed
                    if (aircraftSpeedKnots < constraintSpeed - 10.0f)
                    {
                        result.addIssue(
                            ValidationSeverity::Warning,
                            "Speed constraint 'at least " + to_string(static_cast<int>(constraintSpeed)) +
                            " knots' may not be met",
                            static_cast<int>(i),
                            waypoint.name);
                    }
                    break;

                case '-':  // Maximum speed
                    if (aircraftSpeedKnots > constraintSpeed + 10.0f)
                    {
                        result.addIssue(
                            ValidationSeverity::Warning,
                            "Speed constraint 'at most " + to_string(static_cast<int>(constraintSpeed)) +
                            " knots' may require deceleration",
                            static_cast<int>(i),
                            waypoint.name);
                    }
                    break;

                case ' ':  // Exact speed
                default:
                    if (fabs(aircraftSpeedKnots - constraintSpeed) > 20.0f)
                    {
                        result.addIssue(
                            ValidationSeverity::Info,
                            "Speed change to " + to_string(static_cast<int>(constraintSpeed)) +
                            " knots required at this waypoint",
                            static_cast<int>(i),
                            waypoint.name);
                    }
                    break;
                }
            }
        }

        return result;
    }

    // Validate RNAV/RNP procedure requirements
    ValidationResult validateRnavRnp(
        const vector<XPCifpReader::WaypointWithLocation>& waypoints,
        const string& procedureType,
        const string& procedureName) const
    {
        ValidationResult result;

        if (waypoints.empty())
        {
            return result;
        }

        // Check if this is an RNAV/RNP procedure
        const bool isRnav = procedureName.find("RNAV") != string::npos ||
            procedureName.find("RNP") != string::npos ||
            procedureName.find("GNSS") != string::npos ||
            procedureName.find("GPS") != string::npos ||
            procedureName.find("BRAVO") != string::npos ||
            procedureName.rfind("Q", 0) == 0 ||  // Q routes are RNAV
            procedureName.rfind("T", 0) == 0 ||  // T routes are RNAV
            procedureName.rfind("L", 0) == 0 ||  // L routes are RNAV
            procedureName.rfind("Y", 0) == 0;    // Y routes are RNAV

        if (!isRnav)
        {
            return result;
        }

        // RNAV/RNP procedures require specific waypoint types
        bool hasFlyByWaypoint = false;
        bool hasFlyOverWaypoint = false;

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            const auto& waypoint = waypoints[i];

            // Check for fly-by (TF) vs fly-over (IF, DF) path terminators
            // This is simplified - in real implementation, path terminators would be
            // available in the waypoint data
            if (waypoint.pathTerminator == "TF" || waypoint.pathTerminator == "CF")
            {
                hasFlyByWaypoint = true;
            }
            else if (waypoint.pathTerminator == "IF" || waypoint.pathTerminator == "DF")
            {
                hasFlyOverWaypoint = true;
            }

            // RNAV waypoints should have location data
            if (!waypoint.hasLocation)
            {
                result.addIssue(
                    ValidationSeverity::Warning,
                    "RNAV waypoint '" + waypoint.name + "' has no location data",
                    static_cast<int>(i),
                    waypoint.name);
            }
        }

        if (hasFlyByWaypoint && hasFlyOverWaypoint)
        {
            result.addIssue(
                ValidationSeverity::Info,
                "Procedure contains both fly-by and fly-over waypoints (typical for RNAV/RNP)");
        }

        return result;
    }

    // Validate missed approach procedure
    ValidationResult validateMissedApproach(
        const vector<XPCifpReader::WaypointWithLocation>& missedWaypoints) const
    {
        ValidationResult result;

        if (missedWaypoints.empty())
        {
            result.addIssue(ValidationSeverity::Warning, "No missed approach waypoints found");
            return result;
        }

        // Missed approach should have at least one waypoint
        if (missedWaypoints.size() < 2)
        {
            result.addIssue(ValidationSeverity::Warning, "Missed approach has fewer than 2 waypoints");
        }

        // Check for climb gradient requirement
        // Standard missed approach climb gradient is 2.4% (200 ft/NM) for precision approaches
        // 2.0% (160 ft/NM) for non-precision approaches
        bool hasClimbConstraint = false;
        for (size_t i = 0; i < missedWaypoints.size(); ++i)
        {
            const auto& waypoint = missedWaypoints[i];
            if (waypoint.altitudeConstraint > 0.0f && waypoint.altitudeConstraintType == '+')
            {
                hasClimbConstraint = true;
                result.addIssue(
                    ValidationSeverity::Info,
                    "Missed approach climb constraint: " + to_string(static_cast<int>(waypoint.altitudeConstraint)) +
                    " feet at " + waypoint.name,
                    static_cast<int>(i),
                    waypoint.name);
            }
        }

        if (!hasClimbConstraint)
        {
            result.addIssue(
                ValidationSeverity::Warning,
                "Missed approach has no explicit climb constraint");
        }

        return result;
    }

    // Validate path terminators for procedure type
    ValidationResult validatePathTerminators(
        const vector<XPCifpReader::WaypointWithLocation>& waypoints,
        const string& procedureType) const
    {
        ValidationResult result;

        if (waypoints.empty())
        {
            return result;
        }

        // Valid path terminator transitions per ARINC 424
        // IF -> TF/CF/AF/RF/DF/VA/VD/VI/VM
        // TF -> TF/CF/AF/RF/DF/VA/VD/VI/VM
        // CF -> TF/CF/AF/RF/DF/VA/VD/VI/VM
        // AF -> TF/CF/AF/RF/DF/VA/VD/VI/VM
        // RF -> TF/CF/AF/RF/DF/VA/VD/VI/VM
        // DF -> TF/CF/AF/RF/DF/VA/VD/VI/VM
        // VA -> TF/CF/AF/RF/DF/VA/VD/VI/VM
        // VD -> TF/CF/AF/RF/DF/VA/VD/VI/VM
        // VI -> TF/CF/AF/RF/DF/VA/VD/VI/VM
        // VM -> TF/CF/AF/RF/DF/VA/VD/VI/VM
        // HM -> TF/CF/AF/RF/DF/VA/VD/VI/VM
        // HA -> TF/CF/AF/RF/DF/VA/VD/VI/VM
        // FC -> TF/CF/AF/RF/DF/VA/VD/VI/VM
        // FM -> TF/CF/AF/RF/DF/VA/VD/VI/VM
        // FM -> (end)
        
        const unordered_map<string, vector<string>> validTransitions = {
            {"IF", {"TF", "CF", "AF", "RF", "DF", "VA", "VD", "VI", "VM"}},
            {"TF", {"TF", "CF", "AF", "RF", "DF", "VA", "VD", "VI", "VM"}},
            {"CF", {"TF", "CF", "AF", "RF", "DF", "VA", "VD", "VI", "VM"}},
            {"AF", {"TF", "CF", "AF", "RF", "DF", "VA", "VD", "VI", "VM"}},
            {"RF", {"TF", "CF", "AF", "RF", "DF", "VA", "VD", "VI", "VM"}},
            {"DF", {"TF", "CF", "AF", "RF", "DF", "VA", "VD", "VI", "VM"}},
            {"VA", {"TF", "CF", "AF", "RF", "DF", "VA", "VD", "VI", "VM"}},
            {"VD", {"TF", "CF", "AF", "RF", "DF", "VA", "VD", "VI", "VM"}},
            {"VI", {"TF", "CF", "AF", "RF", "DF", "VA", "VD", "VI", "VM"}},
            {"VM", {"TF", "CF", "AF", "RF", "DF", "VA", "VD", "VI", "VM"}},
            {"HM", {"TF", "CF", "AF", "RF", "DF", "VA", "VD", "VI", "VM"}},
            {"HA", {"TF", "CF", "AF", "RF", "DF", "VA", "VD", "VI", "VM"}},
            {"FC", {"TF", "CF", "AF", "RF", "DF", "VA", "VD", "VI", "VM"}},
            {"FM", {"TF", "CF", "AF", "RF", "DF", "VA", "VD", "VI", "VM"}}
        };

        // Validate transitions
        for (size_t i = 1; i < waypoints.size(); ++i)
        {
            const string& prevTerm = waypoints[i - 1].pathTerminator;
            const string& currTerm = waypoints[i].pathTerminator;

            if (!prevTerm.empty() && !currTerm.empty())
            {
                auto it = validTransitions.find(prevTerm);
                if (it != validTransitions.end())
                {
                    const auto& validNext = it->second;
                    if (find(validNext.begin(), validNext.end(), currTerm) == validNext.end())
                    {
                        result.addIssue(
                            ValidationSeverity::Warning,
                            "Invalid path terminator transition: " + prevTerm + " -> " + currTerm,
                            static_cast<int>(i),
                            waypoints[i].name);
                    }
                }
                else if (prevTerm != "PI")  // PI (Procedure Interface) has no defined transitions
                {
                    result.addIssue(
                        ValidationSeverity::Info,
                        "Unknown path terminator: " + prevTerm,
                        static_cast<int>(i - 1),
                        waypoints[i - 1].name);
                }
            }
        }

        // Check first waypoint for approaches - should typically be IF
        if (procedureType == "APPCH" && !waypoints.empty())
        {
            const string& firstTerm = waypoints.front().pathTerminator;
            if (!firstTerm.empty() && firstTerm != "IF" && firstTerm != "PI")
            {
                result.addIssue(
                    ValidationSeverity::Info,
                    "Approach first waypoint path terminator is " + firstTerm + " (expected IF)",
                    0,
                    waypoints.front().name);
            }
        }

        // Last waypoint should be a runway or MAP (Missed Approach Point)
        if (procedureType == "APPCH" && waypoints.size() >= 2)
        {
            const auto& lastWaypoint = waypoints.back();
            
            // Normalize runway token for comparison
            const string normalizedName = normalizeRunwayToken(lastWaypoint.name);
            const bool isRunway = !normalizedName.empty() &&
                all_of(normalizedName.begin(), normalizedName.end(), ::isdigit);
            
            // Also check if original name starts with RW prefix
            const string upperName = normalizeToken(lastWaypoint.name);
            const bool hasRWPrefix = upperName.rfind("RW", 0) == 0;

            if (!isRunway && !hasRWPrefix)
            {
                result.addIssue(
                    ValidationSeverity::Warning,
                    "Last waypoint in approach is not a runway: " + lastWaypoint.name);
            }
        }

        return result;
    }

    static string normalizeRunwayToken(const string& text)
    {
        string result = normalizeToken(text);
        if (result.rfind("RW", 0) == 0)
        {
            result.erase(0, 2);
        }
        return result;
    }

    // Validate sequence continuity
    ValidationResult validateSequenceContinuity(
        const vector<XPCifpReader::WaypointWithLocation>& waypoints) const
    {
        ValidationResult result;

        if (waypoints.empty())
        {
            return result;
        }

        // Check for duplicate waypoints
        unordered_set<string> seenWaypoints;
        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            const string normalizedName = normalizeToken(waypoints[i].name);
            if (seenWaypoints.count(normalizedName))
            {
                result.addIssue(
                    ValidationSeverity::Warning,
                    "Duplicate waypoint: " + waypoints[i].name,
                    static_cast<int>(i),
                    waypoints[i].name);
            }
            seenWaypoints.insert(normalizedName);
        }

        // Check for consecutive identical waypoints
        for (size_t i = 1; i < waypoints.size(); ++i)
        {
            if (normalizeToken(waypoints[i].name) == normalizeToken(waypoints[i - 1].name))
            {
                result.addIssue(
                    ValidationSeverity::Warning,
                    "Consecutive identical waypoints: " + waypoints[i].name,
                    static_cast<int>(i),
                    waypoints[i].name);
            }
        }

        // Check sequence monotonicity and gaps
        int prevSequence = -1;
        bool hasValidSequence = false;
        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            const int currentSequence = waypoints[i].sequence;
            if (currentSequence >= 0)
            {
                if (hasValidSequence)
                {
                    if (currentSequence <= prevSequence)
                    {
                        result.addIssue(
                            ValidationSeverity::Error,
                            "Sequence number not monotonically increasing: " + to_string(currentSequence) +
                            " follows " + to_string(prevSequence),
                            static_cast<int>(i),
                            waypoints[i].name);
                    }
                    else if (currentSequence > prevSequence + 1)
                    {
                        result.addIssue(
                            ValidationSeverity::Warning,
                            "Sequence gap detected: " + to_string(prevSequence) +
                            " -> " + to_string(currentSequence) + " (gap of " + to_string(currentSequence - prevSequence - 1) + ")",
                            static_cast<int>(i),
                            waypoints[i].name);
                    }
                }
                prevSequence = currentSequence;
                hasValidSequence = true;
            }
        }

        return result;
    }

    // Validate that a procedure is compatible with aircraft performance
    ValidationResult validateAircraftCompatibility(
        const vector<XPCifpReader::WaypointWithLocation>& waypoints,
        float aircraftMaxSpeedKnots,
        float aircraftMaxAltitudeFeet,
        float aircraftMaxClimbRateFpm) const
    {
        ValidationResult result;

        if (waypoints.empty())
        {
            return result;
        }

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            const auto& waypoint = waypoints[i];

            // Check speed constraint against aircraft max speed
            if (waypoint.speedConstraint > 0.0f && waypoint.speedConstraint > aircraftMaxSpeedKnots)
            {
                result.addIssue(
                    ValidationSeverity::Error,
                    "Speed constraint " + to_string(static_cast<int>(waypoint.speedConstraint)) +
                    " knots exceeds aircraft maximum speed " + to_string(static_cast<int>(aircraftMaxSpeedKnots)),
                    static_cast<int>(i),
                    waypoint.name);
            }

            // Check altitude constraint against aircraft max altitude
            if (waypoint.altitudeConstraint > 0.0f && waypoint.altitudeConstraint > aircraftMaxAltitudeFeet)
            {
                result.addIssue(
                    ValidationSeverity::Error,
                    "Altitude constraint " + to_string(static_cast<int>(waypoint.altitudeConstraint)) +
                    " feet exceeds aircraft maximum altitude " + to_string(static_cast<int>(aircraftMaxAltitudeFeet)),
                    static_cast<int>(i),
                    waypoint.name);
            }
        }

        return result;
    }

private:
    shared_ptr<HostServices> m_host;

    static string normalizeToken(const string& text)
    {
        string result;
        for (char c : text)
        {
            if (isalnum(static_cast<unsigned char>(c)))
            {
                result.push_back(static_cast<char>(toupper(static_cast<unsigned char>(c))));
            }
        }
        return result;
    }

    static float calculateDistanceNm(float lat1, float lon1, float lat2, float lon2)
    {
        // Simplified distance calculation (great circle)
        const float lat1Rad = lat1 * M_PI / 180.0f;
        const float lat2Rad = lat2 * M_PI / 180.0f;
        const float dLat = (lat2 - lat1) * M_PI / 180.0f;
        const float dLon = (lon2 - lon1) * M_PI / 180.0f;

        const float a = sin(dLat / 2.0f) * sin(dLat / 2.0f) +
            cos(lat1Rad) * cos(lat2Rad) *
            sin(dLon / 2.0f) * sin(dLon / 2.0f);
        const float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));

        // Earth radius in nautical miles
        return 3440.065f * c;
    }
};
