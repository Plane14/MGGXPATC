// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#pragma once

#include "libworld.h"
#include <chrono>
#include <unordered_map>
#include <unordered_set>

namespace world
{
    // Forward declarations
    class ControllerPosition;
    class Flight;
    class CoordinationMessage;

    // ============================================================================
    // SharedRadarScope - Shared radar scope awareness between controllers
    // ============================================================================

    class SharedRadarScope
    {
    public:
        enum class VisibilityRule
        {
            Full = 0,        // All data shared
            PositionOnly = 1, // Only position/altitude shared
            DataBlockOnly = 2, // Only data block (callsign, type, etc.)
            Restricted = 3   // Minimal data (position only, no callsign)
        };

        enum class UpdateType
        {
            PositionUpdate = 0,
            AltitudeChange = 1,
            SpeedChange = 2,
            HeadingChange = 3,
            FlightPlanUpdate = 4,
            HandoffInitiated = 5,
            HandoffAccepted = 6,
            HandoffCompleted = 7,
            EmergencyStatus = 8,
            LostContact = 9
        };

    private:
        struct FlightScopeData
        {
            shared_ptr<Flight> flight;
            shared_ptr<ControllerPosition> owningController;
            shared_ptr<ControllerPosition> previousController;
            GeoPoint lastKnownPosition;
            float lastKnownAltitudeFeet;
            float lastKnownGroundSpeedKt;
            float lastKnownHeadingDegrees;
            chrono::microseconds lastUpdateTimestamp;
            VisibilityRule visibilityRule;
            bool isEmergency;
            bool isRadarContact;
            bool isHandoffInProgress;
            string remarks;
        };

        struct ScopeVisibility
        {
            shared_ptr<ControllerPosition> controller;
            shared_ptr<ControllerPosition> targetController;
            VisibilityRule rule;
            chrono::microseconds establishedTimestamp;
            bool isBidirectional;
        };

    private:
        shared_ptr<HostServices> m_host;
        unordered_map<int, FlightScopeData> m_flightScopeData;
        vector<ScopeVisibility> m_scopeVisibilities;
        unordered_map<shared_ptr<ControllerPosition>, unordered_set<shared_ptr<Flight>>> m_controllerFlights;
        chrono::milliseconds m_dataFreshnessTimeoutMs;
        chrono::milliseconds m_visibilityTimeoutMs;

    public:
        SharedRadarScope(shared_ptr<HostServices> _host);
        ~SharedRadarScope();

    public:
        // Flight data management
        void updateFlightPosition(
            shared_ptr<Flight> flight,
            shared_ptr<ControllerPosition> controller,
            const GeoPoint& position,
            float altitudeFeet,
            float groundSpeedKt,
            float headingDegrees,
            chrono::microseconds timestamp
        );

        void setFlightEmergency(shared_ptr<Flight> flight, bool isEmergency);
        void setRadarContact(shared_ptr<Flight> flight, bool hasContact);
        void setHandoffInProgress(shared_ptr<Flight> flight, bool inProgress);
        void setFlightRemarks(shared_ptr<Flight> flight, const string& remarks);

        // Scope visibility management
        bool establishScopeVisibility(
            shared_ptr<ControllerPosition> controller,
            shared_ptr<ControllerPosition> targetController,
            VisibilityRule rule,
            bool bidirectional,
            chrono::microseconds timestamp
        );

        bool removeScopeVisibility(
            shared_ptr<ControllerPosition> controller,
            shared_ptr<ControllerPosition> targetController
        );

        // Data queries
        bool hasFlightData(shared_ptr<Flight> flight) const;
        const FlightScopeData* getFlightData(shared_ptr<Flight> flight) const;
        vector<shared_ptr<Flight>> getVisibleFlights(shared_ptr<ControllerPosition> controller) const;
        vector<shared_ptr<Flight>> getFlightsInSector(shared_ptr<ControllerPosition> controller) const;
        bool isFlightVisibleTo(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller) const;

        // Handoff support
        void transferFlightScope(
            shared_ptr<Flight> flight,
            shared_ptr<ControllerPosition> fromController,
            shared_ptr<ControllerPosition> toController,
            chrono::microseconds timestamp
        );

        void initiateHandoffScopeTransfer(
            shared_ptr<Flight> flight,
            shared_ptr<ControllerPosition> fromController,
            shared_ptr<ControllerPosition> toController,
            chrono::microseconds timestamp
        );

        void completeHandoffScopeTransfer(
            shared_ptr<Flight> flight,
            shared_ptr<ControllerPosition> toController,
            chrono::microseconds timestamp
        );

        // Timeout processing
        void processTimeouts(chrono::microseconds timestamp);
        void clearStaleData(chrono::microseconds timestamp);

        // Configuration
        void setDataFreshnessTimeout(chrono::milliseconds timeout) { m_dataFreshnessTimeoutMs = timeout; }
        void setVisibilityTimeout(chrono::milliseconds timeout) { m_visibilityTimeoutMs = timeout; }

        // Statistics
        int trackedFlightCount() const { return static_cast<int>(m_flightScopeData.size()); }
        int visibleFlightCount(shared_ptr<ControllerPosition> controller) const;
        int scopeVisibilityCount() const { return static_cast<int>(m_scopeVisibilities.size()); }

    private:
        FlightScopeData* findFlightScopeData(shared_ptr<Flight> flight);
        const FlightScopeData* findFlightScopeData(shared_ptr<Flight> flight) const;
        bool isDataStale(const FlightScopeData& data, chrono::microseconds timestamp) const;
        bool hasScopeVisibility(shared_ptr<ControllerPosition> controller, shared_ptr<ControllerPosition> target) const;
        void removeFlightFromController(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller);
        void addFlightToController(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller);
        void logScopeEvent(const string& event, shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller) const;
    };

    // ============================================================================
    // SharedRadarScopeFactory - Factory for scope operations
    // ============================================================================

    class SharedRadarScopeFactory
    {
    private:
        shared_ptr<HostServices> m_host;
        shared_ptr<SharedRadarScope> m_scope;

    public:
        SharedRadarScopeFactory(shared_ptr<HostServices> _host, shared_ptr<SharedRadarScope> _scope);

    public:
        void updatePosition(
            shared_ptr<Flight> flight,
            shared_ptr<ControllerPosition> controller,
            const GeoPoint& position,
            float altitudeFeet,
            float groundSpeedKt,
            float headingDegrees,
            chrono::microseconds timestamp
        );

        void transferScope(
            shared_ptr<Flight> flight,
            shared_ptr<ControllerPosition> fromController,
            shared_ptr<ControllerPosition> toController,
            chrono::microseconds timestamp
        );

        void setEmergency(shared_ptr<Flight> flight, bool isEmergency);
        void setRadarContact(shared_ptr<Flight> flight, bool hasContact);
    };
}
