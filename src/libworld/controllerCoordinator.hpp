// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#pragma once

#include "libworld.h"
#include "handoffProtocol.hpp"
#include "sharedRadarScope.hpp"
#include "coordinationMessage.hpp"
#include "workloadModel.hpp"
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <functional>

namespace world
{
    // Forward declarations
    class ControllerPosition;
    class Flight;
    class ControlFacility;

    // ============================================================================
    // ControllerCoordinator - Multi-controller coordination system
    // ============================================================================

    class ControllerCoordinator
    {
    public:
        enum class CoordinationEvent
        {
            HandoffRequested = 0,
            HandoffAccepted = 1,
            HandoffRejected = 2,
            HandoffCompleted = 3,
            HandoffCancelled = 4,
            HandoffFailed = 5,
            PointOutSent = 6,
            PointOutAcknowledged = 7,
            TrafficAdvisorySent = 8,
            ScopeTransferred = 9,
            EmergencyHandoff = 10,
            WorkloadWarning = 11,
            WorkloadOverload = 12
        };

        using EventCallback = function<void(CoordinationEvent event, shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller, const string& details)>;

    private:
        struct FacilityCoordinationLink
        {
            shared_ptr<ControlFacility> facility;
            vector<shared_ptr<ControllerPosition>> adjacentPositions;
            bool isActive;
            chrono::microseconds lastCommunicationTimestamp;
        };

    private:
        shared_ptr<HostServices> m_host;
        shared_ptr<HandoffProtocol> m_handoffProtocol;
        shared_ptr<SharedRadarScope> m_sharedRadarScope;
        shared_ptr<CoordinationMessageFactory> m_messageFactory;
        shared_ptr<WorkloadModel> m_workloadModel;
        vector<FacilityCoordinationLink> m_facilityLinks;
        unordered_map<shared_ptr<ControllerPosition>, vector<shared_ptr<ControllerPosition>>> m_adjacentControllers;
        EventCallback m_eventCallback;
        chrono::milliseconds m_coordinationIntervalMs;
        bool m_isEnabled;

    public:
        ControllerCoordinator(shared_ptr<HostServices> _host);
        ~ControllerCoordinator();

    public:
        // Initialization
        void initialize(
            shared_ptr<HandoffProtocol> handoffProtocol,
            shared_ptr<SharedRadarScope> sharedRadarScope,
            shared_ptr<CoordinationMessageFactory> messageFactory,
            shared_ptr<WorkloadModel> workloadModel
        );

        void registerFacility(shared_ptr<ControlFacility> facility);
        void registerAdjacentControllers(
            shared_ptr<ControllerPosition> controller,
            const vector<shared_ptr<ControllerPosition>>& adjacentControllers
        );
        void setEventCallback(EventCallback callback) { m_eventCallback = callback; }

        // Enable/disable coordination
        void setEnabled(bool enabled) { m_isEnabled = enabled; }
        bool isEnabled() const { return m_isEnabled; }

        // Main coordination loop
        void progressTo(chrono::microseconds timestamp);
        void processCoordination(chrono::microseconds timestamp);

        // Handoff coordination
        bool requestHandoff(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        bool acceptHandoff(
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        bool rejectHandoff(
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            CoordinationMessage::RejectionReason reason,
            chrono::microseconds timestamp
        );

        bool cancelHandoff(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        // Point-out coordination
        bool sendPointOut(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            shared_ptr<Flight> trafficFlight,
            chrono::microseconds timestamp
        );

        bool acknowledgePointOut(
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        // Traffic advisory
        bool sendTrafficAdvisory(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            shared_ptr<Flight> trafficFlight,
            chrono::microseconds timestamp
        );

        // Emergency coordination
        bool requestEmergencyHandoff(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        // Scope management
        void establishScopeVisibility(
            shared_ptr<ControllerPosition> controller,
            shared_ptr<ControllerPosition> targetController,
            SharedRadarScope::VisibilityRule rule,
            bool bidirectional,
            chrono::microseconds timestamp
        );

        void transferFlightScope(
            shared_ptr<Flight> flight,
            shared_ptr<ControllerPosition> fromController,
            shared_ptr<ControllerPosition> toController,
            chrono::microseconds timestamp
        );

        // Workload-based coordination
        bool shouldRequestHandoff(shared_ptr<ControllerPosition> sender, shared_ptr<Flight> flight) const;
        shared_ptr<ControllerPosition> findBestHandoffRecipient(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> sender) const;
        vector<shared_ptr<ControllerPosition>> getAvailableRecipients(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> sender) const;

        // Query state
        HandoffProtocol::State getHandoffState(shared_ptr<Flight> flight) const;
        bool hasPendingHandoff(shared_ptr<Flight> flight) const;
        shared_ptr<ControllerPosition> getPendingHandoffRecipient(shared_ptr<Flight> flight) const;
        WorkloadModel::WorkloadLevel getControllerWorkload(shared_ptr<ControllerPosition> controller) const;
        bool isFlightVisibleTo(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller) const;

        // Timeout processing
        void processTimeouts(chrono::microseconds timestamp);
        void clearCompletedHandoffs();

        // Configuration
        void setCoordinationInterval(chrono::milliseconds interval) { m_coordinationIntervalMs = interval; }
        void setHandoffTimeouts(
            chrono::milliseconds requestTimeout,
            chrono::milliseconds acceptanceTimeout,
            chrono::milliseconds completeTimeout
        );

        // Statistics
        int activeHandoffCount() const;
        int pendingHandoffCount() const;
        int overloadedControllerCount() const;
        vector<shared_ptr<ControllerPosition>> getOverloadedControllers() const;

    private:
        void fireEvent(CoordinationEvent event, shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller, const string& details);
        void processWorkloadWarnings(chrono::microseconds timestamp);
        void processPendingHandoffs(chrono::microseconds timestamp);
        void updateScopeVisibility(chrono::microseconds timestamp);
        shared_ptr<ControllerPosition> resolveNextControllerBySectorOwnership(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> sender) const;
        bool isFlightOwnedByControllerSector(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller) const;
        bool isWithinFrequencyCoverage(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller) const;
        float altitudeMslFeet(shared_ptr<Flight> flight) const;
        void logCoordinationEvent(const string& event, const string& details) const;
    };

    // ============================================================================
    // ControllerCoordinatorFactory - Factory for coordination operations
    // ============================================================================

    class ControllerCoordinatorFactory
    {
    private:
        shared_ptr<HostServices> m_host;
        shared_ptr<ControllerCoordinator> m_coordinator;

    public:
        ControllerCoordinatorFactory(shared_ptr<HostServices> _host, shared_ptr<ControllerCoordinator> _coordinator);

    public:
        bool requestHandoff(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        bool acceptHandoff(
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        bool rejectHandoff(
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            CoordinationMessage::RejectionReason reason,
            chrono::microseconds timestamp
        );

        bool sendPointOut(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            shared_ptr<Flight> trafficFlight,
            chrono::microseconds timestamp
        );

        bool requestEmergencyHandoff(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        void transferScope(
            shared_ptr<Flight> flight,
            shared_ptr<ControllerPosition> fromController,
            shared_ptr<ControllerPosition> toController,
            chrono::microseconds timestamp
        );
    };
}
