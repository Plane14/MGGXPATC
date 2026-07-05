// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#pragma once

#include "libworld.h"
#include "coordinationMessage.hpp"
#include <chrono>
#include <unordered_map>

namespace world
{
    // Forward declarations
    class ControllerPosition;
    class Flight;
    class WorkloadModel;

    // ============================================================================
    // HandoffProtocol - Handoff state machine between controllers
    // ============================================================================

    class HandoffProtocol
    {
    public:
        enum class State
        {
            Idle = 0,
            RequestSent = 1,
            RequestAccepted = 2,
            RequestRejected = 3,
            HandoffInProgress = 4,
            HandoffComplete = 5,
            HandoffFailed = 6,
            Cancelled = 7,
            Timeout = 8
        };

        enum class HandoffType
        {
            Normal = 0,
            PointOut = 1,
            Emergency = 2,
            ScopeTransfer = 3,
            InterFacility = 4
        };

    private:
        struct HandoffRecord
        {
            uint64_t messageId;
            State state;
            HandoffType type;
            shared_ptr<ControllerPosition> sender;
            shared_ptr<ControllerPosition> recipient;
            shared_ptr<Flight> flight;
            shared_ptr<Flight> trafficFlight;
            chrono::microseconds requestTimestamp;
            chrono::microseconds responseTimestamp;
            chrono::microseconds expiryTimestamp;
            CoordinationMessage::RejectionReason rejectionReason;
            int retryCount;
            string remarks;
        };

    private:
        shared_ptr<HostServices> m_host;
        shared_ptr<WorkloadModel> m_workloadModel;
        unordered_map<uint64_t, HandoffRecord> m_activeHandoffs;
        unordered_map<int, shared_ptr<ControllerPosition>> m_pendingHandoffByFlightId;
        chrono::milliseconds m_requestTimeoutMs;
        chrono::milliseconds m_acceptanceTimeoutMs;
        chrono::milliseconds m_handoffCompleteTimeoutMs;
        int m_maxRetries;

    public:
        HandoffProtocol(shared_ptr<HostServices> _host, shared_ptr<WorkloadModel> _workloadModel);
        ~HandoffProtocol();

    public:
        // Main handoff flow
        bool initiateHandoff(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            HandoffType type,
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
        bool initiatePointOut(
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

        // Emergency handoff
        bool initiateEmergencyHandoff(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        // Query state
        State getHandoffState(shared_ptr<Flight> flight) const;
        bool hasPendingHandoff(shared_ptr<Flight> flight) const;
        shared_ptr<ControllerPosition> getPendingHandoffRecipient(shared_ptr<Flight> flight) const;
        chrono::microseconds getHandoffRequestTime(shared_ptr<Flight> flight) const;

        // Timeout processing
        void processTimeouts(chrono::microseconds timestamp);
        void clearCompletedHandoffs();

        // Configuration
        void setRequestTimeout(chrono::milliseconds timeout) { m_requestTimeoutMs = timeout; }
        void setAcceptanceTimeout(chrono::milliseconds timeout) { m_acceptanceTimeoutMs = timeout; }
        void setHandoffCompleteTimeout(chrono::milliseconds timeout) { m_handoffCompleteTimeoutMs = timeout; }
        void setMaxRetries(int maxRetries) { m_maxRetries = maxRetries; }

        // Statistics
        int activeHandoffCount() const { return static_cast<int>(m_activeHandoffs.size()); }
        int pendingHandoffCount() const { return static_cast<int>(m_pendingHandoffByFlightId.size()); }

    private:
        uint64_t generateMessageId();
        HandoffRecord* findHandoffRecord(shared_ptr<Flight> flight);
        const HandoffRecord* findHandoffRecord(shared_ptr<Flight> flight) const;
        bool isHandoffExpired(const HandoffRecord& record, chrono::microseconds timestamp) const;
        bool canAcceptHandoff(shared_ptr<ControllerPosition> recipient, shared_ptr<Flight> flight) const;
        void completeHandoff(HandoffRecord& record, chrono::microseconds timestamp);
        void failHandoff(HandoffRecord& record, chrono::microseconds timestamp, const string& reason);
        void logHandoffEvent(const string& event, const HandoffRecord& record) const;
    };

    // ============================================================================
    // HandoffProtocolFactory - Factory for creating handoff messages
    // ============================================================================

    class HandoffProtocolFactory
    {
    private:
        shared_ptr<HostServices> m_host;
        shared_ptr<CoordinationMessageFactory> m_messageFactory;

    public:
        HandoffProtocolFactory(shared_ptr<HostServices> _host, shared_ptr<CoordinationMessageFactory> _messageFactory);

    public:
        shared_ptr<CoordinationMessage> createHandoffRequestMessage(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        shared_ptr<CoordinationMessage> createHandoffAcceptanceMessage(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        shared_ptr<CoordinationMessage> createHandoffRejectionMessage(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            CoordinationMessage::RejectionReason reason,
            chrono::microseconds timestamp
        );

        shared_ptr<CoordinationMessage> createHandoffCancellationMessage(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        shared_ptr<CoordinationMessage> createPointOutMessage(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            shared_ptr<Flight> trafficFlight,
            chrono::microseconds timestamp
        );

        shared_ptr<CoordinationMessage> createEmergencyHandoffMessage(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );
    };
}
