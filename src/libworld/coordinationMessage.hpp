// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#pragma once

#include "libworld.h"
#include <chrono>

namespace world
{
    // Forward declarations
    class ControllerPosition;
    class Flight;

    // ============================================================================
    // CoordinationMessage - Inter-facility coordination messages
    // ============================================================================

    class CoordinationMessage
    {
    public:
        enum class Type
        {
            Unknown = 0,
            HandoffRequest = 1,
            HandoffAcceptance = 2,
            HandoffRejection = 3,
            HandoffCancellation = 4,
            CoordinationRequest = 5,
            CoordinationReply = 6,
            CoordinationAlert = 7,
            PointOut = 8,
            PointOutAcknowledgment = 9,
            TrafficAdvisory = 10,
            TrafficAdvisoryAcknowledgment = 11,
            EmergencyHandoff = 12,
            EmergencyHandoffAcceptance = 13,
            ScopeTransfer = 14,
            ScopeTransferAcknowledgment = 15
        };

        enum class Priority
        {
            Normal = 0,
            Urgent = 1,
            Emergency = 2
        };

        enum class RejectionReason
        {
            None = 0,
            WorkloadTooHigh = 1,
            SectorFull = 2,
            AircraftNotInSector = 3,
            FrequencyUnavailable = 4,
            EquipmentFailure = 5,
            Other = 6
        };

    private:
        uint64_t m_id;
        Type m_type;
        Priority m_priority;
        shared_ptr<ControllerPosition> m_sender;
        shared_ptr<ControllerPosition> m_recipient;
        shared_ptr<Flight> m_subjectFlight;
        shared_ptr<Flight> m_subjectFlight2;
        chrono::microseconds m_timestamp;
        chrono::microseconds m_expiryTimestamp;
        string m_remarks;
        RejectionReason m_rejectionReason;
        bool m_requiresAcknowledgment;
        bool m_isAcknowledged;
        int m_retryCount;
        int m_maxRetries;

    public:
        CoordinationMessage(
            uint64_t _id,
            Type _type,
            Priority _priority,
            shared_ptr<ControllerPosition> _sender,
            shared_ptr<ControllerPosition> _recipient,
            shared_ptr<Flight> _subjectFlight,
            chrono::microseconds _timestamp
        ) : m_id(_id),
            m_type(_type),
            m_priority(_priority),
            m_sender(_sender),
            m_recipient(_recipient),
            m_subjectFlight(_subjectFlight),
            m_subjectFlight2(nullptr),
            m_timestamp(_timestamp),
            m_expiryTimestamp(_timestamp + chrono::seconds(30)),
            m_rejectionReason(RejectionReason::None),
            m_requiresAcknowledgment(true),
            m_isAcknowledged(false),
            m_retryCount(0),
            m_maxRetries(3)
        {
        }

        CoordinationMessage(
            uint64_t _id,
            Type _type,
            Priority _priority,
            shared_ptr<ControllerPosition> _sender,
            shared_ptr<ControllerPosition> _recipient,
            shared_ptr<Flight> _subjectFlight,
            shared_ptr<Flight> _subjectFlight2,
            chrono::microseconds _timestamp
        ) : m_id(_id),
            m_type(_type),
            m_priority(_priority),
            m_sender(_sender),
            m_recipient(_recipient),
            m_subjectFlight(_subjectFlight),
            m_subjectFlight2(_subjectFlight2),
            m_timestamp(_timestamp),
            m_expiryTimestamp(_timestamp + chrono::seconds(30)),
            m_rejectionReason(RejectionReason::None),
            m_requiresAcknowledgment(true),
            m_isAcknowledged(false),
            m_retryCount(0),
            m_maxRetries(3)
        {
        }

    public:
        uint64_t id() const { return m_id; }
        Type type() const { return m_type; }
        Priority priority() const { return m_priority; }
        shared_ptr<ControllerPosition> sender() const { return m_sender; }
        shared_ptr<ControllerPosition> recipient() const { return m_recipient; }
        shared_ptr<Flight> subjectFlight() const { return m_subjectFlight; }
        shared_ptr<Flight> subjectFlight2() const { return m_subjectFlight2; }
        chrono::microseconds timestamp() const { return m_timestamp; }
        chrono::microseconds expiryTimestamp() const { return m_expiryTimestamp; }
        const string& remarks() const { return m_remarks; }
        RejectionReason rejectionReason() const { return m_rejectionReason; }
        bool requiresAcknowledgment() const { return m_requiresAcknowledgment; }
        bool isAcknowledged() const { return m_isAcknowledged; }
        int retryCount() const { return m_retryCount; }
        int maxRetries() const { return m_maxRetries; }

    public:
        void setRemarks(const string& _remarks) { m_remarks = _remarks; }
        void setRejectionReason(RejectionReason _reason) { m_rejectionReason = _reason; }
        void setRequiresAcknowledgment(bool _requires) { m_requiresAcknowledgment = _requires; }
        void setAcknowledged(bool _ack) { m_isAcknowledged = _ack; }
        void setExpiryTimestamp(chrono::microseconds _expiry) { m_expiryTimestamp = _expiry; }
        void incrementRetryCount() { m_retryCount++; }
        bool isExpired(chrono::microseconds currentTimestamp) const
        {
            return currentTimestamp > m_expiryTimestamp;
        }
        bool canRetry() const
        {
            return m_retryCount < m_maxRetries;
        }

    public:
        static string typeToString(Type type);
        static string priorityToString(Priority priority);
        static string rejectionReasonToString(RejectionReason reason);
    };

    // ============================================================================
    // CoordinationMessageFactory - Factory for creating coordination messages
    // ============================================================================

    class CoordinationMessageFactory
    {
    private:
        uint64_t m_nextId = 1;
        shared_ptr<HostServices> m_host;

    public:
        CoordinationMessageFactory(shared_ptr<HostServices> _host) : m_host(_host) {}

    public:
        shared_ptr<CoordinationMessage> createHandoffRequest(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        shared_ptr<CoordinationMessage> createHandoffAcceptance(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        shared_ptr<CoordinationMessage> createHandoffRejection(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            CoordinationMessage::RejectionReason reason,
            chrono::microseconds timestamp
        );

        shared_ptr<CoordinationMessage> createHandoffCancellation(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        shared_ptr<CoordinationMessage> createPointOut(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            shared_ptr<Flight> trafficFlight,
            chrono::microseconds timestamp
        );

        shared_ptr<CoordinationMessage> createPointOutAcknowledgment(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        shared_ptr<CoordinationMessage> createTrafficAdvisory(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            shared_ptr<Flight> trafficFlight,
            chrono::microseconds timestamp
        );

        shared_ptr<CoordinationMessage> createEmergencyHandoff(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

        shared_ptr<CoordinationMessage> createScopeTransfer(
            shared_ptr<ControllerPosition> sender,
            shared_ptr<ControllerPosition> recipient,
            shared_ptr<Flight> flight,
            chrono::microseconds timestamp
        );

    private:
        uint64_t generateId()
        {
            return m_nextId++;
        }
    };
}
