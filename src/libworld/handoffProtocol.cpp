// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#include "handoffProtocol.hpp"
#include "workloadModel.hpp"

using namespace std;

namespace world
{
    HandoffProtocol::HandoffProtocol(shared_ptr<HostServices> _host, shared_ptr<WorkloadModel> _workloadModel)
        : m_host(_host),
          m_workloadModel(_workloadModel),
          m_requestTimeoutMs(chrono::seconds(30)),
          m_acceptanceTimeoutMs(chrono::seconds(10)),
          m_handoffCompleteTimeoutMs(chrono::seconds(60)),
          m_maxRetries(3)
    {
    }

    HandoffProtocol::~HandoffProtocol()
    {
    }

    bool HandoffProtocol::initiateHandoff(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        HandoffType type,
        chrono::microseconds timestamp
    )
    {
        if (!sender || !recipient || !flight)
        {
            return false;
        }

        // Check if there's already a pending handoff for this flight
        auto existing = findHandoffRecord(flight);
        if (existing && existing->state != State::HandoffComplete && existing->state != State::HandoffFailed && existing->state != State::Cancelled)
        {
            m_host->writeLog(
                "HANDOFF|Flight[%s] already has pending handoff in state[%d]",
                flight->callSign().c_str(),
                (int)existing->state
            );
            return false;
        }

        // Check if recipient can accept the handoff
        if (!canAcceptHandoff(recipient, flight))
        {
            m_host->writeLog(
                "HANDOFF|Recipient[%s] cannot accept handoff for flight[%s]",
                recipient->callSign().c_str(),
                flight->callSign().c_str()
            );
            return false;
        }

        HandoffRecord record;
        record.messageId = generateMessageId();
        record.state = State::RequestSent;
        record.type = type;
        record.sender = sender;
        record.recipient = recipient;
        record.flight = flight;
        record.trafficFlight = nullptr;
        record.requestTimestamp = timestamp;
        record.responseTimestamp = chrono::microseconds(0);
        record.expiryTimestamp = timestamp + chrono::duration_cast<chrono::microseconds>(m_requestTimeoutMs);
        record.rejectionReason = CoordinationMessage::RejectionReason::None;
        record.retryCount = 0;
        record.remarks = "Handoff initiated";

        m_activeHandoffs[record.messageId] = record;
        m_pendingHandoffByFlightId[flight->id()] = recipient;

        logHandoffEvent("INITIATED", record);
        return true;
    }

    bool HandoffProtocol::acceptHandoff(
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (!recipient || !flight)
        {
            return false;
        }

        auto record = findHandoffRecord(flight);
        if (!record)
        {
            m_host->writeLog(
                "HANDOFF|No pending handoff found for flight[%s]",
                flight->callSign().c_str()
            );
            return false;
        }

        if (record->recipient != recipient)
        {
            m_host->writeLog(
                "HANDOFF|Recipient mismatch for flight[%s]: expected[%s] got[%s]",
                flight->callSign().c_str(),
                record->recipient->callSign().c_str(),
                recipient->callSign().c_str()
            );
            return false;
        }

        if (record->state != State::RequestSent)
        {
            m_host->writeLog(
                "HANDOFF|Invalid state for acceptance: flight[%s] state[%d]",
                flight->callSign().c_str(),
                (int)record->state
            );
            return false;
        }

        record->state = State::RequestAccepted;
        record->responseTimestamp = timestamp;
        record->expiryTimestamp = timestamp + chrono::duration_cast<chrono::microseconds>(m_handoffCompleteTimeoutMs);

        logHandoffEvent("ACCEPTED", *record);
        return true;
    }

    bool HandoffProtocol::rejectHandoff(
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        CoordinationMessage::RejectionReason reason,
        chrono::microseconds timestamp
    )
    {
        if (!recipient || !flight)
        {
            return false;
        }

        auto record = findHandoffRecord(flight);
        if (!record)
        {
            m_host->writeLog(
                "HANDOFF|No pending handoff found for flight[%s]",
                flight->callSign().c_str()
            );
            return false;
        }

        if (record->recipient != recipient)
        {
            m_host->writeLog(
                "HANDOFF|Recipient mismatch for flight[%s]",
                flight->callSign().c_str()
            );
            return false;
        }

        if (record->state != State::RequestSent)
        {
            m_host->writeLog(
                "HANDOFF|Invalid state for rejection: flight[%s] state[%d]",
                flight->callSign().c_str(),
                (int)record->state
            );
            return false;
        }

        record->state = State::RequestRejected;
        record->responseTimestamp = timestamp;
        record->rejectionReason = reason;
        record->remarks = "Rejected: " + CoordinationMessage::rejectionReasonToString(reason);

        m_pendingHandoffByFlightId.erase(flight->id());

        logHandoffEvent("REJECTED", *record);
        return true;
    }

    bool HandoffProtocol::cancelHandoff(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (!sender || !flight)
        {
            return false;
        }

        auto record = findHandoffRecord(flight);
        if (!record)
        {
            return false;
        }

        if (record->sender != sender)
        {
            m_host->writeLog(
                "HANDOFF|Sender mismatch for flight[%s]",
                flight->callSign().c_str()
            );
            return false;
        }

        if (record->state != State::RequestSent && record->state != State::RequestAccepted)
        {
            return false;
        }

        record->state = State::Cancelled;
        record->responseTimestamp = timestamp;
        m_pendingHandoffByFlightId.erase(flight->id());

        logHandoffEvent("CANCELLED", *record);
        return true;
    }

    bool HandoffProtocol::initiatePointOut(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        shared_ptr<Flight> trafficFlight,
        chrono::microseconds timestamp
    )
    {
        if (!sender || !recipient || !flight || !trafficFlight)
        {
            return false;
        }

        HandoffRecord record;
        record.messageId = generateMessageId();
        record.state = State::RequestSent;
        record.type = HandoffType::PointOut;
        record.sender = sender;
        record.recipient = recipient;
        record.flight = flight;
        record.trafficFlight = trafficFlight;
        record.requestTimestamp = timestamp;
        record.responseTimestamp = chrono::microseconds(0);
        record.expiryTimestamp = timestamp + chrono::duration_cast<chrono::microseconds>(m_acceptanceTimeoutMs);
        record.rejectionReason = CoordinationMessage::RejectionReason::None;
        record.retryCount = 0;
        record.remarks = "Point out: " + flight->callSign() + " traffic " + trafficFlight->callSign();

        m_activeHandoffs[record.messageId] = record;
        logHandoffEvent("POINT_OUT", record);
        return true;
    }

    bool HandoffProtocol::acknowledgePointOut(
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (!recipient || !flight)
        {
            return false;
        }

        auto record = findHandoffRecord(flight);
        if (!record || record->type != HandoffType::PointOut)
        {
            return false;
        }

        if (record->recipient != recipient)
        {
            return false;
        }

        if (record->state != State::RequestSent)
        {
            return false;
        }

        record->state = State::HandoffComplete;
        record->responseTimestamp = timestamp;
        m_pendingHandoffByFlightId.erase(flight->id());

        logHandoffEvent("POINT_OUT_ACK", *record);
        return true;
    }

    bool HandoffProtocol::initiateEmergencyHandoff(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (!sender || !recipient || !flight)
        {
            return false;
        }

        // Emergency handoffs bypass normal checks
        HandoffRecord record;
        record.messageId = generateMessageId();
        record.state = State::HandoffInProgress;
        record.type = HandoffType::Emergency;
        record.sender = sender;
        record.recipient = recipient;
        record.flight = flight;
        record.trafficFlight = nullptr;
        record.requestTimestamp = timestamp;
        record.responseTimestamp = timestamp;
        record.expiryTimestamp = timestamp + chrono::duration_cast<chrono::microseconds>(m_handoffCompleteTimeoutMs);
        record.rejectionReason = CoordinationMessage::RejectionReason::None;
        record.retryCount = 0;
        record.remarks = "EMERGENCY handoff";

        m_activeHandoffs[record.messageId] = record;
        m_pendingHandoffByFlightId[flight->id()] = recipient;

        logHandoffEvent("EMERGENCY_INITIATED", record);
        return true;
    }

    HandoffProtocol::State HandoffProtocol::getHandoffState(shared_ptr<Flight> flight) const
    {
        if (!flight)
        {
            return State::Idle;
        }

        for (const auto& pair : m_activeHandoffs)
        {
            if (pair.second.flight == flight)
            {
                return pair.second.state;
            }
        }

        return State::Idle;
    }

    bool HandoffProtocol::hasPendingHandoff(shared_ptr<Flight> flight) const
    {
        if (!flight)
        {
            return false;
        }

        for (const auto& pair : m_activeHandoffs)
        {
            if (pair.second.flight == flight)
            {
                return pair.second.state == State::RequestSent || pair.second.state == State::RequestAccepted;
            }
        }

        return false;
    }

    shared_ptr<ControllerPosition> HandoffProtocol::getPendingHandoffRecipient(shared_ptr<Flight> flight) const
    {
        if (!flight)
        {
            return nullptr;
        }

        auto it = m_pendingHandoffByFlightId.find(flight->id());
        if (it != m_pendingHandoffByFlightId.end())
        {
            return it->second;
        }

        return nullptr;
    }

    chrono::microseconds HandoffProtocol::getHandoffRequestTime(shared_ptr<Flight> flight) const
    {
        if (!flight)
        {
            return chrono::microseconds(0);
        }

        for (const auto& pair : m_activeHandoffs)
        {
            if (pair.second.flight == flight)
            {
                return pair.second.requestTimestamp;
            }
        }

        return chrono::microseconds(0);
    }

    void HandoffProtocol::processTimeouts(chrono::microseconds timestamp)
    {
        vector<uint64_t> expiredIds;

        for (const auto& pair : m_activeHandoffs)
        {
            const auto& record = pair.second;
            if (isHandoffExpired(record, timestamp))
            {
                expiredIds.push_back(pair.first);
            }
        }

        for (uint64_t id : expiredIds)
        {
            auto it = m_activeHandoffs.find(id);
            if (it != m_activeHandoffs.end())
            {
                auto& record = it->second;
                
                if (record.state == State::RequestSent && record.retryCount < m_maxRetries)
                {
                    // Retry the handoff
                    record.retryCount++;
                    record.expiryTimestamp = timestamp + chrono::duration_cast<chrono::microseconds>(m_requestTimeoutMs);
                    record.remarks = "Retry " + to_string(record.retryCount) + " of " + to_string(m_maxRetries);
                    logHandoffEvent("RETRY", record);
                }
                else
                {
                    // Fail the handoff
                    failHandoff(record, timestamp, "Timeout");
                    m_pendingHandoffByFlightId.erase(record.flight->id());
                }
            }
        }
    }

    void HandoffProtocol::clearCompletedHandoffs()
    {
        vector<uint64_t> completedIds;

        for (const auto& pair : m_activeHandoffs)
        {
            if (pair.second.state == State::HandoffComplete || 
                pair.second.state == State::HandoffFailed ||
                pair.second.state == State::Cancelled ||
                pair.second.state == State::Timeout)
            {
                completedIds.push_back(pair.first);
            }
        }

        for (uint64_t id : completedIds)
        {
            m_activeHandoffs.erase(id);
        }
    }

    uint64_t HandoffProtocol::generateMessageId()
    {
        static uint64_t nextId = 1;
        return nextId++;
    }

    HandoffProtocol::HandoffRecord* HandoffProtocol::findHandoffRecord(shared_ptr<Flight> flight)
    {
        if (!flight)
        {
            return nullptr;
        }

        for (auto& pair : m_activeHandoffs)
        {
            if (pair.second.flight == flight)
            {
                return &pair.second;
            }
        }

        return nullptr;
    }

    const HandoffProtocol::HandoffRecord* HandoffProtocol::findHandoffRecord(shared_ptr<Flight> flight) const
    {
        if (!flight)
        {
            return nullptr;
        }

        for (const auto& pair : m_activeHandoffs)
        {
            if (pair.second.flight == flight)
            {
                return &pair.second;
            }
        }

        return nullptr;
    }

    bool HandoffProtocol::isHandoffExpired(const HandoffRecord& record, chrono::microseconds timestamp) const
    {
        return timestamp > record.expiryTimestamp;
    }

    bool HandoffProtocol::canAcceptHandoff(shared_ptr<ControllerPosition> recipient, shared_ptr<Flight> flight) const
    {
        if (!recipient || !flight)
        {
            return false;
        }

        // Check workload model if available
        if (m_workloadModel)
        {
            if (!m_workloadModel->canAcceptAdditionalAircraft(recipient))
            {
                return false;
            }
        }

        return true;
    }

    void HandoffProtocol::completeHandoff(HandoffRecord& record, chrono::microseconds timestamp)
    {
        record.state = State::HandoffComplete;
        record.responseTimestamp = timestamp;
        m_pendingHandoffByFlightId.erase(record.flight->id());
        logHandoffEvent("COMPLETED", record);
    }

    void HandoffProtocol::failHandoff(HandoffRecord& record, chrono::microseconds timestamp, const string& reason)
    {
        record.state = State::HandoffFailed;
        record.responseTimestamp = timestamp;
        record.remarks = reason;
        m_pendingHandoffByFlightId.erase(record.flight->id());
        logHandoffEvent("FAILED", record);
    }

    void HandoffProtocol::logHandoffEvent(const string& event, const HandoffRecord& record) const
    {
        if (!m_host)
        {
            return;
        }

        m_host->writeLog(
            "HANDOFF|%s flight[%s] sender[%s] recipient[%s] type[%d] state[%d]",
            event.c_str(),
            record.flight ? record.flight->callSign().c_str() : "N/A",
            record.sender ? record.sender->callSign().c_str() : "N/A",
            record.recipient ? record.recipient->callSign().c_str() : "N/A",
            (int)record.type,
            (int)record.state
        );
    }

    // HandoffProtocolFactory implementation

    HandoffProtocolFactory::HandoffProtocolFactory(
        shared_ptr<HostServices> _host,
        shared_ptr<CoordinationMessageFactory> _messageFactory
    ) : m_host(_host),
        m_messageFactory(_messageFactory)
    {
    }

    shared_ptr<CoordinationMessage> HandoffProtocolFactory::createHandoffRequestMessage(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (!m_messageFactory)
        {
            return nullptr;
        }
        return m_messageFactory->createHandoffRequest(sender, recipient, flight, timestamp);
    }

    shared_ptr<CoordinationMessage> HandoffProtocolFactory::createHandoffAcceptanceMessage(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (!m_messageFactory)
        {
            return nullptr;
        }
        return m_messageFactory->createHandoffAcceptance(sender, recipient, flight, timestamp);
    }

    shared_ptr<CoordinationMessage> HandoffProtocolFactory::createHandoffRejectionMessage(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        CoordinationMessage::RejectionReason reason,
        chrono::microseconds timestamp
    )
    {
        if (!m_messageFactory)
        {
            return nullptr;
        }
        return m_messageFactory->createHandoffRejection(sender, recipient, flight, reason, timestamp);
    }

    shared_ptr<CoordinationMessage> HandoffProtocolFactory::createHandoffCancellationMessage(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (!m_messageFactory)
        {
            return nullptr;
        }
        return m_messageFactory->createHandoffCancellation(sender, recipient, flight, timestamp);
    }

    shared_ptr<CoordinationMessage> HandoffProtocolFactory::createPointOutMessage(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        shared_ptr<Flight> trafficFlight,
        chrono::microseconds timestamp
    )
    {
        if (!m_messageFactory)
        {
            return nullptr;
        }
        return m_messageFactory->createPointOut(sender, recipient, flight, trafficFlight, timestamp);
    }

    shared_ptr<CoordinationMessage> HandoffProtocolFactory::createEmergencyHandoffMessage(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (!m_messageFactory)
        {
            return nullptr;
        }
        return m_messageFactory->createEmergencyHandoff(sender, recipient, flight, timestamp);
    }
}
