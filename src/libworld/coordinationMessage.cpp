// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#include "coordinationMessage.hpp"

using namespace std;

namespace world
{
    string CoordinationMessage::typeToString(Type type)
    {
        switch (type)
        {
        case Type::HandoffRequest: return "HandoffRequest";
        case Type::HandoffAcceptance: return "HandoffAcceptance";
        case Type::HandoffRejection: return "HandoffRejection";
        case Type::HandoffCancellation: return "HandoffCancellation";
        case Type::CoordinationRequest: return "CoordinationRequest";
        case Type::CoordinationReply: return "CoordinationReply";
        case Type::CoordinationAlert: return "CoordinationAlert";
        case Type::PointOut: return "PointOut";
        case Type::PointOutAcknowledgment: return "PointOutAcknowledgment";
        case Type::TrafficAdvisory: return "TrafficAdvisory";
        case Type::TrafficAdvisoryAcknowledgment: return "TrafficAdvisoryAcknowledgment";
        case Type::EmergencyHandoff: return "EmergencyHandoff";
        case Type::EmergencyHandoffAcceptance: return "EmergencyHandoffAcceptance";
        case Type::ScopeTransfer: return "ScopeTransfer";
        case Type::ScopeTransferAcknowledgment: return "ScopeTransferAcknowledgment";
        default: return "Unknown";
        }
    }

    string CoordinationMessage::priorityToString(Priority priority)
    {
        switch (priority)
        {
        case Priority::Normal: return "Normal";
        case Priority::Urgent: return "Urgent";
        case Priority::Emergency: return "Emergency";
        default: return "Unknown";
        }
    }

    string CoordinationMessage::rejectionReasonToString(RejectionReason reason)
    {
        switch (reason)
        {
        case RejectionReason::None: return "None";
        case RejectionReason::WorkloadTooHigh: return "WorkloadTooHigh";
        case RejectionReason::SectorFull: return "SectorFull";
        case RejectionReason::AircraftNotInSector: return "AircraftNotInSector";
        case RejectionReason::FrequencyUnavailable: return "FrequencyUnavailable";
        case RejectionReason::EquipmentFailure: return "EquipmentFailure";
        case RejectionReason::Other: return "Other";
        default: return "Unknown";
        }
    }

    shared_ptr<CoordinationMessage> CoordinationMessageFactory::createHandoffRequest(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        auto message = make_shared<CoordinationMessage>(
            generateId(),
            CoordinationMessage::Type::HandoffRequest,
            CoordinationMessage::Priority::Normal,
            sender,
            recipient,
            flight,
            timestamp
        );
        message->setRemarks("Handoff request for " + flight->callSign());
        return message;
    }

    shared_ptr<CoordinationMessage> CoordinationMessageFactory::createHandoffAcceptance(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        auto message = make_shared<CoordinationMessage>(
            generateId(),
            CoordinationMessage::Type::HandoffAcceptance,
            CoordinationMessage::Priority::Normal,
            sender,
            recipient,
            flight,
            timestamp
        );
        message->setAcknowledged(true);
        message->setRemarks("Handoff accepted for " + flight->callSign());
        return message;
    }

    shared_ptr<CoordinationMessage> CoordinationMessageFactory::createHandoffRejection(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        CoordinationMessage::RejectionReason reason,
        chrono::microseconds timestamp
    )
    {
        auto message = make_shared<CoordinationMessage>(
            generateId(),
            CoordinationMessage::Type::HandoffRejection,
            CoordinationMessage::Priority::Urgent,
            sender,
            recipient,
            flight,
            timestamp
        );
        message->setRejectionReason(reason);
        message->setRemarks("Handoff rejected for " + flight->callSign() + ": " + CoordinationMessage::rejectionReasonToString(reason));
        return message;
    }

    shared_ptr<CoordinationMessage> CoordinationMessageFactory::createHandoffCancellation(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        auto message = make_shared<CoordinationMessage>(
            generateId(),
            CoordinationMessage::Type::HandoffCancellation,
            CoordinationMessage::Priority::Normal,
            sender,
            recipient,
            flight,
            timestamp
        );
        message->setRemarks("Handoff cancelled for " + flight->callSign());
        return message;
    }

    shared_ptr<CoordinationMessage> CoordinationMessageFactory::createPointOut(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        shared_ptr<Flight> trafficFlight,
        chrono::microseconds timestamp
    )
    {
        auto message = make_shared<CoordinationMessage>(
            generateId(),
            CoordinationMessage::Type::PointOut,
            CoordinationMessage::Priority::Normal,
            sender,
            recipient,
            flight,
            trafficFlight,
            timestamp
        );
        message->setRemarks("Point out: " + flight->callSign() + " traffic " + trafficFlight->callSign());
        return message;
    }

    shared_ptr<CoordinationMessage> CoordinationMessageFactory::createPointOutAcknowledgment(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        auto message = make_shared<CoordinationMessage>(
            generateId(),
            CoordinationMessage::Type::PointOutAcknowledgment,
            CoordinationMessage::Priority::Normal,
            sender,
            recipient,
            flight,
            timestamp
        );
        message->setAcknowledged(true);
        message->setRemarks("Point out acknowledged for " + flight->callSign());
        return message;
    }

    shared_ptr<CoordinationMessage> CoordinationMessageFactory::createTrafficAdvisory(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        shared_ptr<Flight> trafficFlight,
        chrono::microseconds timestamp
    )
    {
        auto message = make_shared<CoordinationMessage>(
            generateId(),
            CoordinationMessage::Type::TrafficAdvisory,
            CoordinationMessage::Priority::Urgent,
            sender,
            recipient,
            flight,
            trafficFlight,
            timestamp
        );
        message->setRemarks("Traffic advisory: " + flight->callSign() + " traffic " + trafficFlight->callSign());
        return message;
    }

    shared_ptr<CoordinationMessage> CoordinationMessageFactory::createEmergencyHandoff(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        auto message = make_shared<CoordinationMessage>(
            generateId(),
            CoordinationMessage::Type::EmergencyHandoff,
            CoordinationMessage::Priority::Emergency,
            sender,
            recipient,
            flight,
            timestamp
        );
        message->setExpiryTimestamp(timestamp + chrono::seconds(5));
        message->setRemarks("EMERGENCY handoff for " + flight->callSign());
        return message;
    }

    shared_ptr<CoordinationMessage> CoordinationMessageFactory::createScopeTransfer(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        auto message = make_shared<CoordinationMessage>(
            generateId(),
            CoordinationMessage::Type::ScopeTransfer,
            CoordinationMessage::Priority::Normal,
            sender,
            recipient,
            flight,
            timestamp
        );
        message->setRemarks("Scope transfer for " + flight->callSign());
        return message;
    }
}
