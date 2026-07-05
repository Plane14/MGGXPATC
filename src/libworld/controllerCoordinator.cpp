// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#include "controllerCoordinator.hpp"
#include "workloadModel.hpp"

using namespace std;

namespace world
{
    ControllerCoordinator::ControllerCoordinator(shared_ptr<HostServices> _host)
        : m_host(_host),
          m_handoffProtocol(nullptr),
          m_sharedRadarScope(nullptr),
          m_messageFactory(nullptr),
          m_workloadModel(nullptr),
          m_coordinationIntervalMs(chrono::seconds(5)),
          m_isEnabled(true)
    {
    }

    ControllerCoordinator::~ControllerCoordinator()
    {
    }

    void ControllerCoordinator::initialize(
        shared_ptr<HandoffProtocol> handoffProtocol,
        shared_ptr<SharedRadarScope> sharedRadarScope,
        shared_ptr<CoordinationMessageFactory> messageFactory,
        shared_ptr<WorkloadModel> workloadModel
    )
    {
        m_handoffProtocol = handoffProtocol;
        m_sharedRadarScope = sharedRadarScope;
        m_messageFactory = messageFactory;
        m_workloadModel = workloadModel;
    }

    void ControllerCoordinator::registerFacility(shared_ptr<ControlFacility> facility)
    {
        if (!facility)
        {
            return;
        }

        FacilityCoordinationLink link;
        link.facility = facility;
        link.isActive = true;
        link.lastCommunicationTimestamp = chrono::microseconds(0);

        // Register all positions as adjacent to each other within the facility
        for (const auto& position : facility->positions())
        {
            for (const auto& otherPosition : facility->positions())
            {
                if (position != otherPosition)
                {
                    m_adjacentControllers[position].push_back(otherPosition);
                }
            }
        }

        m_facilityLinks.push_back(link);
        logCoordinationEvent("FACILITY_REGISTERED", facility->callSign());
    }

    void ControllerCoordinator::registerAdjacentControllers(
        shared_ptr<ControllerPosition> controller,
        const vector<shared_ptr<ControllerPosition>>& adjacentControllers
    )
    {
        if (!controller)
        {
            return;
        }

        m_adjacentControllers[controller] = adjacentControllers;

        // Establish scope visibility with all adjacent controllers
        if (m_sharedRadarScope)
        {
            for (const auto& adjacent : adjacentControllers)
            {
                m_sharedRadarScope->establishScopeVisibility(
                    controller,
                    adjacent,
                    SharedRadarScope::VisibilityRule::Full,
                    true,
                    chrono::microseconds(0)
                );
            }
        }

        logCoordinationEvent("ADJACENT_REGISTERED", controller->callSign());
    }

    void ControllerCoordinator::progressTo(chrono::microseconds timestamp)
    {
        if (!m_isEnabled)
        {
            return;
        }

        processCoordination(timestamp);
        processTimeouts(timestamp);
    }

    void ControllerCoordinator::processCoordination(chrono::microseconds timestamp)
    {
        processWorkloadWarnings(timestamp);
        processPendingHandoffs(timestamp);
        updateScopeVisibility(timestamp);
    }

    bool ControllerCoordinator::requestHandoff(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (!m_handoffProtocol || !sender || !recipient || !flight)
        {
            return false;
        }

        // Check if sender is overloaded
        if (m_workloadModel && m_workloadModel->isOverloaded(sender))
        {
            logCoordinationEvent("HANDOFF_DENIED_OVERLOADED", flight->callSign());
            return false;
        }

        bool result = m_handoffProtocol->initiateHandoff(
            sender,
            recipient,
            flight,
            HandoffProtocol::HandoffType::Normal,
            timestamp
        );

        if (result)
        {
            // Update shared radar scope
            if (m_sharedRadarScope)
            {
                m_sharedRadarScope->initiateHandoffScopeTransfer(flight, sender, recipient, timestamp);
            }

            // Update workload model
            if (m_workloadModel)
            {
                m_workloadModel->transferAircraft(flight, sender, recipient);
            }

            fireEvent(CoordinationEvent::HandoffRequested, flight, recipient, "Handoff requested");
            logCoordinationEvent("HANDOFF_REQUESTED", flight->callSign() + " from " + sender->callSign() + " to " + recipient->callSign());
        }

        return result;
    }

    bool ControllerCoordinator::acceptHandoff(
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (!m_handoffProtocol || !recipient || !flight)
        {
            return false;
        }

        // Check if recipient can accept
        if (m_workloadModel && !m_workloadModel->canAcceptAdditionalAircraft(recipient))
        {
            logCoordinationEvent("HANDOFF_REJECTED_CAPACITY", flight->callSign());
            return false;
        }

        bool result = m_handoffProtocol->acceptHandoff(recipient, flight, timestamp);

        if (result)
        {
            // Complete scope transfer
            if (m_sharedRadarScope)
            {
                m_sharedRadarScope->completeHandoffScopeTransfer(flight, recipient, timestamp);
            }

            fireEvent(CoordinationEvent::HandoffAccepted, flight, recipient, "Handoff accepted");
            logCoordinationEvent("HANDOFF_ACCEPTED", flight->callSign() + " by " + recipient->callSign());
        }

        return result;
    }

    bool ControllerCoordinator::rejectHandoff(
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        CoordinationMessage::RejectionReason reason,
        chrono::microseconds timestamp
    )
    {
        if (!m_handoffProtocol || !recipient || !flight)
        {
            return false;
        }

        bool result = m_handoffProtocol->rejectHandoff(recipient, flight, reason, timestamp);

        if (result)
        {
            fireEvent(CoordinationEvent::HandoffRejected, flight, recipient, "Handoff rejected: " + CoordinationMessage::rejectionReasonToString(reason));
            logCoordinationEvent("HANDOFF_REJECTED", flight->callSign() + " by " + recipient->callSign());
        }

        return result;
    }

    bool ControllerCoordinator::cancelHandoff(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (!m_handoffProtocol || !sender || !flight)
        {
            return false;
        }

        bool result = m_handoffProtocol->cancelHandoff(sender, flight, timestamp);

        if (result)
        {
            // Revert scope transfer
            if (m_sharedRadarScope)
            {
                m_sharedRadarScope->setHandoffInProgress(flight, false);
            }

            fireEvent(CoordinationEvent::HandoffCancelled, flight, sender, "Handoff cancelled");
            logCoordinationEvent("HANDOFF_CANCELLED", flight->callSign());
        }

        return result;
    }

    bool ControllerCoordinator::sendPointOut(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        shared_ptr<Flight> trafficFlight,
        chrono::microseconds timestamp
    )
    {
        if (!m_handoffProtocol || !sender || !recipient || !flight || !trafficFlight)
        {
            return false;
        }

        bool result = m_handoffProtocol->initiatePointOut(sender, recipient, flight, trafficFlight, timestamp);

        if (result)
        {
            fireEvent(CoordinationEvent::PointOutSent, flight, recipient, "Point out for " + trafficFlight->callSign());
            logCoordinationEvent("POINT_OUT_SENT", flight->callSign() + " to " + recipient->callSign());
        }

        return result;
    }

    bool ControllerCoordinator::acknowledgePointOut(
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (!m_handoffProtocol || !recipient || !flight)
        {
            return false;
        }

        bool result = m_handoffProtocol->acknowledgePointOut(recipient, flight, timestamp);

        if (result)
        {
            fireEvent(CoordinationEvent::PointOutAcknowledged, flight, recipient, "Point out acknowledged");
            logCoordinationEvent("POINT_OUT_ACK", flight->callSign() + " by " + recipient->callSign());
        }

        return result;
    }

    bool ControllerCoordinator::sendTrafficAdvisory(
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

        if (!m_messageFactory)
        {
            return false;
        }

        auto message = m_messageFactory->createTrafficAdvisory(sender, recipient, flight, trafficFlight, timestamp);
        if (message)
        {
            fireEvent(CoordinationEvent::TrafficAdvisorySent, flight, recipient, "Traffic advisory: " + trafficFlight->callSign());
            logCoordinationEvent("TRAFFIC_ADVISORY", flight->callSign() + " to " + recipient->callSign());
            return true;
        }

        return false;
    }

    bool ControllerCoordinator::requestEmergencyHandoff(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (!m_handoffProtocol || !sender || !recipient || !flight)
        {
            return false;
        }

        bool result = m_handoffProtocol->initiateEmergencyHandoff(sender, recipient, flight, timestamp);

        if (result)
        {
            if (m_sharedRadarScope)
            {
                m_sharedRadarScope->setFlightEmergency(flight, true);
                m_sharedRadarScope->initiateHandoffScopeTransfer(flight, sender, recipient, timestamp);
            }

            fireEvent(CoordinationEvent::EmergencyHandoff, flight, recipient, "EMERGENCY handoff");
            logCoordinationEvent("EMERGENCY_HANDOFF", flight->callSign() + " from " + sender->callSign() + " to " + recipient->callSign());
        }

        return result;
    }

    void ControllerCoordinator::establishScopeVisibility(
        shared_ptr<ControllerPosition> controller,
        shared_ptr<ControllerPosition> targetController,
        SharedRadarScope::VisibilityRule rule,
        bool bidirectional,
        chrono::microseconds timestamp
    )
    {
        if (m_sharedRadarScope && controller && targetController)
        {
            m_sharedRadarScope->establishScopeVisibility(controller, targetController, rule, bidirectional, timestamp);
        }
    }

    void ControllerCoordinator::transferFlightScope(
        shared_ptr<Flight> flight,
        shared_ptr<ControllerPosition> fromController,
        shared_ptr<ControllerPosition> toController,
        chrono::microseconds timestamp
    )
    {
        if (m_sharedRadarScope && flight && fromController && toController)
        {
            m_sharedRadarScope->transferFlightScope(flight, fromController, toController, timestamp);
        }
    }

    bool ControllerCoordinator::shouldRequestHandoff(shared_ptr<ControllerPosition> sender, shared_ptr<Flight> flight) const
    {
        if (!sender || !flight || !m_workloadModel)
        {
            return false;
        }

        // Don't handoff if sender is not overloaded
        if (!m_workloadModel->isOverloaded(sender))
        {
            return false;
        }

        // Check if there's already a pending handoff
        if (m_handoffProtocol && m_handoffProtocol->hasPendingHandoff(flight))
        {
            return false;
        }

        return true;
    }

    shared_ptr<ControllerPosition> ControllerCoordinator::findBestHandoffRecipient(
        shared_ptr<Flight> flight,
        shared_ptr<ControllerPosition> sender
    ) const
    {
        if (!flight || !sender)
        {
            return nullptr;
        }

        auto availableRecipients = getAvailableRecipients(flight, sender);
        if (availableRecipients.empty())
        {
            return nullptr;
        }

        // Find the recipient with the lowest workload
        shared_ptr<ControllerPosition> bestRecipient = nullptr;
        float lowestWorkload = 2.0f;

        for (const auto& recipient : availableRecipients)
        {
            float workload = m_workloadModel ? m_workloadModel->getWorkloadScore(recipient) : 0.0f;
            if (workload < lowestWorkload)
            {
                lowestWorkload = workload;
                bestRecipient = recipient;
            }
        }

        return bestRecipient;
    }

    vector<shared_ptr<ControllerPosition>> ControllerCoordinator::getAvailableRecipients(
        shared_ptr<Flight> flight,
        shared_ptr<ControllerPosition> sender
    ) const
    {
        vector<shared_ptr<ControllerPosition>> available;

        if (!flight || !sender)
        {
            return available;
        }

        // Get adjacent controllers
        auto adjacentIt = m_adjacentControllers.find(sender);
        if (adjacentIt == m_adjacentControllers.end())
        {
            return available;
        }

        for (const auto& candidate : adjacentIt->second)
        {
            if (candidate == sender)
            {
                continue;
            }

            // Check if candidate can accept
            if (m_workloadModel && !m_workloadModel->canAcceptAdditionalAircraft(candidate))
            {
                continue;
            }

            // Check if flight is in candidate's sector
            if (!isFlightOwnedByControllerSector(flight, candidate))
            {
                continue;
            }

            // Check frequency coverage
            if (!isWithinFrequencyCoverage(flight, candidate))
            {
                continue;
            }

            available.push_back(candidate);
        }

        return available;
    }

    HandoffProtocol::State ControllerCoordinator::getHandoffState(shared_ptr<Flight> flight) const
    {
        if (m_handoffProtocol && flight)
        {
            return m_handoffProtocol->getHandoffState(flight);
        }
        return HandoffProtocol::State::Idle;
    }

    bool ControllerCoordinator::hasPendingHandoff(shared_ptr<Flight> flight) const
    {
        if (m_handoffProtocol && flight)
        {
            return m_handoffProtocol->hasPendingHandoff(flight);
        }
        return false;
    }

    shared_ptr<ControllerPosition> ControllerCoordinator::getPendingHandoffRecipient(shared_ptr<Flight> flight) const
    {
        if (m_handoffProtocol && flight)
        {
            return m_handoffProtocol->getPendingHandoffRecipient(flight);
        }
        return nullptr;
    }

    WorkloadModel::WorkloadLevel ControllerCoordinator::getControllerWorkload(shared_ptr<ControllerPosition> controller) const
    {
        if (m_workloadModel && controller)
        {
            return m_workloadModel->getWorkloadLevel(controller);
        }
        return WorkloadModel::WorkloadLevel::Low;
    }

    bool ControllerCoordinator::isFlightVisibleTo(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller) const
    {
        if (m_sharedRadarScope && flight && controller)
        {
            return m_sharedRadarScope->isFlightVisibleTo(flight, controller);
        }
        return false;
    }

    void ControllerCoordinator::processTimeouts(chrono::microseconds timestamp)
    {
        if (m_handoffProtocol)
        {
            m_handoffProtocol->processTimeouts(timestamp);
            m_handoffProtocol->clearCompletedHandoffs();
        }

        if (m_sharedRadarScope)
        {
            m_sharedRadarScope->processTimeouts(timestamp);
        }

        if (m_workloadModel)
        {
            m_workloadModel->processTimeouts(timestamp);
        }
    }

    void ControllerCoordinator::clearCompletedHandoffs()
    {
        if (m_handoffProtocol)
        {
            m_handoffProtocol->clearCompletedHandoffs();
        }
    }

    void ControllerCoordinator::setHandoffTimeouts(
        chrono::milliseconds requestTimeout,
        chrono::milliseconds acceptanceTimeout,
        chrono::milliseconds completeTimeout
    )
    {
        if (m_handoffProtocol)
        {
            m_handoffProtocol->setRequestTimeout(requestTimeout);
            m_handoffProtocol->setAcceptanceTimeout(acceptanceTimeout);
            m_handoffProtocol->setHandoffCompleteTimeout(completeTimeout);
        }
    }

    int ControllerCoordinator::activeHandoffCount() const
    {
        if (m_handoffProtocol)
        {
            return m_handoffProtocol->activeHandoffCount();
        }
        return 0;
    }

    int ControllerCoordinator::pendingHandoffCount() const
    {
        if (m_handoffProtocol)
        {
            return m_handoffProtocol->pendingHandoffCount();
        }
        return 0;
    }

    int ControllerCoordinator::overloadedControllerCount() const
    {
        if (m_workloadModel)
        {
            return static_cast<int>(m_workloadModel->getOverloadedControllers().size());
        }
        return 0;
    }

    vector<shared_ptr<ControllerPosition>> ControllerCoordinator::getOverloadedControllers() const
    {
        if (m_workloadModel)
        {
            return m_workloadModel->getOverloadedControllers();
        }
        return vector<shared_ptr<ControllerPosition>>();
    }

    void ControllerCoordinator::fireEvent(
        CoordinationEvent event,
        shared_ptr<Flight> flight,
        shared_ptr<ControllerPosition> controller,
        const string& details
    )
    {
        if (m_eventCallback)
        {
            m_eventCallback(event, flight, controller, details);
        }
    }

    void ControllerCoordinator::processWorkloadWarnings(chrono::microseconds timestamp)
    {
        if (!m_workloadModel)
        {
            return;
        }

        auto overloaded = m_workloadModel->getOverloadedControllers();
        for (const auto& controller : overloaded)
        {
            fireEvent(CoordinationEvent::WorkloadOverload, nullptr, controller, "Controller overloaded");
            logCoordinationEvent("WORKLOAD_OVERLOAD", controller->callSign());
        }
    }

    void ControllerCoordinator::processPendingHandoffs(chrono::microseconds timestamp)
    {
        if (!m_handoffProtocol)
        {
            return;
        }

        // Process any pending handoffs that need attention
        // This is handled by the handoff protocol's timeout processing
    }

    void ControllerCoordinator::updateScopeVisibility(chrono::microseconds timestamp)
    {
        if (!m_sharedRadarScope)
        {
            return;
        }

        // Update scope visibility based on current handoffs
        // This is handled by the shared radar scope's timeout processing
    }

    shared_ptr<ControllerPosition> ControllerCoordinator::resolveNextControllerBySectorOwnership(
        shared_ptr<Flight> flight,
        shared_ptr<ControllerPosition> sender
    ) const
    {
        if (!flight || !sender || !sender->facility())
        {
            return nullptr;
        }

        const GeoPoint location = flight->aircraft()->location();
        const float altitude = altitudeMslFeet(flight);

        switch (sender->type())
        {
        case ControllerPosition::Type::Departure:
        case ControllerPosition::Type::Area:
            {
                // Try to find approach or tower
                for (const auto& position : sender->facility()->positions())
                {
                    if (position == sender)
                    {
                        continue;
                    }

                    if (position->type() == ControllerPosition::Type::Approach ||
                        position->type() == ControllerPosition::Type::Local)
                    {
                        if (isFlightOwnedByControllerSector(flight, position))
                        {
                            return position;
                        }
                    }
                }
            }
            break;

        case ControllerPosition::Type::Approach:
            {
                // Try to find tower
                for (const auto& position : sender->facility()->positions())
                {
                    if (position == sender)
                    {
                        continue;
                    }

                    if (position->type() == ControllerPosition::Type::Local ||
                        position->type() == ControllerPosition::Type::ClearanceDelivery)
                    {
                        if (isFlightOwnedByControllerSector(flight, position))
                        {
                            return position;
                        }
                    }
                }
            }
            break;

        default:
            break;
        }

        return nullptr;
    }

    bool ControllerCoordinator::isWithinFrequencyCoverage(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller) const
    {
        if (!flight || !flight->aircraft() || !controller || !controller->frequency())
        {
            return false;
        }

        const auto frequency = controller->frequency();
        if (frequency->radiusNm() <= 0.0f || frequency->antennaLocation() == GeoPoint::empty)
        {
            return true;
        }

        const double maxDistanceMeters = frequency->radiusNm() * 1852.0;
        const double actualDistanceMeters = GeoMath::getDistanceMeters(
            flight->aircraft()->location(),
            frequency->antennaLocation()
        );
        return actualDistanceMeters <= maxDistanceMeters;
    }

    float ControllerCoordinator::altitudeMslFeet(shared_ptr<Flight> flight) const
    {
        if (!flight || !flight->aircraft())
        {
            return -1.0f;
        }

        const auto altitude = flight->aircraft()->altitude();
        switch (altitude.type())
        {
        case Altitude::Type::MSL:
            return altitude.feet();
        case Altitude::Type::Ground:
            if (m_host)
            {
                return m_host->queryTerrainElevationAt(flight->aircraft()->location());
            }
            return -1.0f;
        case Altitude::Type::AGL:
            if (m_host)
            {
                return altitude.feet() + m_host->queryTerrainElevationAt(flight->aircraft()->location());
            }
            return -1.0f;
        default:
            return -1.0f;
        }
    }

    bool ControllerCoordinator::isFlightOwnedByControllerSector(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller) const
    {
        if (!flight || !controller || !controller->facility())
        {
            return false;
        }

        const auto sectorOwner = controller->facility()->tryFindPosition(
            controller->type(),
            flight->aircraft()->location(),
            altitudeMslFeet(flight)
        );
        return sectorOwner == controller;
    }

    void ControllerCoordinator::logCoordinationEvent(const string& event, const string& details) const
    {
        if (!m_host)
        {
            return;
        }

        m_host->writeLog("COORD|%s: %s", event.c_str(), details.c_str());
    }

    // ControllerCoordinatorFactory implementation

    ControllerCoordinatorFactory::ControllerCoordinatorFactory(
        shared_ptr<HostServices> _host,
        shared_ptr<ControllerCoordinator> _coordinator
    ) : m_host(_host),
        m_coordinator(_coordinator)
    {
    }

    bool ControllerCoordinatorFactory::requestHandoff(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (m_coordinator)
        {
            return m_coordinator->requestHandoff(sender, recipient, flight, timestamp);
        }
        return false;
    }

    bool ControllerCoordinatorFactory::acceptHandoff(
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (m_coordinator)
        {
            return m_coordinator->acceptHandoff(recipient, flight, timestamp);
        }
        return false;
    }

    bool ControllerCoordinatorFactory::rejectHandoff(
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        CoordinationMessage::RejectionReason reason,
        chrono::microseconds timestamp
    )
    {
        if (m_coordinator)
        {
            return m_coordinator->rejectHandoff(recipient, flight, reason, timestamp);
        }
        return false;
    }

    bool ControllerCoordinatorFactory::sendPointOut(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        shared_ptr<Flight> trafficFlight,
        chrono::microseconds timestamp
    )
    {
        if (m_coordinator)
        {
            return m_coordinator->sendPointOut(sender, recipient, flight, trafficFlight, timestamp);
        }
        return false;
    }

    bool ControllerCoordinatorFactory::requestEmergencyHandoff(
        shared_ptr<ControllerPosition> sender,
        shared_ptr<ControllerPosition> recipient,
        shared_ptr<Flight> flight,
        chrono::microseconds timestamp
    )
    {
        if (m_coordinator)
        {
            return m_coordinator->requestEmergencyHandoff(sender, recipient, flight, timestamp);
        }
        return false;
    }

    void ControllerCoordinatorFactory::transferScope(
        shared_ptr<Flight> flight,
        shared_ptr<ControllerPosition> fromController,
        shared_ptr<ControllerPosition> toController,
        chrono::microseconds timestamp
    )
    {
        if (m_coordinator)
        {
            m_coordinator->transferFlightScope(flight, fromController, toController, timestamp);
        }
    }
}
