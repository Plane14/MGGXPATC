//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#include "flowManager.hpp"
#include "flowStateMachine.hpp"
#include "runwayOccupancyTracker.hpp"
#include "arrivalFlowController.hpp"
#include "departureFlowController.hpp"
#include "groundStopManager.hpp"

using namespace std;

namespace world
{
    FlowManager::FlowManager()
    {
        m_stateMachine = make_shared<FlowStateMachine>(nullptr, "FlowManager");
        m_occupancyTracker = make_shared<RunwayOccupancyTracker>();
        m_arrivalController = make_shared<ArrivalFlowController>();
        m_departureController = make_shared<DepartureFlowController>();
        m_groundStopManager = make_shared<GroundStopManager>();
    }

    void FlowManager::setAirport(shared_ptr<Airport> airport)
    {
        m_airport = airport;
        if (airport)
        {
            m_config.airportIcao = airport->header().icao();
            m_arrivalController->setAirport(m_config.airportIcao);
            m_departureController->setAirport(m_config.airportIcao);
            m_groundStopManager->setAirport(m_config.airportIcao);
        }
    }

    void FlowManager::configure(const FlowManagerConfig& config)
    {
        m_config = config;
        
        // Configure arrival controller
        ArrivalFlowConfig arrivalConfig(m_config.airportIcao);
        arrivalConfig.maxArrivalsPerHour = m_config.arrivalRatePerHour;
        arrivalConfig.currentRatePerHour = m_config.arrivalRatePerHour;
        m_arrivalController->configure(arrivalConfig);

        // Configure departure controller
        DepartureFlowConfig departureConfig(m_config.airportIcao);
        departureConfig.maxDeparturesPerHour = m_config.departureRatePerHour;
        departureConfig.currentRatePerHour = m_config.departureRatePerHour;
        m_departureController->configure(departureConfig);
    }

    bool FlowManager::canAcceptArrival(
        shared_ptr<Flight> flight,
        chrono::steady_clock::time_point currentTime) const
    {
        if (!m_config.enableFlowControl)
        {
            return true;
        }

        // Check flow state
        if (m_stateMachine)
        {
            FlowState state = m_stateMachine->flowState();
            if (state == FlowState::GroundStop || state == FlowState::HoldShort)
            {
                return false;
            }
        }

        // Check arrival rate
        return m_arrivalController->canAcceptArrival(currentTime);
    }

    bool FlowManager::canReleaseDeparture(
        shared_ptr<Flight> flight,
        chrono::steady_clock::time_point currentTime) const
    {
        if (!m_config.enableFlowControl)
        {
            return true;
        }

        // Check flow state
        if (m_stateMachine->currentState())
        {
            FlowState state = m_stateMachine->flowState();
            if (state == FlowState::GroundStop)
            {
                return false;
            }
        }

        // Check ground stop
        if (m_config.enableGroundStop && m_groundStopManager)
        {
            if (m_groundStopManager->isFlightUnderGroundStop(flight, currentTime))
            {
                return false;
            }
        }

        // Check departure rate
        return m_departureController->canReleaseDeparture(currentTime);
    }

    void FlowManager::recordRunwayEntry(
        shared_ptr<Flight> flight,
        const string& runwayName,
        bool isDeparture)
    {
        m_occupancyTracker->recordRunwayEntry(flight, runwayName, isDeparture);

        if (isDeparture)
        {
            m_departureController->recordDeparture(flight, chrono::steady_clock::now());
        }
        else
        {
            m_arrivalController->recordArrival(flight, chrono::steady_clock::now());
        }
    }

    void FlowManager::recordRunwayExit(
        shared_ptr<Flight> flight,
        const string& runwayName)
    {
        m_occupancyTracker->recordRunwayExit(flight, runwayName);
    }

    void FlowManager::initiateGroundStop(
        const string& destinationIcao,
        int durationSeconds,
        const string& reason)
    {
        if (m_groundStopManager)
        {
            m_groundStopManager->initiateGroundStop(destinationIcao, durationSeconds, reason);
        }
    }

    void FlowManager::cancelGroundStop(const string& destinationIcao)
    {
        if (m_groundStopManager)
        {
            m_groundStopManager->cancelGroundStop(destinationIcao);
        }
    }

    void FlowManager::updateFlowState(chrono::steady_clock::time_point currentTime)
    {
        // In a real implementation, this would check weather, congestion, etc.
        // and trigger state transitions accordingly
        // For now, this is a placeholder for future integration
    }

    FlowState FlowManager::getFlowState() const
    {
        if (m_stateMachine)
        {
            return m_stateMachine->flowState();
        }
        return FlowState::Normal;
    }

    chrono::steady_clock::time_point FlowManager::getNextArrivalTime(
        chrono::steady_clock::time_point currentTime) const
    {
        return m_arrivalController->getNextAvailableTime(currentTime);
    }

    chrono::steady_clock::time_point FlowManager::getNextDepartureTime(
        chrono::steady_clock::time_point currentTime) const
    {
        return m_departureController->getNextAvailableTime(currentTime);
    }
}