//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once

#include <memory>
#include <string>
#include <chrono>
#include "libworld.h"
#include "stateMachine.hpp"

namespace world
{
    /**
     * @brief Flow control state enumeration
     * States for managing airport arrival/departure flow rates
     */
    enum class FlowState
    {
        Normal = 0,      ///< Normal operations - full arrival/departure rates
        Reduced = 1,     ///< Reduced rates due to weather or congestion
        GroundStop = 2,  ///< Ground stop in effect - no departures
        HoldShort = 3    ///< Hold short - arrivals held before landing
    };

    /**
     * @brief Flow state machine trigger events
     */
    enum class FlowTrigger
    {
        EnterReduced = 0,       ///< Enter reduced flow state
        EnterGroundStop = 1,    ///< Enter ground stop state
        EnterHoldShort = 2,     ///< Enter hold short state
        ResumeNormal = 3,       ///< Resume normal operations
        TimeoutExpired = 4,     ///< State timeout expired
        WeatherImproved = 5,    ///< Weather conditions improved
        CongestionCleared = 6   ///< Congestion cleared
    };

    /**
     * @brief Flow state transition information
     */
    struct FlowTransition
    {
        FlowState fromState;
        FlowState toState;
        FlowTrigger trigger;
        std::chrono::steady_clock::time_point timestamp;
        std::string reason;

        FlowTransition(FlowState from, FlowState to, FlowTrigger trig, const std::string& r)
            : fromState(from), toState(to), trigger(trig), reason(r)
        {
            timestamp = std::chrono::steady_clock::now();
        }
    };

    /**
     * @brief Flow state machine for managing airport flow control
     * Implements state transitions for Normal, Reduced, GroundStop, and HoldShort states
     */
    class FlowStateMachine : public StateMachine<FlowState, FlowTrigger>
    {
    public:
        /**
         * @brief Construct a flow state machine
         * @param host Host services for logging
         * @param nameForLog Name for logging purposes
         */
        FlowStateMachine(std::shared_ptr<HostServices> host, const std::string& nameForLog);

        /**
         * @brief Get the current flow state
         */
        FlowState flowState() const { return m_currentFlowState; }

        /**
         * @brief Get the time when the current state was entered
         */
        std::chrono::steady_clock::time_point stateEnteredTime() const { return m_stateEnteredTime; }

        /**
         * @brief Get the list of state transitions
         */
        const std::vector<FlowTransition>& transitionHistory() const { return m_transitionHistory; }

        /**
         * @brief Check if a state change is allowed
         * @param newState Proposed new state
         * @return true if transition is valid
         */
        bool canTransitionTo(FlowState newState) const;

        /**
         * @brief Get the minimum time in seconds before state can change
         * Used to prevent rapid state oscillations
         */
        int minimumStateDurationSeconds() const { return m_minimumStateDurationSeconds; }

        /**
         * @brief Set the minimum state duration
         */
        void setMinimumStateDurationSeconds(int seconds) { m_minimumStateDurationSeconds = seconds; }

    private:
        FlowState m_currentFlowState = FlowState::Normal;
        std::chrono::steady_clock::time_point m_stateEnteredTime;
        std::vector<FlowTransition> m_transitionHistory;
        int m_minimumStateDurationSeconds = 30; // Default 30 seconds minimum

        // State factory methods
        std::shared_ptr<State> createNormalState();
        std::shared_ptr<State> createReducedState();
        std::shared_ptr<State> createGroundStopState();
        std::shared_ptr<State> createHoldShortState();

        void recordTransition(FlowState from, FlowState to, FlowTrigger trigger, const std::string& reason);
    };
}