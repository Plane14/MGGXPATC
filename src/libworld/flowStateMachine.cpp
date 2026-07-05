//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#include "flowStateMachine.hpp"

using namespace std;

namespace world
{
    FlowStateMachine::FlowStateMachine(shared_ptr<HostServices> host, const string& nameForLog)
        : StateMachine<FlowState, FlowTrigger>(host, nameForLog)
    {
        m_stateEnteredTime = chrono::steady_clock::now();

        // Register all states
        addState(FlowState::Normal, [this]() { return createNormalState(); });
        addState(FlowState::Reduced, [this]() { return createReducedState(); });
        addState(FlowState::GroundStop, [this]() { return createGroundStopState(); });
        addState(FlowState::HoldShort, [this]() { return createHoldShortState(); });

        // Start in Normal state
        transitionToState(FlowState::Normal);
    }

    bool FlowStateMachine::canTransitionTo(FlowState newState) const
    {
        auto now = chrono::steady_clock::now();
        auto elapsed = chrono::duration_cast<chrono::seconds>(now - m_stateEnteredTime).count();

        // Check minimum state duration
        if (elapsed < m_minimumStateDurationSeconds)
        {
            return false;
        }

        // Validate state transitions
        switch (m_currentFlowState)
        {
        case FlowState::Normal:
            // From Normal, can go to any state
            return true;

        case FlowState::Reduced:
            // From Reduced, can go to Normal or GroundStop
            return (newState == FlowState::Normal || newState == FlowState::GroundStop);

        case FlowState::GroundStop:
            // From GroundStop, can only go to HoldShort or Normal
            return (newState == FlowState::HoldShort || newState == FlowState::Normal);

        case FlowState::HoldShort:
            // From HoldShort, can only go to Normal
            return (newState == FlowState::Normal);

        default:
            return false;
        }
    }

    void FlowStateMachine::recordTransition(FlowState from, FlowState to, FlowTrigger trigger, const string& reason)
    {
        m_transitionHistory.emplace_back(from, to, trigger, reason);
        m_currentFlowState = to;
        m_stateEnteredTime = chrono::steady_clock::now();
    }

    shared_ptr<StateMachine<FlowState, FlowTrigger>::State> FlowStateMachine::createNormalState()
    {
        auto state = new DeclarativeState(
            FlowState::Normal,
            "Normal",
            {
                // Enter Reduced on EnterReduced trigger
                Transition(
                    [](FlowTrigger t) { return t == FlowTrigger::EnterReduced; },
                    [this](StateMachine<FlowState, FlowTrigger>& machine) {
                        recordTransition(m_currentFlowState, FlowState::Reduced, FlowTrigger::EnterReduced, "Reduced flow conditions");
                        machine.transitionToState(FlowState::Reduced);
                    }
                ),
                // Enter GroundStop on EnterGroundStop trigger
                Transition(
                    [](FlowTrigger t) { return t == FlowTrigger::EnterGroundStop; },
                    [this](StateMachine<FlowState, FlowTrigger>& machine) {
                        recordTransition(m_currentFlowState, FlowState::GroundStop, FlowTrigger::EnterGroundStop, "Ground stop initiated");
                        machine.transitionToState(FlowState::GroundStop);
                    }
                ),
                // Enter HoldShort on EnterHoldShort trigger
                Transition(
                    [](FlowTrigger t) { return t == FlowTrigger::EnterHoldShort; },
                    [this](StateMachine<FlowState, FlowTrigger>& machine) {
                        recordTransition(m_currentFlowState, FlowState::HoldShort, FlowTrigger::EnterHoldShort, "Hold short for arrivals");
                        machine.transitionToState(FlowState::HoldShort);
                    }
                )
            },
            [this]() {
                if (host())
                {
                    host()->writeLog("%s|FLOW_STATE Normal state entered", nameForLog().c_str());
                }
            }
        );
        return shared_ptr<StateMachine<FlowState, FlowTrigger>::State>(state);
    }

    shared_ptr<StateMachine<FlowState, FlowTrigger>::State> FlowStateMachine::createReducedState()
    {
        auto state = new DeclarativeState(
            FlowState::Reduced,
            "Reduced",
            {
                // Resume Normal on ResumeNormal or WeatherImproved
                Transition(
                    [](FlowTrigger t) { return t == FlowTrigger::ResumeNormal || t == FlowTrigger::WeatherImproved; },
                    [this](StateMachine<FlowState, FlowTrigger>& machine) {
                        recordTransition(m_currentFlowState, FlowState::Normal, FlowTrigger::ResumeNormal, "Normal operations resumed");
                        machine.transitionToState(FlowState::Normal);
                    }
                ),
                // Enter GroundStop on EnterGroundStop
                Transition(
                    [](FlowTrigger t) { return t == FlowTrigger::EnterGroundStop; },
                    [this](StateMachine<FlowState, FlowTrigger>& machine) {
                        recordTransition(m_currentFlowState, FlowState::GroundStop, FlowTrigger::EnterGroundStop, "Ground stop initiated");
                        machine.transitionToState(FlowState::GroundStop);
                    }
                )
            },
            [this]() {
                if (host())
                {
                    host()->writeLog("%s|FLOW_STATE Reduced state entered", nameForLog().c_str());
                }
            }
        );
        return shared_ptr<StateMachine<FlowState, FlowTrigger>::State>(state);
    }

    shared_ptr<StateMachine<FlowState, FlowTrigger>::State> FlowStateMachine::createGroundStopState()
    {
        auto state = new DeclarativeState(
            FlowState::GroundStop,
            "GroundStop",
            {
                // Resume Normal on ResumeNormal
                Transition(
                    [](FlowTrigger t) { return t == FlowTrigger::ResumeNormal; },
                    [this](StateMachine<FlowState, FlowTrigger>& machine) {
                        recordTransition(m_currentFlowState, FlowState::Normal, FlowTrigger::ResumeNormal, "Ground stop cancelled");
                        machine.transitionToState(FlowState::Normal);
                    }
                ),
                // Enter HoldShort on EnterHoldShort
                Transition(
                    [](FlowTrigger t) { return t == FlowTrigger::EnterHoldShort; },
                    [this](StateMachine<FlowState, FlowTrigger>& machine) {
                        recordTransition(m_currentFlowState, FlowState::HoldShort, FlowTrigger::EnterHoldShort, "Hold short for arrivals");
                        machine.transitionToState(FlowState::HoldShort);
                    }
                )
            },
            [this]() {
                if (host())
                {
                    host()->writeLog("%s|FLOW_STATE GroundStop state entered", nameForLog().c_str());
                }
            }
        );
        return shared_ptr<StateMachine<FlowState, FlowTrigger>::State>(state);
    }

    shared_ptr<StateMachine<FlowState, FlowTrigger>::State> FlowStateMachine::createHoldShortState()
    {
        auto state = new DeclarativeState(
            FlowState::HoldShort,
            "HoldShort",
            {
                // Resume Normal on ResumeNormal
                Transition(
                    [](FlowTrigger t) { return t == FlowTrigger::ResumeNormal; },
                    [this](StateMachine<FlowState, FlowTrigger>& machine) {
                        recordTransition(m_currentFlowState, FlowState::Normal, FlowTrigger::ResumeNormal, "Hold short cancelled");
                        machine.transitionToState(FlowState::Normal);
                    }
                )
            },
            [this]() {
                if (host())
                {
                    host()->writeLog("%s|FLOW_STATE HoldShort state entered", nameForLog().c_str());
                }
            }
        );
        return shared_ptr<StateMachine<FlowState, FlowTrigger>::State>(state);
    }
}