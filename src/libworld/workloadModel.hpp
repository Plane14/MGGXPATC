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

    // ============================================================================
    // WorkloadModel - Controller workload modeling
    // ============================================================================

    class WorkloadModel
    {
    public:
        enum class WorkloadLevel
        {
            Low = 0,
            Normal = 1,
            High = 2,
            Overloaded = 3
        };

        enum class ComplexityFactor
        {
            AircraftCount = 0,
            AirspaceComplexity = 1,
            WeatherComplexity = 2,
            TrafficDensity = 3,
            CoordinationLoad = 4,
            EmergencyLoad = 5,
            TrainingFactor = 6
        };

        struct WorkloadMetrics
        {
            int aircraftCount;
            int pendingHandoffs;
            int pendingClearances;
            int pendingPointOuts;
            int emergencyAircraft;
            float airspaceComplexity;
            float weatherComplexity;
            float trafficDensity;
            float coordinationLoad;
            float totalWorkload;
            WorkloadLevel level;
            chrono::microseconds timestamp;
        };

        struct CapacityConfig
        {
            int maxAircraft;
            int maxSimultaneousHandoffs;
            int maxPendingClearances;
            int maxPointOuts;
            float maxWorkloadThreshold;
            float overloadThreshold;
            float complexityMultiplier;
        };

    private:
        shared_ptr<HostServices> m_host;
        CapacityConfig m_capacityConfig;
        unordered_map<shared_ptr<ControllerPosition>, WorkloadMetrics> m_metricsByController;
        unordered_map<shared_ptr<ControllerPosition>, vector<shared_ptr<Flight>>> m_aircraftByController;
        unordered_map<shared_ptr<ControllerPosition>, chrono::microseconds> m_lastUpdateTimestamp;
        chrono::milliseconds m_updateIntervalMs;

    public:
        WorkloadModel(shared_ptr<HostServices> _host);
        ~WorkloadModel();

    public:
        // Configuration
        void setCapacityConfig(const CapacityConfig& config) { m_capacityConfig = config; }
        const CapacityConfig& getCapacityConfig() const { return m_capacityConfig; }
        void setUpdateInterval(chrono::milliseconds interval) { m_updateIntervalMs = interval; }

        // Aircraft tracking
        void addAircraft(shared_ptr<ControllerPosition> controller, shared_ptr<Flight> flight);
        void removeAircraft(shared_ptr<ControllerPosition> controller, shared_ptr<Flight> flight);
        void transferAircraft(
            shared_ptr<Flight> flight,
            shared_ptr<ControllerPosition> fromController,
            shared_ptr<ControllerPosition> toController
        );

        // Workload calculation
        WorkloadMetrics calculateWorkload(shared_ptr<ControllerPosition> controller, chrono::microseconds timestamp);
        WorkloadLevel getWorkloadLevel(shared_ptr<ControllerPosition> controller) const;
        float getWorkloadScore(shared_ptr<ControllerPosition> controller) const;

        // Capacity queries
        bool canAcceptAdditionalAircraft(shared_ptr<ControllerPosition> controller) const;
        int availableCapacity(shared_ptr<ControllerPosition> controller) const;
        int currentAircraftCount(shared_ptr<ControllerPosition> controller) const;
        bool isOverloaded(shared_ptr<ControllerPosition> controller) const;

        // Complexity calculation
        float calculateAirspaceComplexity(shared_ptr<ControllerPosition> controller) const;
        float calculateTrafficDensity(shared_ptr<ControllerPosition> controller) const;
        float calculateCoordinationLoad(shared_ptr<ControllerPosition> controller) const;

        // Timeout processing
        void processTimeouts(chrono::microseconds timestamp);
        void clearStaleMetrics(chrono::microseconds timestamp);

        // Statistics
        WorkloadMetrics getMetrics(shared_ptr<ControllerPosition> controller) const;
        vector<shared_ptr<ControllerPosition>> getOverloadedControllers() const;
        vector<shared_ptr<ControllerPosition>> getControllersSortedByWorkload() const;

    private:
        WorkloadMetrics calculateMetricsInternal(shared_ptr<ControllerPosition> controller, chrono::microseconds timestamp);
        float calculateComplexityScore(const WorkloadMetrics& metrics) const;
        WorkloadLevel determineWorkloadLevel(float score) const;
        bool isMetricsStale(shared_ptr<ControllerPosition> controller, chrono::microseconds timestamp) const;
        void logWorkloadEvent(const string& event, shared_ptr<ControllerPosition> controller, const WorkloadMetrics& metrics) const;
    };

    // ============================================================================
    // WorkloadModelFactory - Factory for workload operations
    // ============================================================================

    class WorkloadModelFactory
    {
    private:
        shared_ptr<HostServices> m_host;
        shared_ptr<WorkloadModel> m_model;

    public:
        WorkloadModelFactory(shared_ptr<HostServices> _host, shared_ptr<WorkloadModel> _model);

    public:
        void addAircraft(shared_ptr<ControllerPosition> controller, shared_ptr<Flight> flight);
        void removeAircraft(shared_ptr<ControllerPosition> controller, shared_ptr<Flight> flight);
        void transferAircraft(
            shared_ptr<Flight> flight,
            shared_ptr<ControllerPosition> fromController,
            shared_ptr<ControllerPosition> toController
        );
        bool canAcceptHandoff(shared_ptr<ControllerPosition> controller);
        WorkloadModel::WorkloadLevel getWorkloadLevel(shared_ptr<ControllerPosition> controller);
    };
}
