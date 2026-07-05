// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#include "workloadModel.hpp"

using namespace std;

namespace world
{
    WorkloadModel::WorkloadModel(shared_ptr<HostServices> _host)
        : m_host(_host),
          m_updateIntervalMs(chrono::seconds(5))
    {
        // Default capacity configuration
        m_capacityConfig.maxAircraft = 15;
        m_capacityConfig.maxSimultaneousHandoffs = 5;
        m_capacityConfig.maxPendingClearances = 8;
        m_capacityConfig.maxPointOuts = 4;
        m_capacityConfig.maxWorkloadThreshold = 0.7f;
        m_capacityConfig.overloadThreshold = 0.9f;
        m_capacityConfig.complexityMultiplier = 1.0f;
    }

    WorkloadModel::~WorkloadModel()
    {
    }

    void WorkloadModel::addAircraft(shared_ptr<ControllerPosition> controller, shared_ptr<Flight> flight)
    {
        if (!controller || !flight)
        {
            return;
        }

        m_aircraftByController[controller].push_back(flight);
        m_lastUpdateTimestamp[controller] = chrono::microseconds(0);
    }

    void WorkloadModel::removeAircraft(shared_ptr<ControllerPosition> controller, shared_ptr<Flight> flight)
    {
        if (!controller || !flight)
        {
            return;
        }

        auto it = m_aircraftByController.find(controller);
        if (it != m_aircraftByController.end())
        {
            auto& aircraftList = it->second;
            aircraftList.erase(remove(aircraftList.begin(), aircraftList.end(), flight), aircraftList.end());
        }

        m_lastUpdateTimestamp[controller] = chrono::microseconds(0);
    }

    void WorkloadModel::transferAircraft(
        shared_ptr<Flight> flight,
        shared_ptr<ControllerPosition> fromController,
        shared_ptr<ControllerPosition> toController
    )
    {
        if (!flight || !fromController || !toController)
        {
            return;
        }

        removeAircraft(fromController, flight);
        addAircraft(toController, flight);
    }

    WorkloadModel::WorkloadMetrics WorkloadModel::calculateWorkload(
        shared_ptr<ControllerPosition> controller,
        chrono::microseconds timestamp
    )
    {
        if (!controller)
        {
            WorkloadMetrics emptyMetrics = {};
            emptyMetrics.level = WorkloadLevel::Low;
            emptyMetrics.totalWorkload = 0.0f;
            emptyMetrics.timestamp = timestamp;
            return emptyMetrics;
        }

        // Check if we need to recalculate
        auto lastUpdateIt = m_lastUpdateTimestamp.find(controller);
        bool needsUpdate = (lastUpdateIt == m_lastUpdateTimestamp.end() ||
            (timestamp - lastUpdateIt->second) > chrono::duration_cast<chrono::microseconds>(m_updateIntervalMs));

        if (!needsUpdate)
        {
            return m_metricsByController[controller];
        }

        WorkloadMetrics metrics = calculateMetricsInternal(controller, timestamp);
        m_metricsByController[controller] = metrics;
        m_lastUpdateTimestamp[controller] = timestamp;

        logWorkloadEvent("CALCULATED", controller, metrics);
        return metrics;
    }

    WorkloadModel::WorkloadLevel WorkloadModel::getWorkloadLevel(shared_ptr<ControllerPosition> controller) const
    {
        auto it = m_metricsByController.find(controller);
        if (it != m_metricsByController.end())
        {
            return it->second.level;
        }
        return WorkloadLevel::Low;
    }

    float WorkloadModel::getWorkloadScore(shared_ptr<ControllerPosition> controller) const
    {
        auto it = m_metricsByController.find(controller);
        if (it != m_metricsByController.end())
        {
            return it->second.totalWorkload;
        }
        return 0.0f;
    }

    bool WorkloadModel::canAcceptAdditionalAircraft(shared_ptr<ControllerPosition> controller) const
    {
        if (!controller)
        {
            return false;
        }

        auto it = m_metricsByController.find(controller);
        if (it != m_metricsByController.end())
        {
            const auto& metrics = it->second;
            if (metrics.level == WorkloadLevel::Overloaded)
            {
                return false;
            }
            if (metrics.aircraftCount >= m_capacityConfig.maxAircraft)
            {
                return false;
            }
        }

        return true;
    }

    int WorkloadModel::availableCapacity(shared_ptr<ControllerPosition> controller) const
    {
        if (!controller)
        {
            return 0;
        }

        auto it = m_aircraftByController.find(controller);
        int currentCount = (it != m_aircraftByController.end()) ? static_cast<int>(it->second.size()) : 0;
        return max(0, m_capacityConfig.maxAircraft - currentCount);
    }

    int WorkloadModel::currentAircraftCount(shared_ptr<ControllerPosition> controller) const
    {
        if (!controller)
        {
            return 0;
        }

        auto it = m_aircraftByController.find(controller);
        return (it != m_aircraftByController.end()) ? static_cast<int>(it->second.size()) : 0;
    }

    bool WorkloadModel::isOverloaded(shared_ptr<ControllerPosition> controller) const
    {
        return getWorkloadLevel(controller) == WorkloadLevel::Overloaded;
    }

    float WorkloadModel::calculateAirspaceComplexity(shared_ptr<ControllerPosition> controller) const
    {
        if (!controller || !controller->facility())
        {
            return 0.0f;
        }

        // Complexity based on airspace type and geometry
        float complexity = 0.0f;
        const auto airspace = controller->facility()->airspace();
        if (airspace && airspace->geometry())
        {
            const auto& geometry = airspace->geometry();
            // More complex airspace has more boundaries, restricted areas, etc.
            complexity = 0.3f; // Base complexity
        }

        // Add complexity for terminal areas vs center
        switch (controller->facility()->type())
        {
        case ControlFacility::Type::Tower:
            complexity += 0.2f;
            break;
        case ControlFacility::Type::Terminal:
            complexity += 0.4f;
            break;
        case ControlFacility::Type::Center:
            complexity += 0.3f;
            break;
        default:
            break;
        }

        return min(1.0f, complexity);
    }

    float WorkloadModel::calculateTrafficDensity(shared_ptr<ControllerPosition> controller) const
    {
        if (!controller)
        {
            return 0.0f;
        }

        int count = currentAircraftCount(controller);
        float density = static_cast<float>(count) / static_cast<float>(m_capacityConfig.maxAircraft);
        return min(1.0f, density);
    }

    float WorkloadModel::calculateCoordinationLoad(shared_ptr<ControllerPosition> controller) const
    {
        if (!controller)
        {
            return 0.0f;
        }

        auto it = m_metricsByController.find(controller);
        if (it == m_metricsByController.end())
        {
            return 0.0f;
        }

        const auto& metrics = it->second;
        float coordinationLoad = 0.0f;

        // Factor in pending handoffs and point-outs
        coordinationLoad += static_cast<float>(metrics.pendingHandoffs) / static_cast<float>(m_capacityConfig.maxSimultaneousHandoffs) * 0.5f;
        coordinationLoad += static_cast<float>(metrics.pendingPointOuts) / static_cast<float>(m_capacityConfig.maxPointOuts) * 0.3f;

        return min(1.0f, coordinationLoad);
    }

    void WorkloadModel::processTimeouts(chrono::microseconds timestamp)
    {
        clearStaleMetrics(timestamp);
    }

    void WorkloadModel::clearStaleMetrics(chrono::microseconds timestamp)
    {
        vector<shared_ptr<ControllerPosition>> staleControllers;

        for (const auto& pair : m_lastUpdateTimestamp)
        {
            if ((timestamp - pair.second) > chrono::duration_cast<chrono::microseconds>(chrono::minutes(10)))
            {
                staleControllers.push_back(pair.first);
            }
        }

        for (auto controller : staleControllers)
        {
            m_metricsByController.erase(controller);
            m_aircraftByController.erase(controller);
            m_lastUpdateTimestamp.erase(controller);
        }
    }

    WorkloadModel::WorkloadMetrics WorkloadModel::getMetrics(shared_ptr<ControllerPosition> controller) const
    {
        auto it = m_metricsByController.find(controller);
        if (it != m_metricsByController.end())
        {
            return it->second;
        }

        WorkloadMetrics emptyMetrics = {};
        emptyMetrics.level = WorkloadLevel::Low;
        emptyMetrics.totalWorkload = 0.0f;
        return emptyMetrics;
    }

    vector<shared_ptr<ControllerPosition>> WorkloadModel::getOverloadedControllers() const
    {
        vector<shared_ptr<ControllerPosition>> overloaded;

        for (const auto& pair : m_metricsByController)
        {
            if (pair.second.level == WorkloadLevel::Overloaded)
            {
                overloaded.push_back(pair.first);
            }
        }

        return overloaded;
    }

    vector<shared_ptr<ControllerPosition>> WorkloadModel::getControllersSortedByWorkload() const
    {
        vector<shared_ptr<ControllerPosition>> controllers;
        for (const auto& pair : m_metricsByController)
        {
            controllers.push_back(pair.first);
        }

        sort(controllers.begin(), controllers.end(),
            [this](shared_ptr<ControllerPosition> a, shared_ptr<ControllerPosition> b) {
                return getWorkloadScore(a) > getWorkloadScore(b);
            });

        return controllers;
    }

    WorkloadModel::WorkloadMetrics WorkloadModel::calculateMetricsInternal(
        shared_ptr<ControllerPosition> controller,
        chrono::microseconds timestamp
    )
    {
        WorkloadMetrics metrics = {};
        metrics.timestamp = timestamp;

        // Count aircraft
        auto aircraftIt = m_aircraftByController.find(controller);
        metrics.aircraftCount = (aircraftIt != m_aircraftByController.end())
            ? static_cast<int>(aircraftIt->second.size())
            : 0;

        // Calculate complexity factors
        metrics.airspaceComplexity = calculateAirspaceComplexity(controller);
        metrics.trafficDensity = calculateTrafficDensity(controller);
        metrics.coordinationLoad = calculateCoordinationLoad(controller);

        // Default values for other factors
        metrics.weatherComplexity = 0.0f;
        metrics.pendingHandoffs = 0;
        metrics.pendingClearances = 0;
        metrics.pendingPointOuts = 0;
        metrics.emergencyAircraft = 0;

        // Calculate total workload score
        metrics.totalWorkload = calculateComplexityScore(metrics);
        metrics.level = determineWorkloadLevel(metrics.totalWorkload);

        return metrics;
    }

    float WorkloadModel::calculateComplexityScore(const WorkloadMetrics& metrics) const
    {
        float score = 0.0f;

        // Aircraft count factor (0-0.4)
        float aircraftFactor = min(1.0f, static_cast<float>(metrics.aircraftCount) / static_cast<float>(m_capacityConfig.maxAircraft));
        score += aircraftFactor * 0.4f;

        // Traffic density factor (0-0.2)
        score += metrics.trafficDensity * 0.2f;

        // Airspace complexity factor (0-0.15)
        score += metrics.airspaceComplexity * 0.15f;

        // Coordination load factor (0-0.15)
        score += metrics.coordinationLoad * 0.15f;

        // Weather complexity factor (0-0.1)
        score += metrics.weatherComplexity * 0.1f;

        // Apply complexity multiplier
        score *= m_capacityConfig.complexityMultiplier;

        return min(1.0f, score);
    }

    WorkloadModel::WorkloadLevel WorkloadModel::determineWorkloadLevel(float score) const
    {
        if (score >= m_capacityConfig.overloadThreshold)
        {
            return WorkloadLevel::Overloaded;
        }
        else if (score >= m_capacityConfig.maxWorkloadThreshold)
        {
            return WorkloadLevel::High;
        }
        else if (score >= 0.4f)
        {
            return WorkloadLevel::Normal;
        }
        else
        {
            return WorkloadLevel::Low;
        }
    }

    bool WorkloadModel::isMetricsStale(shared_ptr<ControllerPosition> controller, chrono::microseconds timestamp) const
    {
        auto it = m_lastUpdateTimestamp.find(controller);
        if (it == m_lastUpdateTimestamp.end())
        {
            return true;
        }
        return (timestamp - it->second) > chrono::duration_cast<chrono::microseconds>(m_updateIntervalMs * 2);
    }

    void WorkloadModel::logWorkloadEvent(const string& event, shared_ptr<ControllerPosition> controller, const WorkloadMetrics& metrics) const
    {
        if (!m_host)
        {
            return;
        }

        m_host->writeLog(
            "WORKLOAD|%s controller[%s] aircraft[%d] workload[%.2f] level[%d]",
            event.c_str(),
            controller ? controller->callSign().c_str() : "N/A",
            metrics.aircraftCount,
            metrics.totalWorkload,
            (int)metrics.level
        );
    }

    // WorkloadModelFactory implementation

    WorkloadModelFactory::WorkloadModelFactory(
        shared_ptr<HostServices> _host,
        shared_ptr<WorkloadModel> _model
    ) : m_host(_host),
        m_model(_model)
    {
    }

    void WorkloadModelFactory::addAircraft(shared_ptr<ControllerPosition> controller, shared_ptr<Flight> flight)
    {
        if (m_model)
        {
            m_model->addAircraft(controller, flight);
        }
    }

    void WorkloadModelFactory::removeAircraft(shared_ptr<ControllerPosition> controller, shared_ptr<Flight> flight)
    {
        if (m_model)
        {
            m_model->removeAircraft(controller, flight);
        }
    }

    void WorkloadModelFactory::transferAircraft(
        shared_ptr<Flight> flight,
        shared_ptr<ControllerPosition> fromController,
        shared_ptr<ControllerPosition> toController
    )
    {
        if (m_model)
        {
            m_model->transferAircraft(flight, fromController, toController);
        }
    }

    bool WorkloadModelFactory::canAcceptHandoff(shared_ptr<ControllerPosition> controller)
    {
        if (m_model)
        {
            return m_model->canAcceptAdditionalAircraft(controller);
        }
        return true;
    }

    WorkloadModel::WorkloadLevel WorkloadModelFactory::getWorkloadLevel(shared_ptr<ControllerPosition> controller)
    {
        if (m_model)
        {
            return m_model->getWorkloadLevel(controller);
        }
        return WorkloadModel::WorkloadLevel::Low;
    }
}
