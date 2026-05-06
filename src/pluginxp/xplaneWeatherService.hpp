// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#pragma once

#include <algorithm>
#include <utility>

#include "XPLMWeather.h"
#include "libworld.h"

using namespace std;
using namespace world;

class XPlaneWeatherService : public WeatherService
{
private:
    shared_ptr<HostServices> m_host;
public:
    explicit XPlaneWeatherService(shared_ptr<HostServices> host) :
        m_host(std::move(host))
    {
        if (m_host)
        {
            m_host->writeLog("WXSRV|XPlaneWeatherService initialized");
        }
    }
public:
    WeatherSnapshot getWeatherAt(const GeoPoint& location, float altitudeMeters = 0.0f) override
    {
        WeatherSnapshot snapshot;

#if defined(XPLM400)
        snapshot.available = true;
        XPLMWeatherInfo_t info = { 0 };
        info.structSize = sizeof(XPLMWeatherInfo_t);

        const int detailed = XPLMGetWeatherAtLocation(location.latitude, location.longitude, altitudeMeters, &info);
        snapshot.detailed = detailed != 0;
        snapshot.temperatureC = info.temperature_alt;
        snapshot.dewpointC = info.dewpoint_alt;
        snapshot.pressurePa = info.pressure_alt > 0.0f ? info.pressure_alt : info.pressure_sl;
        snapshot.visibilityMeters = info.visibility;
        snapshot.precipitationRate = info.precip_rate_alt;
        snapshot.windDirectionTrueDegrees = info.wind_dir_alt;
        snapshot.windSpeedMetersPerSecond = max(0.0f, info.wind_spd_alt);
        snapshot.windGustMetersPerSecond = 0.0f;

        for (int i = 0 ; i < XPLM_NUM_WIND_LAYERS ; ++i)
        {
            const auto& layer = info.wind_layers[i];
            if (layer.speed < 0.0f)
            {
                continue;
            }

            snapshot.windLayerAltitudesMeters.push_back(layer.alt_msl);
            snapshot.windLayerSpeedsMetersPerSecond.push_back(layer.speed);
            snapshot.windLayerDirectionsTrueDegrees.push_back(layer.direction);
            snapshot.windGustMetersPerSecond = max(snapshot.windGustMetersPerSecond, layer.gust_speed);
        }

        for (int i = 0 ; i < XPLM_NUM_CLOUD_LAYERS ; ++i)
        {
            const auto& layer = info.cloud_layers[i];
            if (layer.coverage <= 0.0f && layer.alt_base <= 0.0f && layer.alt_top <= 0.0f)
            {
                continue;
            }

            snapshot.cloudLayerCoverages.push_back(layer.coverage);
            snapshot.cloudLayerBasesMeters.push_back(layer.alt_base);
            snapshot.cloudLayerTopsMeters.push_back(layer.alt_top);
        }

        const auto normalizedHeadingDelta = [](float a, float b) {
            float delta = fmodf(fabsf(a - b), 360.0f);
            return delta > 180.0f ? 360.0f - delta : delta;
        };

        const auto layerShearBetween = [&snapshot, &normalizedHeadingDelta](size_t lhs, size_t rhs) {
            const float speedLhs = snapshot.windLayerSpeedsMetersPerSecond.at(lhs);
            const float speedRhs = snapshot.windLayerSpeedsMetersPerSecond.at(rhs);
            const float dirLhs = snapshot.windLayerDirectionsTrueDegrees.at(lhs);
            const float dirRhs = snapshot.windLayerDirectionsTrueDegrees.at(rhs);
            const float directionDeltaDegrees = normalizedHeadingDelta(dirLhs, dirRhs);
            const float crossDirectionalShear = min(speedLhs, speedRhs) * static_cast<float>(sin(GeoMath::degreesToRadians(directionDeltaDegrees)));
            return fabsf(speedLhs - speedRhs) + fabsf(crossDirectionalShear);
        };

        int nearestLayerIndex = -1;
        float nearestLayerAltitudeDelta = 1.0e9f;
        for (size_t i = 0; i < snapshot.windLayerAltitudesMeters.size(); ++i)
        {
            const float altitudeDelta = fabsf(snapshot.windLayerAltitudesMeters.at(i) - altitudeMeters);
            if (altitudeDelta < nearestLayerAltitudeDelta)
            {
                nearestLayerAltitudeDelta = altitudeDelta;
                nearestLayerIndex = static_cast<int>(i);
            }
        }

        if (nearestLayerIndex >= 0)
        {
            const size_t nearest = static_cast<size_t>(nearestLayerIndex);
            if (nearest > 0)
            {
                snapshot.windShearMetersPerSecond = max(snapshot.windShearMetersPerSecond, layerShearBetween(nearest, nearest - 1));
            }
            if (nearest + 1 < snapshot.windLayerAltitudesMeters.size())
            {
                snapshot.windShearMetersPerSecond = max(snapshot.windShearMetersPerSecond, layerShearBetween(nearest, nearest + 1));
            }
        }

        const float gustExcessMetersPerSecond = max(0.0f, snapshot.windGustMetersPerSecond - snapshot.windSpeedMetersPerSecond);
        const float gustFactor = min(0.55f, gustExcessMetersPerSecond / 12.0f);
        const float shearFactor = min(0.45f, snapshot.windShearMetersPerSecond / 18.0f);
        const float precipitationFactor = min(0.20f, max(0.0f, snapshot.precipitationRate) * 0.35f);

        float cloudFactor = 0.0f;
        for (size_t i = 0; i < snapshot.cloudLayerCoverages.size(); ++i)
        {
            const float coverage = snapshot.cloudLayerCoverages.at(i);
            const float baseMeters = i < snapshot.cloudLayerBasesMeters.size() ? snapshot.cloudLayerBasesMeters.at(i) : 0.0f;
            const float topMeters = i < snapshot.cloudLayerTopsMeters.size() ? snapshot.cloudLayerTopsMeters.at(i) : baseMeters;
            if (coverage >= 0.4f && baseMeters - 250.0f <= altitudeMeters && altitudeMeters <= topMeters + 250.0f)
            {
                cloudFactor = max(cloudFactor, min(0.18f, coverage * 0.18f));
            }
        }

        snapshot.turbulenceStrength = max(0.0f, min(1.0f, gustFactor + shearFactor + precipitationFactor + cloudFactor));
#else
        if (m_host)
        {
            static bool s_loggedMissingApi = false;
            if (!s_loggedMissingApi)
            {
                s_loggedMissingApi = true;
                m_host->writeLog("WXSRV|XPLMWeather API unavailable at compile time; returning empty snapshot");
            }
        }
#endif

        return snapshot;
    }
};
