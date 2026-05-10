// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#pragma once

#include <string>
#include <sstream>
#include "libworld.h"
#include "clearanceTypes.hpp"
#include "intentTypes.hpp"

using namespace std;


namespace world
{
    class WorldHelper
    {
    private:
        shared_ptr<HostServices> m_host;
    private:
        float altitudeFeetMsl(shared_ptr<Flight> flight) const
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
            case Altitude::Type::AGL:
                return altitude.feet() + m_host->queryTerrainElevationAt(flight->aircraft()->location());
            case Altitude::Type::Ground:
                return m_host->queryTerrainElevationAt(flight->aircraft()->location());
            default:
                return -1.0f;
            }
        }

        shared_ptr<ControllerPosition> tryGetLocalTowerPosition(
            shared_ptr<Airport> airport,
            const GeoPoint& location,
            float altitudeMslFeet = -1.0f)
        {
            if (!airport)
            {
                return nullptr;
            }

            try
            {
                return airport->localAt(location, altitudeMslFeet);
            }
            catch(const exception&)
            {
            }

            auto tower = airport->tower();
            if (!tower)
            {
                return nullptr;
            }

            return tower->tryFindPosition(ControllerPosition::Type::Local, location, altitudeMslFeet);
        }
    public:
        WorldHelper(shared_ptr<HostServices> _host) : 
            m_host(_host)
        {
        }
    public:
        shared_ptr<Airport> getDepartureAirport(shared_ptr<Flight> flight)
        {
            return m_host->getWorld()->getAirport(flight->plan()->departureAirportIcao());
        }

        shared_ptr<Airport> getArrivalAirport(shared_ptr<Flight> flight)
        {
            return m_host->getWorld()->getAirport(flight->plan()->arrivalAirportIcao());
        }

        shared_ptr<ControllerPosition> getClearanceDelivery(shared_ptr<Flight> flight)
        {
            auto airport = getDepartureAirport(flight);
            if (!airport)
            {
                return nullptr;
            }
            return airport->clearanceDeliveryAt(flight->aircraft()->location(), altitudeFeetMsl(flight));
        }
        
        shared_ptr<ControllerPosition> getDepartureGround(shared_ptr<Flight> flight)
        {
            auto airport = getDepartureAirport(flight);
            if (!airport)
            {
                return nullptr;
            }
            return airport->groundAt(flight->aircraft()->location(), altitudeFeetMsl(flight));
        }

        shared_ptr<ControllerPosition> getDepartureTower(shared_ptr<Flight> flight)
        { 
            auto airport = getDepartureAirport(flight);
            if (!airport)
            {
                throw runtime_error("WorldHelper::getDepartureTower: departure airport not loaded");
            }
            const GeoPoint location = flight->aircraft()->location();
            const float altitudeMslFeet = altitudeFeetMsl(flight);
            if (auto local = tryGetLocalTowerPosition(airport, location, altitudeMslFeet))
            {
                return local;
            }

            auto tower = airport->tower();
            if (!tower)
            {
                throw runtime_error("WorldHelper::getDepartureTower: airport has no tower/controller facility");
            }

            const vector<ControllerPosition::Type> fallbackTypes = {
                ControllerPosition::Type::Local,
                ControllerPosition::Type::Departure,
                ControllerPosition::Type::Approach
            };

            for (const auto fallbackType : fallbackTypes)
            {
                if (auto fallback = tower->tryFindPosition(fallbackType, location, altitudeMslFeet))
                {
                    return fallback;
                }
            }

            if (!tower->positions().empty())
            {
                return tower->positions().front();
            }

            throw runtime_error("WorldHelper::getDepartureTower: no suitable controller position found");
        }

            shared_ptr<ControllerPosition> tryGetDepartureTower(shared_ptr<Flight> flight)
            {
                try
                {
                    return getDepartureTower(flight);
                }
                catch(const exception&)
                {
                    return nullptr;
                }
            }

        shared_ptr<ControllerPosition> getArrivalTower(shared_ptr<Flight> flight, const GeoPoint& landingPoint)
        { 
            auto airport = getArrivalAirport(flight);
            if (!airport)
            {
                throw runtime_error("WorldHelper::getArrivalTower: arrival airport not loaded");
            }
            const float altMsl = altitudeFeetMsl(flight);
            if (auto local = tryGetLocalTowerPosition(airport, landingPoint, altMsl))
            {
                return local;
            }

            auto tower = airport->tower();
            if (!tower)
            {
                throw runtime_error("WorldHelper::getArrivalTower: airport has no tower/controller facility");
            }

            const vector<ControllerPosition::Type> fallbackTypes = {
                ControllerPosition::Type::Local,
                ControllerPosition::Type::Approach,
                ControllerPosition::Type::Departure
            };

            for (const auto fallbackType : fallbackTypes)
            {
                if (auto fallback = tower->tryFindPosition(fallbackType, landingPoint, altMsl))
                {
                    return fallback;
                }
            }

            if (!tower->positions().empty())
            {
                return tower->positions().front();
            }

            throw runtime_error("WorldHelper::getArrivalTower: no suitable controller position found");
        }

        shared_ptr<ControllerPosition> tryGetArrivalTower(shared_ptr<Flight> flight, const GeoPoint& landingPoint)
        {
            try
            {
                return getArrivalTower(flight, landingPoint);
            }
            catch(const exception&)
            {
                return nullptr;
            }
        }

        shared_ptr<ControllerPosition> getArrivalGround(shared_ptr<Flight> flight, const GeoPoint& landingPoint)
        { 
            auto airport = getArrivalAirport(flight);
            auto ground = airport->groundAt(landingPoint, altitudeFeetMsl(flight));
            return ground;
        }

        shared_ptr<ControllerPosition> tryGetDepartureGround(shared_ptr<Flight> flight)
        {
            try
            {
                return getDepartureGround(flight);
            }
            catch(const exception&)
            {
                return tryGetDepartureTower(flight);
            }
        }

        shared_ptr<ControllerPosition> tryGetArrivalGround(shared_ptr<Flight> flight, const GeoPoint& landingPoint)
        {
            try
            {
                return getArrivalGround(flight, landingPoint);
            }
            catch(const exception&)
            {
                return tryGetArrivalTower(flight, landingPoint);
            }
        }

        shared_ptr<ControllerPosition> tryGetDeparture(shared_ptr<Flight> flight)
        { 
            auto airport = getDepartureAirport(flight);
            if (!airport || !airport->tower())
            {
                return nullptr;
            }
            return airport->tower()->tryFindPosition(
                ControllerPosition::Type::Departure,
                flight->aircraft()->location(),
                altitudeFeetMsl(flight));
        }

        shared_ptr<ControllerPosition> tryGetDepartureOrArea(shared_ptr<Flight> flight)
        {
            auto departure = tryGetDeparture(flight);
            if (departure)
            {
                return departure;
            }

            return tryGetEnRouteArea(flight);
        }

        shared_ptr<ControllerPosition> tryGetEnRouteArea(shared_ptr<Flight> flight, float altitudeMslFeet = -1.0f)
        {
            if (!flight)
            {
                return nullptr;
            }

            const GeoPoint location = flight->aircraft()->location();
            if (altitudeMslFeet < 0.0f)
            {
                altitudeMslFeet = altitudeFeetMsl(flight);
            }

            const auto tryAirportArea = [location, altitudeMslFeet](shared_ptr<Airport> airport) {
                if (!airport || !airport->tower())
                {
                    return shared_ptr<ControllerPosition>();
                }

                const vector<ControllerPosition::Type> types = {
                    ControllerPosition::Type::Area,
                    ControllerPosition::Type::Departure
                };

                for (const auto type : types)
                {
                    if (auto candidate = airport->tower()->tryFindPosition(type, location, altitudeMslFeet))
                    {
                        return candidate;
                    }
                }

                return shared_ptr<ControllerPosition>();
            };

            if (auto departureArea = tryAirportArea(getDepartureAirport(flight)))
            {
                return departureArea;
            }

            return tryAirportArea(getArrivalAirport(flight));
        }

        shared_ptr<ControllerPosition> tryGetArrivalApproach(
            shared_ptr<Flight> flight,
            const GeoPoint& location,
            float altitudeMslFeet = -1.0f)
        {
            auto airport = getArrivalAirport(flight);
            if (!airport || !airport->tower())
            {
                return nullptr;
            }

            if (altitudeMslFeet < 0.0f)
            {
                altitudeMslFeet = altitudeFeetMsl(flight);
            }

            // Fallback order for airborne arrivals: Approach is primary, then
            // Area (center), Departure (which often covers approach sectors at
            // smaller airports), and only then Local/Ground as last resorts.
            const vector<ControllerPosition::Type> fallbackTypes = {
                ControllerPosition::Type::Approach,
                ControllerPosition::Type::Area,
                ControllerPosition::Type::Departure,
                ControllerPosition::Type::Local,
                ControllerPosition::Type::Ground
            };

            for (const auto fallbackType : fallbackTypes)
            {
                if (auto fallback = airport->tower()->tryFindPosition(fallbackType, location, altitudeMslFeet))
                {
                    return fallback;
                }
            }

            return nullptr;
        }

        shared_ptr<Intent> verbalize(shared_ptr<Intent> intent)
        {
            //no-op
            // Utterance utterance = m_verbalizer.verbalizeIntent(intent);
            // intent->setTransmissionText(utterance.plainText());
            return intent;
        }

        GeoPoint getLandingPoint(shared_ptr<Flight> flight)
        {
            const auto& runwayEnd = getLandingRunwayEnd(flight);
            return runwayEnd.centerlinePoint().geo();
        }

        const Runway::End& getLandingRunwayEnd(shared_ptr<Flight> flight)
        {
            auto airport = getArrivalAirport(flight);
            if (!airport)
            {
                throw runtime_error(
                    "WorldHelper::getLandingRunwayEnd: arrival airport [" +
                    flight->plan()->arrivalAirportIcao() + "] not loaded");
            }
            auto runwayName = flight->plan()->arrivalRunway();
            auto runway = runwayName.empty() ? nullptr : airport->tryFindRunway(runwayName);

            if (!runway)
            {
                runway = airport->findPreferredArrivalRunway();
                if (!runway)
                {
                    throw runtime_error(
                        "WorldHelper::getLandingRunwayEnd: no preferred arrival runway found at [" +
                        airport->header().icao() + "]");
                }

                for (const auto& activeRunwayName : airport->activeArrivalRunways())
                {
                    if (airport->tryFindRunway(activeRunwayName) == runway)
                    {
                        runwayName = activeRunwayName;
                        break;
                    }
                }

                if (runwayName.empty())
                {
                    runwayName = runway->end1().name();
                }
            }

            return runway->getEndOrThrow(runwayName);
        }
    };
}
