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
        shared_ptr<ControllerPosition> tryGetLocalTowerPosition(shared_ptr<Airport> airport, const GeoPoint& location)
        {
            if (!airport)
            {
                return nullptr;
            }

            try
            {
                return airport->localAt(location);
            }
            catch(const exception&)
            {
            }

            auto tower = airport->tower();
            if (!tower)
            {
                return nullptr;
            }

            return tower->tryFindPosition(ControllerPosition::Type::Local, location);
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
            return airport->clearanceDeliveryAt(flight->aircraft()->location());
        }
        
        shared_ptr<ControllerPosition> getDepartureGround(shared_ptr<Flight> flight)
        {
            auto airport = getDepartureAirport(flight);
            if (!airport)
            {
                return nullptr;
            }
            return airport->groundAt(flight->aircraft()->location());
        }

        shared_ptr<ControllerPosition> getDepartureTower(shared_ptr<Flight> flight)
        { 
            auto airport = getDepartureAirport(flight);
            if (!airport)
            {
                throw runtime_error("WorldHelper::getDepartureTower: departure airport not loaded");
            }
            const GeoPoint location = flight->aircraft()->location();
            if (auto local = tryGetLocalTowerPosition(airport, location))
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
                if (auto fallback = tower->tryFindPosition(fallbackType, location))
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
            if (auto local = tryGetLocalTowerPosition(airport, landingPoint))
            {
                return local;
            }

            throw runtime_error("WorldHelper::getArrivalTower: no local/tower controller position found");
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
            auto ground = airport->groundAt(landingPoint);
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
                flight->aircraft()->location());
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

        shared_ptr<ControllerPosition> tryGetEnRouteArea(shared_ptr<Flight> flight)
        {
            if (!flight)
            {
                return nullptr;
            }

            const GeoPoint location = flight->aircraft()->location();
            const auto tryAirportArea = [location](shared_ptr<Airport> airport) {
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
                    if (auto candidate = airport->tower()->tryFindPosition(type, location))
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

        shared_ptr<ControllerPosition> tryGetArrivalApproach(shared_ptr<Flight> flight, const GeoPoint& location)
        {
            auto airport = getArrivalAirport(flight);
            if (!airport || !airport->tower())
            {
                return nullptr;
            }

            const vector<ControllerPosition::Type> fallbackTypes = {
                ControllerPosition::Type::Approach,
                ControllerPosition::Type::Area,
                ControllerPosition::Type::Local,
                ControllerPosition::Type::Ground,
                ControllerPosition::Type::Departure
            };

            for (const auto fallbackType : fallbackTypes)
            {
                if (auto fallback = airport->tower()->tryFindPosition(fallbackType, location))
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
