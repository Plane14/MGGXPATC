cmake_minimum_required(VERSION 3.5)
project(libworld VERSION 0.1.0)

add_library(libworld STATIC
    airlineReferenceTable.cpp
    airlineReferenceTable.hpp
    aircraftTypeReferenceTable.hpp
    aircraftTypeReferenceTable.cpp
    actor.cpp
    aircraft.cpp
    airport.cpp
    airportFlow.cpp
    airportFlow.hpp
    airspaceClass.cpp
    altitude.cpp
    arrivalFlowController.cpp
    arrivalFlowController.hpp
    basicManeuverTypes.hpp
    clearanceTypes.hpp
    controlFacility.cpp
    controllerPosition.cpp
    controllerCoordinator.cpp
    controllerCoordinator.hpp
    conflictDetector.cpp
    conflictDetector.hpp
    conflictResolver.cpp
    conflictResolver.hpp
    coordinationMessage.cpp
    coordinationMessage.hpp
    departureFlowController.cpp
    departureFlowController.hpp
    flight.cpp
    flowManager.cpp
    flowManager.hpp
    flowStateMachine.cpp
    flowStateMachine.hpp
    frequency.cpp
    geoMath.cpp
    geoPoint.cpp
    groundStopManager.cpp
    groundStopManager.hpp
    handoffProtocol.cpp
    handoffProtocol.hpp
    intent.cpp
    intentFactory.hpp
    intentTypes.cpp
    intentTypes.hpp
    libworld.cpp
    libworld.h
    maneuver.cpp
    radarSeparationMinima.cpp
    radarSeparationMinima.hpp
    runway.cpp
    runwayOccupancyTracker.cpp
    runwayOccupancyTracker.hpp
    separationManager.cpp
    separationManager.hpp
    sharedRadarScope.cpp
    sharedRadarScope.hpp
    simplePhraseologyService.hpp
    state.h
    stlhelpers.cpp
    stlhelpers.h
    taxiEdge.cpp
    taxiNet.cpp
    taxiNode.cpp
    taxiPath.cpp
    uniPoint.cpp
    utteranceBuilder.cpp
    verticalSeparationRules.cpp
    verticalSeparationRules.hpp
    wakeTurbulenceCalculator.cpp
    wakeTurbulenceCalculator.hpp
    workloadModel.cpp
    workloadModel.hpp
    world.cpp
    worldBuilder.cpp
    worldHelper.hpp
    stateMachine.hpp
    hostServices.cpp
)

set_property(TARGET libworld PROPERTY CXX_STANDARD 14)

if (UNIX)
    target_compile_options(libworld PUBLIC -fPIC)
endif()
