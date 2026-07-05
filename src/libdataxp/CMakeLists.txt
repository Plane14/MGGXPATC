cmake_minimum_required(VERSION 3.9)
project(libdataxp VERSION 0.1.0)

add_library(libdataxp STATIC 
    libdataxp.h
    xpCifpReader.hpp
    xpNavDataReader.hpp
    xpAltitudeReader.hpp
    holdingPattern.hpp
    holdingPattern.cpp
    holdingEntry.hpp
    holdingEntry.cpp
    missedApproach.hpp
    missedApproach.cpp
    procedureValidator.hpp
    procedureValidator.cpp
    navdataManager.hpp
    navdataManager.cpp
    xpAirportReader.cpp
    xpFmsxReader.cpp
)

set_property(TARGET libdataxp PROPERTY CXX_STANDARD 14)
target_include_directories(libdataxp PUBLIC ../libworld)
target_link_libraries(libdataxp libworld) 

if (UNIX)
    target_compile_options(libdataxp PUBLIC -fPIC)
endif()
