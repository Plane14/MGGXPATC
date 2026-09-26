# MGGXPATC Complete Rewrite — Implementation Plan

## Overview

Complete rewrite of all components from scratch, preserving the technology stack (C++, C#, TypeScript) while modernizing architecture. The C# daemon transitions from Orleans grains to gRPC services. All features from current implementation plus planned features from `plans/ARCHITECTURE.md` are included.

**Plan path**: `.kilo/plans/1790105472834-rewrite-all-features.md`

## Technology Stack (Preserved)

| Component | Language | Build | Communication |
|-----------|----------|-------|---------------|
| Core libraries | C++17 | CMake 3.16+ | gRPC/protobuf |
| X-Plane plugin | C++17 | CMake | X-Plane SDK |
| Daemon (server) | C# .NET 8 | dotnet | gRPC/protobuf |
| Console (UI) | TypeScript/React | npm/esbuild | gRPC-Web |
| Tests | C++ (GoogleTest), C# (xUnit), TS (Jest) | CMake, dotnet, npm | — |

## Module Structure

```
rewrite/src/
├── core/           # Core types, geo math, world primitives
├── world/          # World model, airports, flights, taxi net
├── ai/             # AI controllers, pilots, aircraft, maneuvers
├── atc/            # ATC logic: separation, flow, coordination
├── network/        # gRPC server, protobuf definitions, serialization
├── xplane/         # X-Plane plugin, SDK integration
├── daemon/         # C# gRPC server (replaces Orleans)
└── console/        # TypeScript React console (replaces current)
```

## Phase 1: Core Library (`rewrite/src/core/`)

Implement all fundamental types from `libworld.h` in a clean, self-contained module.

### 1.1 Geo Mathematics (`core/geoMath.{hpp,cpp}`)
- `GeoPoint`, `GeoVector`, `LocalPoint`, `UniPoint`
- `GeoMath`: heading math, distance calculations, turn calculations, Great Circle
- `GeoPolygon`, `GeoEdge` with arc/circle/great-circle types
- `Altitude` (Ground/AGL/MSL), `AircraftAttitude`, `Velocity`
- `HaveKey<TKey>` template, `EntityRef<TKey, TEntity>` template
- `AirspaceGeometry` with bounds

### 1.2 World Primitives (`core/worldPrimitives.{hpp,cpp}`)
- `World` class: entity management, time simulation, work item queue, change sets
- `World::ChangeSet`, `World::EntityChangeSet<T, TKey>`
- `WorldBuilder` forward declarations
- `Airport::Header`, `Runway`, `Runway::End`, `Runway::Bounds`
- `ParkingStand`, `ControlledAirspace`, `AirspaceClass`, `RadarScope`
- `ControlFacility`, `ControllerPosition`, `Frequency`
- `Controller` (base), `Actor`, `Pilot`, `Aircraft`, `Flight`, `FlightPlan`
- `FlightPlan::Leg`, `FlightPlan::LegType`, `FlightPlan::Cursor`
- `Clearance`, `Clearance::Type`, `Maneuver`, `Maneuver::Type`, `Maneuver::State`
- `Intent`, `Intent::Direction`, `Intent::Type`
- `Transmission`, `Utterance`, `UtteranceBuilder`
- `TrafficFlow`, `TrafficFlowWindRule`, `TrafficFlowRunwayUse`, `MagneticVariation`
- `TaxiNet`, `TaxiNode`, `TaxiEdge`, `TaxiPath`
- `ActiveZoneMask`, `ActiveZoneMatrix`
- All `ControllerPosition::Type`, `ControlFacility::Type`, `ControlledAirspace::Type` enums

### 1.3 Utility (`core/stlhelpers.{hpp,cpp}`, `core/stateMachine.{hpp,cpp}`)
- `getValueOrThrow`, `tryGetValue`, string helpers
- `StateMachine<S, E>` template
- `DeclineReason` enum
- `TrafficAdvisory` class
- `AircraftPerformanceProfile`, `AircraftPerformanceTable`

### 1.4 Data Providers (`core/dataProviders.{hpp,cpp}`)
- `CIFPReader`, `XPNavDataReader`, `XPAirportReader` interfaces
- `xpCifpReader`, `xpAtcDatReader` for procedure/airport data
- `xpFmsxReader` for FMS data

**Dependencies**: None (foundation layer)

## Phase 2: World Model (`rewrite/src/world/`)

Implement world simulation and management.

### 2.1 World Management (`world/worldManager.{hpp,cpp}`)
- `WorldManager`: owns `World`, drives simulation loop
- `WorldBuilder`: assembles airports, control facilities, airspaces from data
- Time management, heartbeat, change notification

### 2.2 Airport System (`world/airportSystem.{hpp,cpp}`)
- `Airport` implementation with runways, parking stands, taxi net
- `AirportFlow` with wind-based runway selection from apt.dat
- `TrafficFlow` matching (weather, wind, visibility, ceiling)
- `Airport::selectActiveRunways()`, `forceActiveRunways()`
- `SimpleRunwayMutex` with timing thresholds, wake turbulence separation, strip board

### 2.3 Flight System (`world/flightSystem.{hpp,cpp}`)
- `Flight` with phases (Departure, EnRoute, Arrival, TurnAround)
- `FlightPlan` with procedures (SID, STAR, Approach, Holding, GoAround)
- `FlightPlan::rebuildProcedureLegs()` from navdata
- Clearance management, intent management

### 2.4 Taxi Network (`world/taxiNet.{hpp,cpp}`)
- `TaxiNet` with nodes, edges, path finding
- `TaxiPath::find()` with cost functions
- `assignFlightPhaseAllocation()`
- Taxi routing: departure to runway, arrival to gate, runway exit

### 2.5 Airport Geometry (`world/airportGeometry.{hpp,cpp}`)
- Runway bounds calculation
- Taxi edge/node/path geometry
- Parking stand positioning

**Dependencies**: Phase 1

## Phase 3: AI Systems (`rewrite/src/ai/`)

Implement AI controllers, pilots, and aircraft behavior.

### 3.1 AI Aircraft (`ai/aiAircraft.{hpp,cpp}`)
- `AIAircraft` extending `Aircraft`
- Dynamic flight model: updateDynamicFlightModel, moveFor
- Performance envelopes: `getGroundSpeedEnvelopeKt()`, `getVerticalSpeedEnvelopeFpm()`
- Landing/takeoff geometry: `configureArrivalStart()`, `landingTouchdownPointForRunway()`
- Centerline constraint, formation support
- `AircraftPerformanceTable::lookup()` by model ICAO
- `setOnFinal()`, `prepareForApproachRetry()`, `park()`

### 3.2 AI Pilot (`ai/aiPilot.{hpp,cpp}`)
- `AIPilot` extending `Pilot`
- Flight cycle: `maneuverFlightCycle()`, `maneuverFinalToGate()`
- Procedure leg execution: `maneuverProcedureLeg()` with speed/VS/heading control
- Waypoint tracking with proportional heading control
- Descent rate limiting (~4.5 deg glidepath clamp)
- Intent handling: all pilot intents (clearance readback, request, etc.)
- Helicopter support, fighter support, mission profiles
- Departure/arrival logic: check-in, handoff, frequency switching

### 3.3 AI Controller Base (`ai/aiControllerBase.{hpp,cpp}`)
- `AIControllerBase` extending `Controller`
- Intent dispatch table (`AI_CONTROLLER_MAP_INTENT` macro)
- `receiveIntent()`, `progressTo()`, `clearFlights()`
- Sector ownership resolution: `resolveNextControllerBySectorOwnership()`
- Handoff logic: `handoffTrackedFlightsToNextController()`
- Radar tracking, squawk code assignment
- Fallback intent handlers (all pilot intents)
- `findLocalControllerOrThrow()`, `isFlightDepartureRunway()`

### 3.4 Local Controller (`ai/localController.{hpp,cpp}`)
- `LocalController` extending `AIControllerBase`
- Runway mutex management (`m_activeRunwayMutex`)
- `selectActiveRunways()` with wind-based selection from apt.dat
- `checkWindAndUpdateRunways()` (5-minute interval, 15-degree threshold)
- Intent handlers: `PilotReportFinal`, `PilotCheckInWithTower`, `PilotLineUpAndWaitReadback`, `GroundCrossRunwayRequestFromTower`
- Tower operations: `clearForTakeoff()`, `authorizeLineUpAndWait()`, `clearToLand()`, `requestGoAround()`
- Departure handoff to tower: `handoffDeparturesToTower()`

### 3.5 Approach Controller (`ai/approachController.{hpp,cpp}`)
- `ApproachController` extending `AIControllerBase`
- Auto-issue approach clearances at ~25 NM range
- Approach type inference: ILS, VIS, RNAV, RNP, VOR, NDB, GPS
- Altitude floor by approach type (3000ft ILS, 2500ft RNAV/RNP, 0ft Visual)
- Approach clearance issued tracking, cleanup

### 3.6 Maneuver System (`ai/maneuverSystem.{hpp,cpp}`)
- `Maneuver` base class with type, state, parameters
- `Animation`, `DeferredManeuver`, `Sequence`, `Parallel`
- `ManeuverFactory` for creating maneuvers
- `basicManeuverTypes`: Flight, Departure*, Arrival*, Taxi*, FlyVector, FlyDirect
- `await()`, `delay()`, `instantAction()` helpers

### 3.7 Clearances & Intents (`ai/clearanceIntentFactory.{hpp,cpp}`)
- `ClearanceFactory`: all clearance types (IFR, takeoff, landing, approach, go-around, etc.)
- `IntentFactory`: all intent types (pilot check-in, handoff, readback, request, etc.)
- `ClearanceTypes`, `IntentTypes` definitions

**Dependencies**: Phase 1, Phase 2

## Phase 4: ATC Systems (`rewrite/src/atc/`)

Implement all planned architecture features from `plans/ARCHITECTURE.md`.

### 4.1 Separation & Conflict Detection (`atc/separation/{separationManager,wakeTurbulence,verticalSeparation,radarSeparation,conflictDetector,conflictResolver}.{hpp,cpp}`)
- `SeparationManager`: central coordinator
- `WakeTurbulenceCalculator`: time-based separation (ICAO Doc 4444 Table 8-1), time-to-vacate
- `VerticalSeparationRules`: 1000ft below FL290, 2000ft above, RVSM checks
- `RadarSeparationMinima`: 3NM approach, 5NM enroute
- `ConflictDetector`: CPA calculations, 1-5 minute lookahead
- `ConflictResolver`: resolution strategy selection

### 4.2 Flow Management (`atc/flow/{flowManager,arrivalFlowController,departureFlowController,runwayOccupancyTracker,groundStopManager,flowStateMachine}.{hpp,cpp}`)
- `FlowManager`: central coordinator
- `ArrivalFlowController`: arrival rate limiting
- `DepartureFlowController`: departure rate limiting
- `RunwayOccupancyTracker`: occupancy time tracking
- `GroundStopManager`: ground stop/resume logic
- `FlowStateMachine`: Normal → Metered → GroundStop → Resume

### 4.3 Controller Coordination (`atc/coordination/{controllerCoordinator,handoffProtocol,sharedRadarScope,coordinationMessage,workloadModel}.{hpp,cpp}`)
- `ControllerCoordinator`: multi-controller coordination
- `HandoffProtocol`: handoff state machine with timeouts
- `SharedRadarScope`: shared flight data between controllers
- `CoordinationMessage`: inter-facility coordination
- `WorkloadModel`: controller workload tracking

### 4.4 Enhanced Navdata (`atc/navdata/{navdataManager,procedureValidator,holdingPattern,holdingEntry,missedApproach}.{hpp,cpp}`)
- `NavdataManager`: central coordinator
- `ProcedureValidator`: RNAV/RNP validation, constraint checking
- `HoldingPattern`: standard timing (1/1.5 min), entry procedures (direct, parallel, teardrop, overhead)
- `HoldingEntry`: entry procedure selection
- `MissedApproach`: branch selection, constraint validation

### 4.5 Taxi Enhancements (`atc/taxi/{taxiRouter,dynamicEdgeWeights,taxiSequencer}.{hpp,cpp}`)
- `TaxiRouter`: dynamic edge weights, congestion-aware routing
- `DynamicEdgeWeights`: time-based weights, priority adjustments
- `TaxiSequencer`: instruction sequencing, hold-short timing

**Dependencies**: Phase 1, Phase 2, Phase 3

## Phase 5: Network Layer (`rewrite/src/network/`)

Implement gRPC-based inter-component communication (replacing libserver TCP + protobuf and Orleans).

### 5.1 Protobuf Definitions (`network/proto/`)
- `world.proto`: core types (GeoPoint, Aircraft, Airport, Runway, etc.) — expanded from current libserver proto
- `daemon.proto`: daemon services (WorldGrain, AircraftGrain, ControllerGrain, etc.)
- `comms.proto`: communication (Intent, Transmission, Clearance, Frequency, Voice)
- `navdata.proto`: navdata types (CIFP, procedures, navaids)

### 5.2 gRPC Server (`network/grpcServer.{hpp,cpp}`)
- C++ gRPC server (replaces `libserver` TCP server on port 9002)
- Service interfaces matching daemon contracts
- Thread pool, connection management, authentication

### 5.3 gRPC Client (`network/grpcClient.{hpp,cpp}`)
- C++ client for connecting to C# daemon
- C# client for connecting to C++ core
- Reconnection logic, load balancing

### 5.4 Serialization (`network/serialization.{hpp,cpp}`)
- Protocol buffer conversion: C++ ↔ protobuf ↔ C#
- `ProtocolConverter` expanded for all types
- Version negotiation, backward compatibility

**Dependencies**: Phase 1 (core types needed for proto generation)

## Phase 6: X-Plane Plugin (`rewrite/src/xplane/`)

Implement the X-Plane plugin with SDK integration.

### 6.1 Plugin Entry (`xplane/pluginInstance.{hpp,cpp}`)
- `PluginInstance` with state machine (Stopped, Assembling, Assembled, Running, Failed)
- `PluginHostServices`: host services abstraction
- X-Plane SDK integration: DataRef subscriptions, commands, callbacks
- `PluginWorldLoader`: world assembly from data files

### 6.2 Aircraft Objects (`xplane/aircraftObjects.{hpp,cpp}`)
- `Xpmp2AircraftObjectService`: XPMP2 integration for multiplayer
- CSL model loading, aircraft object management
- User aircraft/pilot management

### 6.3 Communication (`xplane/communication.{hpp,cpp}`)
- `Frequency` management (radio frequencies, transmission queue)
- `TextToSpeechService`: TTS integration (OpenAL, speech libraries)
- `XPLMNavRef` wrapper for navaid queries
- `MSACalculator`, `MORACalculator` via XPLM terrain probing

### 6.4 X-Plane SDK Integration (`xplane/sdkIntegration.{hpp,cpp}`)
- `SDKIntegration`: X-Plane SDK lifecycle
- `ATCCommands`: custom ATC command registration
- `DataRefSubscriptions`: efficient DataRef management
- `SubscriptionManager`: change notification callbacks
- `XPLMWeatherService`: weather from X-Plane
- `Configuration`: plugin configuration management

### 6.5 User Interface (`xplane/pluginMenu.{hpp,cpp}`)
- Plugin menu, tool panel
- Console integration (Electron IPC)

### 6.6 Speech (`xplane/speech.{hpp,cpp}`)
- `SpeechSoundBuffer`, `SpeechSoundPlayer`
- `XplmSpeakStringTtsService`: X-Plane speech
- `NativeTextToSpeechService`: platform-native TTS
- `TranscriptInterface`: transcription

**Dependencies**: Phase 1-4, Phase 5

## Phase 7: C# Daemon (`rewrite/src/daemon/`)

Rewrite the Orleans-based daemon as a gRPC C# .NET 8 service.

### 7.1 Daemon Host (`daemon/Program.cs`, `daemon/HostBuilder.cs`)
- .NET 8 Generic Host with gRPC server
- Configuration from appsettings.json
- Logging, health checks, metrics

### 7.2 World Services (`daemon/Services/WorldService.cs`, `daemon/Services/AirportService.cs`)
- `WorldService`: airport management, world state
- `AirportService`: airport queries, traffic flow, runway selection
- Replaces `WorldGrain`, `AirportGrain`, `LlhzAirportGrain`

### 7.3 Traffic Services (`daemon/Services/TrafficService.cs`, `daemon/Services/AircraftService.cs`)
- `AircraftService`: aircraft lifecycle, CSL/model data
- `PilotFlyingService`: AI pilot behavior (flight cycle, procedures)
- `ControllerService`: AI controller behavior (separation, clearances, intents)
- Replaces `AircraftGrain`, `AIControllerGrain`, `AIPilotFlyingGrain`, `AIAircraftGrain`

### 7.4 Communication Services (`daemon/Services/CommsService.cs`, `daemon/Services/RadioService.cs`)
- `RadioService`: radio station management, transmission queue
- `CommsService`: intent/clearance/transmission management
- `VerbalizationService`: speech synthesis
- Replaces `RadioStationGrain`, `AIRadioOperatorGrain`, `GroundStationRadioMediumGrain`

### 7.5 Conversation Services (`daemon/Services/ConversationService.cs`)
- `ConversationHandler`: conversation management
- `SpeechService`: TTS orchestration
- Replaces conversation-related Orleans grains

### 7.6 gRPC Contracts (`daemon/Contracts/`)
- C# protobuf contracts matching `network/proto/`
- `IAviationDatabase`, `IAirportData`, `IAircraftData`, etc.
- Intent, Transmission, Clearance, Frequency contracts

**Dependencies**: Phase 5 (gRPC), Phase 1-4 (protobuf definitions)

## Phase 8: Console (`rewrite/src/console/`)

Rewrite the Electron/TypeScript console with gRPC-Web.

### 8.1 Console App (`console/src/main/`, `console/src/renderer/`)
- Electron main process (window management, gRPC-Web setup)
- React renderer with Redux state management
- Airport view, map view, tool panel, settings

### 8.2 gRPC-Web Client (`console/src/proto/`, `console/src/services/`)
- TypeScript protobuf/gRPC-Web client generated from `network/proto/`
- World service client, traffic service client, comms service client

### 8.3 Console Components (`console/src/renderer/components/`)
- Graph component (airport layout visualization)
- Tool panel (zoom, pinpoint, taxi tools)
- Application view, airport view, settings

### 8.4 Protocol (`console/src/__protocol/`)
- TypeScript protocol definitions
- IPC bridge to Electron main

**Dependencies**: Phase 5 (gRPC), Phase 7 (daemon running)

## Phase 9: Testing

### 9.1 C++ Tests
- `core_test/`: geo math, altitude, frequency, entity ref
- `world_test/`: airport, runway, taxi net, flight plan
- `ai_test/`: aircraft, pilot, controller, maneuver
- `atc_test/`: separation, flow, coordination, navdata
- `network_test/`: serialization, protobuf conversion

### 9.2 C# Tests
- `daemon_test/`: world services, traffic services, comms services
- Integration tests for gRPC endpoints

### 9.3 TypeScript Tests
- `console/test/`: unit tests, component tests, e2e tests

### 9.4 Integration Tests
- Full simulation scenario: AI traffic at LLLL (London Heathrow)
- End-to-end: pilot → controller → daemon → plugin loop
- Performance benchmarks

**Dependencies**: All phases

## Migration Path

1. **Phase 1-3** can be developed in parallel with existing code (new libraries side-by-side)
2. **Phase 4-5** build on completed core modules
3. **Phase 6** requires C++ core + daemon gRPC
4. **Phase 7-8** require gRPC infrastructure
5. **Phase 9** runs throughout development
6. **Cutover**: When all phases pass integration, switch plugin entry point to new code

## Key Interfaces (Cross-Module Contracts)

| Interface | Provider | Consumer | Protocol |
|-----------|----------|----------|----------|
| World data API | C++ core | C# daemon | gRPC/protobuf |
| Aircraft state API | C++ core | C# daemon | gRPC/protobuf |
| Intent/clearance API | C# daemon | C++ core | gRPC/protobuf |
| Airport/navdata API | C++ core | C# daemon | gRPC/protobuf |
| Console ↔ Daemon | C# daemon | TS console | gRPC-Web |
| X-Plane ↔ Plugin | X-Plane | C++ plugin | X-Plane SDK |

## Risks & Mitigations

| Risk | Mitigation |
|------|-----------|
| Feature regression during rewrite | Parallel development; existing code remains functional until cutover |
| Performance degradation | Profile-critical paths early; maintain benchmark tests |
| gRPC latency for real-time sim | Keep time-critical paths in C++ plugin; use gRPC for AI coordination only |
| Protobuf schema drift | Schema-first development; version all proto files |
| Orleans lock-in knowledge | C# team uses standard ASP.NET Core gRPC patterns instead |
| Massive scope | Phase-gated delivery; each phase independently testable |

## Open Questions (Resolved)

1. **C# daemon AI logic scope** → **Split scope**: Real-time flight model stays in C++ (X-Plane SDK requirement); C# daemon handles high-level AI (intent generation, clearances, coordination, handoffs, separation). gRPC for state sync.
2. **Console gRPC-Web proxy** → **Direct gRPC-Web**: Electron's Chromium supports gRPC-Web natively; Node.js proxy adds unnecessary complexity.
3. **Shared protobuf schema ownership** → **Single source in `network/proto/`**: C++ team owns schema changes; other languages consume via `protoc` generation. Version all `.proto` files from v1.
4. **X-Plane plugin process model** → **Keep simulation in plugin**: X-Plane SDK requires real-time updates in the plugin process; daemon handles AI decision-making.

## Phase 1 Status: ✅ COMPLETE

### Files Created
| File | Description |
|------|-------------|
| `rewrite/src/core/geoMath.hpp/cpp` | GeoPoint, GeoVector, UniPoint, GeoPolygon, GeoMath, Altitude, AircraftAttitude, HaveKey, EntityRef, AirspaceGeometry, RunwayBounds |
| `rewrite/src/core/worldPrimitives.hpp` | HostServices, Actor, Frequency, Transmission, Utterance, Intent, Clearance, Maneuver, Aircraft, Flight, Pilot, World, Airport, Runway, etc. |
| `rewrite/src/core/worldPrimitives.cpp` | UniPoint, RunwayBounds implementations |
| `rewrite/src/core/stlhelpers.hpp/cpp` | tryGetValue, getValueOrThrow, stringStartsWith, initTime, DECLARE_ENUM_BITWISE_OP |
| `rewrite/src/core/stateMachine.hpp` | StateMachine<S, E> template with DeclarativeState, Transition |
| `rewrite/src/core/dataProviders.hpp` | CIFPReader, XPNavDataReader, XPAirportReader interfaces |
| `rewrite/src/core/CMakeLists.txt` | Core library target (C++17, no external deps) |
| `src/CMakeLists.txt` (modified) | Added `add_subdirectory` for rewrite core |

### Known Issues (resolved during compilation/implementation)
- **Header ordering**: `worldPrimitives.hpp` has Aircraft before World which causes `World::OnChangesCallback` dependency issues. Resolution: during compilation, move World class definition before Aircraft (World doesn't depend on Aircraft, only forward-declares it).
- **Missing implementations**: Many inline methods in worldPrimitives.hpp are st implementations (empty bodies or `{}`). Full implementations will be added during Phase 2+.

## Phase 2 Status: ✅ COMPLETE

### Files Created
| File | Description |
|------|-------------|
| `rewrite/src/world/worldManager.{hpp,cpp}` | WorldManager (simulation loop, tick processing), WorldBuilder factory |
| `rewrite/src/world/airportSystem.{hpp,cpp}` | Runway bounds, active runway selection, parking stands, traffic flow matching |
| `rewrite/src/world/flightSystem.{hpp,cpp}` | Flight creation, progressTo, clearance management, procedure leg rebuilding, FlightPlan helpers |
| `rewrite/src/world/taxiNet.{hpp,cpp}` | TaxiNet path finding (Dijkstra), node/edge management, departure/arrival taxi routing |
| `rewrite/src/world/taxiNet.hpp` | taxiNet declarations (separate from .cpp due to template) |
| `rewrite/src/world/airportGeometry.{hpp,cpp}` | Runway bounds calculation, geometry helpers |
| `rewrite/src/world/CMakeLists.txt` | world_model library target, depends on core |
| `src/CMakeLists.txt` (modified) | Added add_subdirectory for world library |

### Phase 2 Implementation Summary

The world model layer provides:
- **WorldManager**: Simulation loop wrapper around World, tick-based processing
- **Airport System**: Active runway selection, ILS preference, parallel runway handling, parking stand lookup, traffic flow matching
- **Flight System**: Flight/FlightPlan lifecycle, clearance management, procedure leg rebuilding stub (CIFP/navdata reads delegated to Phase 4), RVSM altitude assignment
- **Taxi Net**: Dijkstra-based path finding, departure/arrival taxi routing, runway exit finding, flight phase allocation
- **Airport Geometry**: Runway bounds calculation from heading/width, coordinate helpers

### Phase 3: AI Systems — NEXT
