# Self-Driving Bot (SDB)

**Automated Driving System for Open-World Roads (Unity)**

Self-Driving Bot (SDB) is a Unity-based Automated Driving System (ADS) prototype built for open-world, two-way road driving (no intersections). It bridges the gap between basic waypoint traffic AI and more structured ADS-style pipelines: perception -> planning -> behavior -> control.

It procedurally builds a lane path from road scanning, follows it with Pure Pursuit steering, regulates speed with PID, and adds safety behaviors for traffic lights and front-car following (IDM-style) using a lightweight priority arbitration layer.



## Demo

- SDB run (YouTube): https://youtu.be/r0TC5QLBo5A
- BeamNG ADS run (YouTube): https://youtu.be/AmPZhkvouV4

add a short GIF here (10–20s)
Media/demo.gif


## What this project does

- Perceives the road using arc raycast scanning against Road colliders (tag-based).
- Builds a rolling waypoint window ahead of the ego vehicle (sliding refresh).
- Generates a smooth path with Catmull-Rom splines.
- Follows the path using Pure Pursuit steering.
- Plans speed using curvature look-ahead and applies throttle/brake via PID speed control.
- Adds safety behaviors:
  - Front-car following via an IDM-style acceleration model
  - Traffic light stopping (red/yellow) with 'stop before the object' behavior
- Arbitrates behaviors with a unified ControlCommand and priority selection.

## Key design note (important)
**Steering always comes from the path follower.** 

Safety behaviors (traffic lights / front car) primarily affect throttle and braking. The final command’s SteeringAngle is overwritten by the path follower’s steering every physics step.

This is intentional: lateral stability stays consistent while longitudinal behaviors override speed safely.

System architecture (high-level)
flowchart LR
  Road[Road collider tagged "Road"] --> Scan[WaypointManager / Arc raycasts]
  Scan --> WPs[Waypoints / (sliding window)]
  WPs --> Spline[SplineManager / Catmull-Rom spline samples]
  Spline --> Actions[Actions / EvaluatePath / EvaluateTrafficLight / GetCar]
  WM[WorldModel/Cars/Lights/Obstacles] --> Track[TrackingManager / closest object ahead]
  Track --> Actions
  Actions --> Cmd[ControlCommand / (throttle, brake, steer, priority)]
  Cmd --> Driver[Driver / priority selection]
  Driver --> Sim[SimulationHandler / WheelCollider actuation]

## How it works
### 1) Road scanning -> Waypoints (WaypointManager)

The ego vehicle "looks" ahead using a fan/arc of downward raycasts against colliders tagged Road. From the hits, it estimates left/right road edges, infers a lane center, and spawns a waypoint at a fixed spacing.

A small rolling window of waypoints is maintained:
- when the vehicle reaches the target waypoint, the first waypoint is removed
- a new waypoint is scanned and appended
- the spline is updated incrementally

### 2) Waypoints -> Spline path (SplineManager)

Waypoints are converted into a dense sample path using Catmull-Rom interpolation with configurable subdivisionsPerSegment. This provides:
- a smooth tracking path
- curvature estimation (used for speed adaptation)
- nearest spline point lookup

### 3) Path following + speed planning (Actions)

- Steering (Pure Pursuit): selects a look-ahead point on the spline based on speed and computes a steering angle (degrees).
- Speed planning: reduces target speed as curvature increases (curve look-ahead).
- Speed control: a PID controller produces throttle/brake output based on the target speed vs current speed.

### 4) Safety behaviors + priority arbitration (Driver + Actions)

Traffic light and front-car behaviors are evaluated in parallel and return ControlCommands with priorities. The Driver picks the highest priority command each physics step.

When priority changes, PID state is reset to reduce "carry over" between modes.

## Getting started

### Requirements

1) Unity: Check ProjectSettings/ProjectVersion.txt for the exact editor version used.
2) Physics: WheelCollider-based vehicle setup (included in SimulationHandler)
3) Input: None required (fully autonomous)

## Quickstart (clone → play)

1) Open the project in Unity.
2) Open the demo scene (recommended to include one, e.g. Assets/Scenes/Demo.unity).
3) Ensure your drivable road colliders are tagged: Road
4) Place the SDB vehicle prefab in the scene.
5) Ensure there is exactly one WorldModel in the scene.
6) Press Play.

## Scene setup checklist

### A) The ego vehicle (same GameObject)

Add/confirm these components:
- SimulationHandler (WheelCollider actuation + Ackermann steering)
- WaypointManager (road scan + rolling waypoint window)
- SplineManager (Catmull-Rom spline + curvature)
- Actions (path following + traffic/car behaviors + PID)
- TrackingManager (selects closest relevant object ahead from WorldModel)
- Driver (priority selection + applies final command)

Also required:
- a Rigidbody assigned to SimulationHandler.carRigidbody
- WheelColliders assigned in SimulationHandler (front/rear, left/right)
- a waypoint prefab assigned in WaypointManager.waypointPrefab

### B) World model (one per scene)

Add a WorldModel object somewhere in the scene.

### C) Detectable objects (must register into WorldModel)

To be "seen" by the ADS, objects must have the registration scripts:

- Vehicles: CarMono
- Traffic lights: TrafficLightMono
- Static obstacles: ObstacleMono

Note: WorldModel.Update() is currently a placeholder. Registration happens via these *Mono scripts at runtime.

## Configuration (Inspector highlights)

### Path following & speed
- Actions.desiredSpeed — cruise speed target (see Units note below)
- Actions.minLookAheadDistance / maxLookAheadDistance — steering stability vs responsiveness
- Actions.lookaheadGain — increases look-ahead with speed
- Actions.curvatureThreshold — curve sensitivity threshold
- Actions.motorPropGain / motorIntegralGain / motorDerivativeGain — PID tuning

### Scanning & path density

- WaypointManager.wpSpacing — waypoint spacing (meters)
- WaypointManager.waypointCount — size of rolling window
- WaypointManager.waypointReachThreshold — refresh trigger distance
- SplineManager.subdivisionsPerSegment — spline smoothness vs cost

### Behavior arbitration

- Path command uses a baseline priority (cruise).
- Traffic lights and front-car behavior compute a dynamic priority based on urgency (gap + required deceleration).
- Driver selects the max priority each physics step.

## Units note (read this if you tune the system)

SimulationHandler.CURRENT_SPEED is computed from Rigidbody velocity and stored in km/h.

Some parts of the longitudinal behavior (IDM) operate on Rigidbody velocity magnitude (which is m/s). For consistent tuning, it’s recommended to standardize units (either fully km/h or fully m/s) when polishing the project.

## Debugging & visualization

- Spline visualization: SplineManager draws spline segments and sample points in Gizmos.
- Path/lane visualization + look-ahead point: Actions.OnDrawGizmosSelected()
- Obstacle debug lines: Actions draws lines to detected target objects.

## Evaluation summary (thesis)

A thesis evaluation compared SDB against BeamNG.drive’s built-in ADS on a ~1.9 km route using standardized indicators:
- time / average speed
- traffic-light compliance
- off-lane events

Reported metrics (thesis):
- SDB: 233s, 28.7 km/h avg, 13 off-lane events
- BeamNG ADS: 261s, 27.63 km/h avg, 18 off-lane events

(See thesis for methodology details and limitations, especially around traffic light setup on the BeamNG route.)

## Limitations

- Designed for two-way roads without intersections (no lane graph / junction logic yet).
- Road perception relies on colliders tagged Road and assumes usable raycast hits.
- No occlusion reasoning (object relevance is primarily “in front + within distance”).
- Driving smoothness can show steering oscillations / occasional unnecessary braking (PID and look-ahead tuning opportunities).

## Future work

- Intersection recognition + lane graph (multi-lane / routing)
- Better perception (occlusion, lane boundaries, road markings)
- Behavior layering (yielding, overtakes, hazard awareness)
- Human-like profiles (patience/aggression, comfort braking)
- Unit standardization + tuning tools (in-editor graphs, replayable benchmarks)

## Citation

If you use or reference this work academically:

@thesis{kapitsakis2025ads,
  title  = {ADS in Video Games: Analysis, Development, and Evaluation},
  author = {Emmanouil Kapitsakis},
  year   = {2025},
  school = {SAE Creative Media College, Athens Campus}
}
