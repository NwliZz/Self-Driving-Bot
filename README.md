Self-Driving Bot (SDB) — Automated Driving System for Open-World Games (Unity)

Self-Driving Bot (SDB) is a Unity-based Automated Driving System (ADS) designed to drive safely and efficiently in open-world style road scenarios (two-way roads, no intersections), bridging the gap between simple scripted traffic AI and more advanced driving agents.

Demo

Media/demo.gif (recommended: add a 10–20s GIF of the bot driving + stopping at a red light)

Videos used in the thesis (copy/paste):

SDB run: https://youtu.be/r0TC5QLBo5A
BeamNG ADS run: https://youtu.be/AmPZhkvouV4

What this project does

Builds a drivable lane path procedurally from road scanning and continuously updates it ahead of the vehicle.

Generates a smooth path using Catmull-Rom splines and follows it with Pure Pursuit steering.

Plans speed proactively using curvature look-ahead and applies throttle/brake with a PID controller.

Handles traffic lights and front-car following (IDM-based) with dynamic priority arbitration between behaviors.

Key features
Perception

Road detection via raycast arc scanning to estimate road edges and lane placement.

A lightweight World Model approach for tracking objects (cars, obstacles, traffic lights) in the scene.

Planning

Waypoint sliding window ahead of the ego vehicle (continuous path refresh).

Spline path generation (Catmull-Rom) with configurable subdivisions per segment.

Prediction & Behavior

Front-car handling (IDM) for safe longitudinal following.

Traffic light handling with reliable “stop-before-line” behavior.

Control & Arbitration

Behaviors output a unified ControlCommand (throttle, brake, steering, priority).

A central Driver selects the highest-priority command each frame and resets controllers on priority changes to reduce control carry-over between modes.

System architecture
flowchart LR
  Road[Road collider/tagged "Road"] --> Scan[WaypointManager / Scan\nArc raycast scan]
  Scan --> WPs[Waypoints\n(sliding window)]
  WPs --> Spline[SplineManager\nCatmull-Rom spline]
  Spline --> Actions[Actions\nEvaluatePath / EvaluateTrafficLight / GetCar]
  World[WorldModel\nCars/Obstacles/Lights] --> Track[TrackingManager\nobjects in front]
  Track --> Actions
  Actions --> Cmd[ControlCommand\n(throttle, brake, steer, priority)]
  Cmd --> Driver[Driver\npriority arbitration]
  Driver --> Sim[SimulationHandler\nWheelColliders physics]

How it works (high level)
1) Road → Waypoints

The bot scans the road surface with downward raycasts arranged in an arc. From the detected road hits, it estimates left/right edges and places waypoints on the target lane. Waypoints are refreshed continuously as the vehicle advances (sliding window).

2) Waypoints → Spline Path

Waypoints become control points for a Catmull-Rom spline, producing a dense SplinePath suitable for smooth tracking and curvature measurement.

3) Path following

Steering: Pure Pursuit selects a dynamic look-ahead point on the spline and computes steering toward it.

Speed planning: A curvature look-ahead reduces target speed for sharper curves.

Speed control: A PID controller translates speed error into throttle/brake.

4) Safety behaviors + dynamic priority

Traffic lights and front vehicles are evaluated in parallel. Each behavior returns a ControlCommand with a priority. The Driver applies the command with the highest priority (e.g., urgent braking overrides path cruising).

Getting started
Requirements

Unity version: TODO (fill in your Unity Editor version)

Platform tested: TODO (Windows / macOS / Linux)

Input: SDB drives autonomously (no player input required)

Setup

Open the project in Unity.

Ensure your drivable road meshes/colliders are tagged as: Road.

Place the SDB vehicle prefab in the scene (or create one and add the required components below).

Press Play.

Required components (typical)

On the SDB vehicle (or relevant scene objects), you should have:

Driver

Actions

TrackingManager

WaypointManager

SplineManager

SimulationHandler (WheelCollider-based)

Plus a scene-level:

WorldModel (registers/maintains detectable objects)

Project structure (important scripts)
Script	Role
Driver.cs	Main orchestrator: collects candidate commands, picks highest priority, applies to vehicle control
Actions.cs	Core behaviors: path following, traffic light handling, front-car handling, look-ahead selection
WaypointManager.cs / Scan.cs	Procedural road scanning + waypoint placement + sliding window refresh
SplineManager.cs	Catmull-Rom spline generation, curvature estimation, spline visualization
TrackingManager.cs	Queries “relevant objects in front” from the world model
WorldModel.cs	Central state store for cars/obstacles/traffic lights (game-friendly “network-dependent” ADS approach)
SimulationHandler.cs	Low-level actuation: throttle/brake/steer → WheelCollider physics
CntrlCmnd.cs	ControlCommand struct (throttle, brake, steering, priority)
Configuration (Inspector highlights)
Driving feel

Actions.desiredSpeed: cruising speed target

SplineManager.minLookAheadDistance / maxLookAheadDistance: steering stability vs responsiveness

SimulationHandler.topSpeed, maxMotorTorque, maxBrakeForce, maxSteerAngle: vehicle limits

Scanning & path density

WaypointManager.wpSpacing, scanRadius, rayCount: road detection quality vs cost

SplineManager.subdivisionsPerSegment: spline smoothness vs cost

Behavior arbitration

Priority outputs from behaviors determine which command is executed when multiple constraints apply (e.g., “red light braking” overrides “follow path”).

Evaluation summary (thesis)

An evaluation compared SDB against the BeamNG.drive built-in ADS on a ~1.9 km route using standardized indicators (time/average speed, traffic-light compliance, off-lane events).

Main result: SDB completed the route faster and with fewer lane departures than the BeamNG ADS in the reported experiment.

Reported metrics (thesis):

SDB: 233s, 28.7 km/h avg, 13 off-lane events

BeamNG ADS: 261s, 27.63 km/h avg, 18 off-lane events

(See thesis for methodology details and limitations around traffic light setup on the BeamNG route.)

Limitations

Designed for two-way roads without intersections (intersection/lane-graph support is listed as future work).

Driving smoothness can show steering oscillations and occasional unnecessary braking at times (tuning + control smoothing opportunities).

Future work

Intersection recognition & multi-lane reasoning

Event awareness (reacting to world incidents: crashes, hazards, etc.)

More “human-like” driving traits (patience, aggressiveness profiles, etc.)

Citation
BibTeX (thesis)
@thesis{kapitsakis2025ads,
  title  = {ADS in Video Games: Analysis, Development, and Evaluation},
  author = {Emmanouil Kapitsakis},
  year   = {2025},
  school = {SAE Creative Media College, Athens Campus}
}
