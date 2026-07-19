Vision support is under active development. There may be bugs or unfinished features.

# Limelight Lib Vendordep

https://limelightvision.github.io/limelightlib-public/LimelightLib.json

# Limelight 2027.0 Beta Camera Images

https://drive.google.com/drive/folders/1NwZB1yKtuCkDpp5WSTrInyImtRlClsnv?usp=sharing

# Limelight 2027.0 Beta Migration Guide

Breaking changes TLDR:

1. All results are sent via a single messagepack topic in NetworkTables. The original NT API can be enabled per-pipeline in the pipeline output tab.

2. A single robot-orientation update (posted to a new limelightshared NT table) can feed every camera using MegaTag2.

3. LimelightLib 2 introduces an object-oriented API with built-in pose filtering and trust scaling. Telemetry for accepted and rejected pose estimates is enabled by default, allowing those estimates and related statistics to be visualized in AdvantageScope, Elastic, and Glass.

## Quick start

### Visual servoing

```java
double turnKp = -0.02;
Limelight camera = new Limelight();

// Each loop
double turn = joystick.getRightX();
if(aimingEnabled && camera.hasTarget()){
     turn = turnKp * camera.getTXDegrees();
}
arcadeDrive(forward, turn); // pseudocode (todo)
```

### MegaTag1 localization

```java
Pose3d cameraPoseRobotSpace = new Pose3d(0.30, 0.0, 0.20, new Rotation3d());
Limelight camera = new Limelight("limelight", cameraPoseRobotSpace);
// ^ Limelight instances now have default filtering and trust scaling configurations that serve as reasonable starting points.

// Each robot loop, drain the entire PoseEstimate Queue. See rejected poses in AdvantageScope/Elastic with automatic telemetry
for (var estimate : camera.readAcceptedPoseEstimates(Limelight.PoseEstimateType.MT1_WPIBLUE)) {
    poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
}
```

### MegaTag2 localization

Publish robot yaw before reading the queue each robot loop.

```java
Pose3d cameraPoseRobotSpace = new Pose3d(0.30, 0.0, 0.20, new Rotation3d());
Limelight camera = new Limelight("limelight", cameraPoseRobotSpace);

// Each robot loop, drain the entire PoseEstimate Queue. See rejected poses in AdvantageScope/Elastic with automatic telemetry
Limelight.setSharedRobotOrientation(robotYawDegrees);
for (var estimate : camera.readAcceptedPoseEstimates(Limelight.PoseEstimateType.MT2_WPIBLUE)) {
    poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
}
```

### Configured MegaTag1 and MegaTag2

This example configures every filter and standard-deviation scaling term.
XY uncertainty scales linearly with distance, divides by the square root of
fielded tag count, and is clamped to 0.0001-2.0 meters. Vision heading is not
fused. The field bounds are for the 2026 welded field. Tune the other thresholds
on your robot.

```java
double untrusted = Limelight.PoseEstimateConfig.UNTRUSTED;
Limelight.PoseEstimateConfig mt1Config = Limelight.PoseEstimateConfig.defaultMT1() // starting with the safe default means you are not required to set every single configuration value
        .withMinTagCount(1)
        .withMaxSingleTagAmbiguity(0.7) // MT1 needs low-ambiguity perspectives
        .withMaxSingleTagDistance(4.0) // If we only see one tag, don't trust it unless we are at most 4m away from it
        .withMaxAvgTagDistance(6.0) // If we see multiple tags, max avg distance must be less than 6 meters.
        .withMinAvgTagArea(0.02) // 0-100, percentage of image area. This is .02%, not 2%.
        .withFieldBounds(16.541, 8.069) // Reject pose estimates that are out of bounds.
        .withFieldBoundsMargin(0.5) // Add .5m margin around field.
        .withStdDevXY(0.5, 0.05, 2.0) // .5 base, absolute min .05, absolute max 2.0. With scaling enabled, .5 at 1m distance
        .withStdDevTheta(untrusted, untrusted, untrusted) // never incorporate pose estimate rotation. You may want to incorporate rotation by setting these to other values
        .withStdDevDistanceScaling(1.0, 0.0, 6.0) // linear scaling, only scale between 0 and 6m.
        .withStdDevTagCountDivision(0.5) // Enhance trust by square root of number of contributing tags
        .withTelemetry(true); // Keep automatic telemetry enabled. All accepted and rejected poses will remain easy to visualize in popular dashboards

Limelight.PoseEstimateConfig mt2Config = Limelight.PoseEstimateConfig.defaultMT2()
        .withMinTagCount(1)
        .withMaxSingleTagAmbiguity(1.0) // MT2 can handle maximally ambiguous perspectives. Accept tags regardless of ambiguity value.
        .withMaxSingleTagDistance(0.0) // 0 disables this check
        .withMaxAvgTagDistance(8.0)
        .withMinAvgTagArea(0.02) //0-100, percentage of image area
        .withFieldBounds(16.541, 8.069)
        .withFieldBoundsMargin(0.5)
        .withStdDevXY(0.3, 0.0001, 2.0)
        .withStdDevTheta(untrusted, untrusted, untrusted)
        .withStdDevDistanceScaling(0.5, 0.0, 8.0) // Less aggressive STDDev scaling for MT2. Scale by sqrt(distance) rather than distance^1.
        .withStdDevTagCountDivision(0.5) // Enhance trust by a factor equal to the square root of number of contributing tags
        .withTelemetry(true);

Pose3d cameraPoseRobotSpace = new Pose3d(0.30, 0.0, 0.20, new Rotation3d());
Limelight camera = new Limelight("limelight", cameraPoseRobotSpace)
        .withPoseEstimateConfig_MT1(mt1Config)
        .withPoseEstimateConfig_MT2(mt2Config);
boolean useMegaTag2 = true;

// Each robot loop
Limelight.PoseEstimateType type = useMegaTag2 ? Limelight.PoseEstimateType.MT2_WPIBLUE : Limelight.PoseEstimateType.MT1_WPIBLUE;
if (useMegaTag2) {
    Limelight.setSharedRobotOrientation(robotYawDegrees); // Use your robot pose yaw here, not your raw IMU reading.
}
for (var estimate : camera.readAcceptedPoseEstimates(type)) {
    poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
}
```

4. 2027.0 unifies every 3D coordinate system into the **NWU, right-handed** (X forward, Y left, Z up) standard.

**3D Robot Space**: X forward, Y left, Z up.
**3D Camera* Space**: X forward (boresight), Y left, Z up.
**3D Tag / target**: X out of the tag face, Y tag-left, Z up.


2D targeting features still use the Limelight Optical space, in which the camera looks down -Z. This matches other 3D graphics convetions and the standard 2D Cartesian coordinate system (Center Image Pixel is (0,0), +Y Up, +X Right).

This release still has the same three field localization systems: native centered, wpiblue, and wpired. Future releases will adapt to a new unified centered coordinate system. botpose, botpose_wpiblue, and botpose_wpired will work as they did in 2026 releases.

5. Limelight 3G, 3A, and 4 now have OTA update capabilities. You can test OTA by first using the newest hardware manager to update to 2027.0. You can then OTA update (still to 2027.0) using the .llupdate files in the folder.


## Checklist

1. Update firmware to 2027.0 and LimelightHelpers to the 2027 release.
2. Re-enter camera mount side and pitch on every pipeline (flip both signs).
3. Flip the third component of every POI (pipelines and old `.fmap` files).
4. Update any code using targetspace, cameraspace transforms to work with the new coordinate system,
5. If you want to use the classic NetworkTables keys, re-enable them in the
   Output tab (per pipeline).
6. Verify visually: the web UI's 3D views render exactly what NetworkTables
   publishes, so if the robot, camera, and tags look right in the visualizers, you're good to go.


# 2027.0 Changelog

## Limelight OS 2027.0 (IN PROGRESS CHANGE LOG)

### Significantly Improved Update UX

1. Limelight OS can now be updated over the air (OTA) through the web UI with .llupdate files (LL3A, LL3G, LL4 only). No hardware manager, no USB, no flash mode. Power on, drop the update into the web UI, and you're done. You will need to use the very latest version of the hardware manager one time to get your camera on 2027.0 or later.
Complete pipelines, including python scripts, neural networks, and
apriltag maps, can now be downloaded/uploaded as `.llpipeline` files. All 10 pipelines can be downloaded/uploaded as a single `.llpipelinepack` file.
2. Recovery image files and the new .llupdate OTA update packages have been optimized for fast downloads and minimal disk utilization.
3. The filesystem now expands on first boot to provide around 4 additional gigabytes of storage for rewind recordings.
4. Every downloaded file now includes the camera name and hardware type name. This makes rewind and pipeline organization easier than before.
5. The hardware manager, while no longer necessary for most teams after 2027.0, is now cross-platform.

### LimelightLib 2

LimelightLib 2 introduces an object-oriented API with built-in pose filtering and trust scaling. Telemetry for accepted and rejected pose estimates is enabled by default, allowing those estimates and related statistics to be visualized in AdvantageScope, Elastic, and Glass.

### IMU Reliability

Known IMU fault conditions have been eliminated.

### Unified 3D Coordinate Systems (BREAKING)

* Limelight now uses the NWU right-hand convention everywhere. It follows the right-hand 
rule, with X forward, Y left, and Z up.
* Robotspace and Points of Interest now follow NWU as well, so pipelines and code
 need updated cameraPoseInRobotSpace side, pitch, and Point of Interest values.
* See the 2027.0 Migration Guide below

### Atomic MessagePack Results (BREAKING)

* All results are published to NT as a single messagepack topic. This guarantees atomicity for all results data from a camera.
* Results now include camera intrinsics and a `customcal` flag indicating whether
the camera is using a user calibration.
* The classic NetworkTables results API is disabled by default. It can be
enabled again per pipeline in the Output tab.

### Shared Robot Orientation

* All Limelights now read `robotOrientation_set` from the new
`limelightshared` table by default. A single robot orientation update can feed
every MegaTag2 camera.
* Robot code can opt individual Limelights out of the shared orientation and
publish directly to those cameras instead.

### Full Resolution Capture Endpoint

The new `/capture` HTTP endpoint returns the latest uncompressed frame as a PNG
at the camera's full resolution.

### MAVLink Output

Limelight vision instances can now publish MAVLink for various drone platforms.

### WebRTC Updates

* WebRTC retransmission capabilities are now disabled.
* Focus pipelines now use an H.264 profile that prioritizes higher sharpness and clarity while HQ
Mode is enabled. The stream runs at native resolution at approximately 5 FPS
with a 2.5 Mbps cap.

### AprilTag and Pipeline Updates

* AprilTag pipelines now default to 3D mode.
* Target area is consistently reported on a 0 to 100 scale.
* The UI indicates when Camera Pose in Robot Space or fiducial downscaling is
  controlled by robot code. Those values are shown directly and the
  corresponding controls are locked.

### Calibration and Distortion Diagnostics

Improve distortion visual in web UI

### Live Diagnostics Search

The web UI now includes a search bar within the live JSON results view.

### Systemcore Support

2027.0 runs on Systemcore image 12 and supports USB cameras. 18 cameras have been manually curated and benefit from hardcoded workarounds for various USB firmware issues. Non-curated cameras will still function and auto-populate controls. All curated cameras expose one or two video modes selected for utility on FIRST robots. Non-curated cameras will expose exactly one video mode that is automatically selected. By default, USB cameras are configured for auto exposure and auto white balance.

Global-shutter Vision Cameras
1. Arducam OV9281 USB
2. Arducam OV9782 / goBILDA Global Shutter
3. Arducam OV2311 USB
4. Innomaker OV9281 USB
5. Waveshare OV9281 USB - not recommended due to poor optics, warning displayed in UI
6. ThriftyBot ThriftyCam
7. ThriftyBot ThriftiestCam (supported in the next update)

Rolling Shutter Vision Cameras
8. UC60 / goBILDA Rolling Shutter

Logitech webcams
9. Logitech C270 (960p variant)
10. Logi C270 (720p variant) — no manual exposure, not recommended, warning displayed in UI.
11. Logitech C310
12. Logitech C920 Family
13. Logitech Brio 101
14. Logitech Brio 100
15. Logitech C922 Pro Stream
16. Logitech 1080P Pro Stream
17. Logitech C930

Other
18. Microsoft LifeCam HD-3000
19. Sony PS3 Eyecam