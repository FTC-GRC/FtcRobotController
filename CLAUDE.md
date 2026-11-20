# Team context for Claude

FTC robot code for the 2026-2027 season, **BIOBUZZ** (manual version TU10). Team repo: `FTC-GRC/FtcRobotController`.
All team code is in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

The person working with you is a student on the team, not the main coder. Explain things plainly, keep answers short,
and flag anything risky before a competition. Time before events is usually very short: prefer simple, safe changes
over clever ones.

## Branches
- `Attila`: the team's working branch. Don't change it directly.
- `fix/field-centric-drive`: this branch. It has the fixes and the new auto described below. It hasn't been merged
  into `Attila` yet and **hasn't been tested on the robot** at the time of writing.

## Build setup (Windows)
- Android Gradle Plugin 8.13.2 and Gradle 9.1.0, so the build **needs JDK 17+**. Use Android Studio's bundled JBR
  (JDK 21): Settings > Build Tools > Gradle > Gradle JDK = the bundled `jbr` / `GRADLE_LOCAL_JAVA_HOME`.
  Don't use a non-LTS JDK such as Java 20.
- Command line: set `JAVA_HOME` to `C:\Program Files\Android\Android Studio\jbr`, then run
  `./gradlew :TeamCode:assembleDebug`.
- `local.properties` (with `sdk.dir`) is gitignored; Android Studio creates it.
- If Android Studio offers to upgrade Gradle or the Android Gradle plugin, **decline**.
- FTC SDK 12.0, so the Driver Station app should also be 12.0.
- Do the first build with internet, because Gradle downloads its dependencies then.
- Clone into a short path. Long Windows paths break git and Gradle.

## Robot hardware (robot configuration names must match exactly)
| Name | Device | Notes |
|---|---|---|
| `LF` `RF` `LR` `RR` | drive motors (mecanum) | left FORWARD, right REVERSE, BRAKE, RUN_USING_ENCODER |
| `DE` | goBILDA Pinpoint | offsets (-84, -168) mm, goBILDA 4-bar pods, both encoders FORWARD |
| `IN` | intake motor | REVERSE |
| `RT` | transfer motor | REVERSE |
| `PS` | shooter motor (DcMotorEx) | REVERSE, own PIDF loop (kF 0.00051, kP 0.0005), **3300 RPM**, 28 CPR |
| `PR` | ring rotator CRServo | REVERSE |
| `PE` | through-bore encoder on a motor port | 8192 CPR, gear ratio 2.0 |
| `PH` | pollen holder servo | HOLD 0.0 / RELEASE 0.5 |

Robot size: about 45 x 45 cm (17.7 in). The robot will have odometry pods.

## Code overview
- `BetterWithoutLL.java`: **TeleOp "BWLL"**, the main teleop. Classes in the same file: `Control` (all button
  mapping), `Drive`, `Intake`, `PollenShooter`, `PollenHolder`, `Telemetry`.
  - `Drive.USE_PEDRO = false` (default): the hand-written mecanum drive that has actually run on the robot.
    `true` switches to Pedro teleop, which is untested.
  - `Drive.ENABLE_DRIVE_TO_TARGET = false`: the Y-button Pedro path. Keep it off until Pedro is tuned.
  - Share/Back (PS4 Share, PS5 Create, Xbox View) toggles field-relative / robot-relative. A resets the field heading.
  - `HEADING_SIGN`: set to -1 if field-relative feels mirrored.
  - gamepad2: R1 selects a shooter gain, D-pad up/down changes it (live PIDF tuning).
- `SimpleAuto.java`: **Auto "Simple Shoot + Park"** (preselects BWLL). No Pedro. Uses Pinpoint odometry.
  - Shooting: shooter on, wait for speed (0.5–2.5 s), then transfer and intake together for `FEED_TIME_S` = 5 s.
  - Then drives waypoints in robot coordinates (inches; X = forward, Y = left): (21, 0) LEAVE, (21, 29),
    (99, 29), (99, 46) PARK.
  - Safety aborts: odometry not moving, moving away from the target, or heading off by more than 30°.
    Timeout per move scales with distance.
  - Other settings: `SKIP_PARK` (leave only), `HOLD_HEADING`, `START_DELAY_S`, `MAX_POWER` 0.5.
  - INIT telemetry shows Pinpoint status, X/Y and heading for the pod check: push forward -> X up,
    push left -> Y up, turn left -> heading up.
- `PedroAuto.java`: Pedro example auto, `@Disabled` until Pedro is tuned.
- `pedroPathing/Constants.java`: Pedro 2.1.2 setup (motors, Pinpoint, brake in teleop). Mass and velocities are
  **untuned placeholders**.
- `Better.java`: older teleop with Limelight. Not the one in use.

## Field geometry used for the auto (BIOBUZZ, from manual figures, +/- 1 in)
Field is 144 x 144 in. Origin is the audience-left corner; x runs toward the blue wall, y away from the audience.
Red alliance wall is x = 0; red side is columns A–C (x < 72).
- HIVE frame base: x 47.3–96.7, y 52.5–91.5. HIVE centers 25.5 in apart: red hive x = 59.25, blue = 84.75.
- Each HIVE is 42.9 in long along y; cells are 12 in deep with openings of 20 x 14 in, tilted 30°, 53.5–65.6 in
  above the tiles.
- At match start, the **red** upward-facing CELL is the audience-side one; it opens toward the audience wall.
  For **blue** it's the far-side one. The field is symmetric under a 180° rotation, so the auto works unchanged
  for both alliances.
- FLOWERS (on the walls): (0, 48), (48, 144), (144, 96), (96, 0).
- Red LOADING ZONE: x 0–11, y 96–119 (against the red wall). Blue: x 133–144, y 24–48.
- Auto start: robot's back against the wall the upward CELL faces, facing the HIVE, with the robot's left side
  2.4 in (6 cm) from the tile seam on its left (red: B/C seam, blue: D/E seam). The robot is then centered on our
  HIVE at about 40 in horizontal distance to the CELL opening.

## Relevant rules (TU10)
- AUTO is 30 s and the 30 s timer must be on (G305). LEAVE (3 pts): the robot no longer touches the perimeter
  wall. AUTO PARK (5 pts): at least partly in our LOADING ZONE. HIVE TIP is 20 pts. SWARM RP: combined
  LEAVE + PARK at or above a threshold, so both alliance robots should leave and park.
- G304 start: fully on our half, touching the wall, not in the LOADING ZONE, not touching a FLOWER,
  exactly 4 POLLEN pre-loads, motionless after INIT.
- G402: no disrupting the opponent in AUTO, and crossing to their half is risky (MAJOR FOUL).
- G403: no powered movement between AUTO and TELEOP. If teleop INIT makes anything twitch, wait for TELEOP to start.
- G417: only launching into the upward CELL may move the HIVE. Launching at a cell while the HIVE is tipping can
  ruin the TIP.

## Open problems / next steps
1. **Field-relative drive behaves exactly like robot-relative**, meaning the heading never changes. The math is
   correct (it's the FTC sample). Likely causes: the Pinpoint isn't mounted flat, or it isn't connected properly
   (I2C cable, port, configured as "goBILDA Pinpoint Odometry Computer" named `DE`). Diagnose by turning the
   robot by hand and watching "Raw heading"; a quarter turn left should read about +90.
2. Test SimpleAuto on the robot: the pod check, the shot from the start position (does 3300 RPM land from about
   40 in?), whether 5 s of feeding is enough, and the waypoint distances.
3. Coordinate with the alliance partner. Our path runs 15–33 in from the alliance wall, and we park in the
   audience half of the LOADING ZONE.
4. Later, with time: tune Pedro (mass, xVelocity/yVelocity, PIDFs), then consider `USE_PEDRO = true` and a
   Pedro auto.
5. After testing on the robot, open a PR from `fix/field-centric-drive` into `Attila`.
