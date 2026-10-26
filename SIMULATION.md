# Simulation

This robot runs in desktop simulation against box3d-sim (`F:\Projects\box3d-sim`), a 3D physics sim of the
2026 REBUILT field. FUEL are real rigid bodies, every Spark runs REV's own simulated firmware, and the Pigeon 2 and
battery are driven by the physics. Sim code lives in `src/main/java/frc/robot/sim/`. No other robot code imports the
library.

## Prerequisites

- WPILib 2027 alpha 7, installed for all users (`C:\Users\Public\wpilib\2027_alpha7`). It provides JDK 25 and
  AdvantageScope with the 2026 field.
- From the command line, set `$env:JAVA_HOME = 'C:\Users\Public\wpilib\2027_alpha7\jdk'` first. The WPILib VS Code
  sets it for you.
- box3d-sim published locally. Run `.\gradlew.bat publishToMavenLocal` in `F:\Projects\box3d-sim` after any
  library change.

## Modes

| Command | Mode |
|---|---|
| `.\gradlew.bat run` (or VS Code → WPILib: Simulate Robot Code) | interactive: real time, sim GUI |
| `.\gradlew.bat run -PsimMode=fast -PsimAuto="Shoot at Hub"` | headless lockstep: plays the auto and exits 0/1 with `build/sim-results.json` |
| `.\gradlew.bat run -PsimMode=fast -PsimMatch=full` | headless full match: auto, then teleop with a few human-player throws and an intake check (back bumper against the neutral zone, roller intaking). It checks that the sim disabled and enabled the robot like FMS, and that a second auto starts on a freshly reset field |
| `.\gradlew.bat run -PsimMode=replay` | AdvantageKit log replay |

`pwsh scripts/sim-ci.ps1` publishes the library and runs both headless cases.

In fast mode the console shows "Watchdog not fed" warnings, because HAL time is stepped inside the loop. They are
expected.

## Playing a match solo

1. **Sim GUI setup.** The WPILib Simulation GUI opens on its own when you run `.\gradlew.bat run` (interactive mode
   only). It's one window holding several sub-windows; any you don't see can be reopened from its menu bar.
   - **System Joysticks** lists your USB controllers plus Keyboard 0–2. Drag the gamepad from there onto slot 0 of
     the **Joysticks** window.
   - Drag **Keyboard 0** onto slot 1 for the sim controls below. Its key mapping is in the **Keyboard 0 Settings**
     window (hidden by default).
   - In the **DS** window, set **Alliance Station** to **Blue 1** *before* picking a mode. The GUI defaults to
     Red 1, and PathPlanner flips paths for red. Once a mode is selected the DS counts as attached and that window
     goes read-only.
   - **Robot State** has the Disabled / Autonomous / Teleoperated choices and Enable / Disable.
   - The GUI remembers these assignments in `simgui-ds.json` and `simgui.json` in the project folder.
2. **AdvantageScope.**
   1. **File → Connect to Simulator**.
   2. Add a 3D Field tab with the **2026 Field**.
   3. Drag `RealOutputs/FieldSimulation/RobotPose` in as the robot (model: 2026 KitBot).
   4. Drag `RealOutputs/FieldSimulation/Fuel` in as **Game Piece → Fuel**.
   5. Optionally drag `RealOutputs/Drive/Pose` in as a Ghost.
   6. Save the layout with **File → Export Layout…** to `advantagescope/box3d-sim-layout.json` and commit it.
3. **Auto.**
   - Pick the auto **before enabling**, because the robot reads it in `autonomousInit`. It's a 2027 `Selectable`
     published at `/Tunables/Auto Path` in NetworkTables. In the sim GUI, open it from the menu bar:
     **NetworkTables → Tunables → Auto Path**, then choose **Shoot at Hub** in the dropdown. The default is `None`
     (no auto). AdvantageScope can't set it, because its tuning mode edits only numbers and booleans.
   - Select **Autonomous** and **Enable**. From there the sim acts as FMS:
     - after 20 s of AUTO it disables the robot;
     - 3 s later it switches Robot State to **Teleoperated** and enables it;
     - after 2:20 of TELEOP it disables the robot.
   - These changes show in the sim GUI's Robot State window and in AdvantageKit's `DriverStation/*` values. The
     console prints a `[box3d-sim]` line at each transition.
   - The sim's match clock is `FieldSimulation/Match/MatchTime`. The GUI keeps its own DS match time, which doesn't
     follow the sim's clock.
4. **Teleop.**
   - Nothing to click; it starts by itself after AUTO. Clicking Teleoperated → Enable sooner also works.
   - The match runs TRANSITION (10 s), SHIFT 1–4 (25 s each, active HUBs alternating) and END GAME (30 s).
   - FUEL scored into an inactive HUB is worth 0. Watch `FieldSimulation/Match/BlueHubActive`.
   - **G407:** launching FUEL **into your own HUB** while your bumpers are fully outside your ALLIANCE ZONE is a
     MAJOR foul (15 to the opponent). Launching from midfield to pass FUEL back to your side is legal and isn't
     penalized.
   - **Free play:** set `/Tuning/Sim/FreePlay` to true in AdvantageScope's tuning mode to drop the match clock
     entirely. The robot is never disabled by the sim, and HUB FUEL is counted without points in
     `FieldSimulation/Match/BlueFreePlayFuel` and `RedFreePlayFuel`.
     Set it back to false, and the next enable starts a timed match.
5. **Human player.** Use the Keyboard 0 joystick on slot 1 (buttons are 0-based; the GUI's Keyboard 0 settings
   show the keys), or toggle the `/Tuning/Sim/*` booleans in AdvantageScope's tuning mode.

   | Key (Keyboard 0 default) | Button | Tuning toggle | Action |
   |---|---|---|---|
   | Z | 0 | `ResetField` | reset the field, scores and robot pose (works while disabled) |
   | X | 1 | `ReleaseChute` | open the CHUTE: stored FUEL rolls onto the field |
   | C | 2 | `RefillChute` | move CORRAL FUEL into the CHUTE |
   | V | 3 | `ThrowAtHub` | the human player throws one FUEL at the HUB (about half land) |
   | — | — | `FreePlay` | on: no match clock, the robot is never disabled by the sim (stays on until turned off) |

   Keys reach the robot only while the sim GUI window has focus. Keyboard 0 also maps W/A/S/D and E/R to axes,
   but nothing on port 1 reads them.

   **Keys and the gamepad do nothing while Robot State is set to Autonomous**, enabled or not. The sim GUI stops
   sending all joystick data in that mode, just as a real DS ignores drivers during auto. Press the keys with
   **Teleoperated** (or Disabled) selected. The AdvantageScope `/Tuning/Sim/*` toggles go over NetworkTables and
   work in any mode.

6. **Next match:**
   - Select **Autonomous** and **Enable** again; that's all.
   - Once a match has ended (POST_MATCH), enabling Autonomous resets the field automatically: FUEL, scores,
     outposts and the robot's 8-FUEL preload.
   - The robot isn't moved; the auto's own odometry reset places it.
   - To also put the robot back at the start (for example before a teleop-only practice match), press **Z** with
     Teleoperated selected, or use the `ResetField` toggle.

   Enabling teleop without auto starts a teleop-only practice match.

Gamepad controls are the robot's own.

**Intake:** the intake is on the robot's **back**.
- FUEL touching the back bumper is pulled in while the roller spins in the intake direction (negative output,
  as X's −0.9).
- Whether the intake is deployed doesn't matter in the sim.
- A (deploy) also sets the roller to +0.2, which is outward and doesn't intake. So after A, the first X press stops
  the roller and the second starts it intaking.
- `FieldSimulation/Robot/Hopper` and `IntakeAcquired` show the count.

## What's assumed

Mechanism geometry comes from the team's Onshape CAD (`Offseason Rebuilt 2026 4470`, MAIN assembly):

- shooter: one Vortex belted 1:1 to two SDS 4 in flywheels, with a 2 in top roller geared 1:1 so the FUEL is
  pinched between them;
- hood: zip-tied at 0, as raced. The FUEL leaves at a fixed 78°, and hood commands only spin the unloaded NEO;
- all other mechanism motors are NEOs. The feeder is 3:1 into 3 in rollers, and the intake roller is 3:1 into 2 in
  flex wheels;
- the intake on the back is 0.53 m wide;
- the robot is 0.55 m tall and fits under the TRENCH. The CAD's top is 0.583 m, but the raced robot was filed down
  to fit.

The exit speed ratio, 0.55 of the flywheel's surface speed, is calibrated so the hub preset sinks from the end of
`Shoot at Hub`, as it did on the real robot. The hub preset scores from 1.0–2.0 m and the side preset from
1.5–2.5 m.

Values still marked ASSUMPTION in `src/main/java/frc/robot/sim/SimConstants.java`:

- hopper capacity (40);
- intake pivot gearing;
- cg height.

After changing shooter values, rerun box3d-sim's `ShotCalibrationTest` (its constants mirror `SimConstants`).

## Known v0 limitations

- The robot is locked flat. BUMPs don't affect robots, while FUEL rolls over them.
- The intake has no physical roller collider, and storage is counted rather than simulated.
- The HUB is a square bin with spawn-point exits. No opponents, no climbing, no vision.
- REV sim adds unseeded velocity noise, so runs aren't bit-for-bit repeatable.
