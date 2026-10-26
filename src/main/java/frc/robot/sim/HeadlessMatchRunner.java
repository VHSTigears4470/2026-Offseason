package frc.robot.sim;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.simulation.DriverStationSim;

import box3dsim.arena.Alliance;
import box3dsim.math.Pose3;
import box3dsim.season.rebuilt2026.MatchPhase;
import box3dsim.season.rebuilt2026.RebuiltField;

import frc.robot.RobotContainer;

/**
 * Plays a scripted match in FAST (lockstep) mode with no GUI or human, then exits 0 (pass), 1 (fail) or 2
 * (watchdog) and writes build/sim-results.json. Options come from SIM_AUTO, SIM_MATCH and SIM_EXPECT_MIN_SCORE.
 */
public final class HeadlessMatchRunner {
    private static final double LOOP = 0.02;
    private static final double ENABLE_AT = 0.5;
    private static final double AUTO_END = ENABLE_AT + 20;
    private static final double TELEOP_AT = AUTO_END + 3;
    private static final double TELEOP_END = TELEOP_AT + 140;
    /** Full mode then starts a second auto, which must begin on a freshly reset field. */
    private static final double SECOND_AUTO_AT = TELEOP_END + 4;
    private static final double FULL_END = SECOND_AUTO_AT + 0.5;
    private static final double HUMAN_THROWS_FROM = TELEOP_AT + 1;
    private static final int HUMAN_THROWS = 5;
    /** Full mode backs the robot's intake side against the neutral-zone FUEL with the roller intaking. */
    private static final double INTAKE_CHECK_AT = TELEOP_AT + 5;
    private static final double INTAKE_CHECK_END = INTAKE_CHECK_AT + 2;
    private static final int INTAKE_MIN_TAKEN = 4;
    private static final double MAX_SIM_SECONDS = 200;
    private static final long MAX_WALL_MILLIS = 10 * 60 * 1000;
    private static final double MAX_ODOMETRY_ERROR = 0.30;
    private static final double HEADING_LAG_WARN_DEG = 2;

    private final RobotSim sim;
    private final RobotContainer container;
    private final String auto;
    private final boolean fullMatch;
    private final int expectMinScore;
    private final long wallStart = System.nanoTime();

    private int loops;
    private double odometryError = Double.NaN;
    private double maxHeadingLagDeg;
    private double maxSpeed;
    private Pose3 autoEndTruth;
    private Pose2d autoEndOdometry;
    private int humanThrows;
    private boolean postMatchReached;
    private boolean intakeStarted;
    private int intakeTaken = -1;
    private boolean autoEnabled;
    private boolean secondAutoSelected;
    private boolean secondAutoEnabled;
    private boolean checkedAutoEndDisable;
    private boolean checkedTeleopEnable;
    private boolean checkedMatchEndDisable;
    private boolean fmsDisabledAtAutoEnd;
    private boolean fmsEnabledTeleop;
    private boolean fmsDisabledAtMatchEnd;

    public HeadlessMatchRunner(RobotSim sim, RobotContainer container) {
        this.sim = sim;
        this.container = container;
        String a = System.getenv("SIM_AUTO");
        this.auto = a == null ? "" : a.trim();
        this.fullMatch = "full".equalsIgnoreCase(String.valueOf(System.getenv("SIM_MATCH")).trim());
        String min = System.getenv("SIM_EXPECT_MIN_SCORE");
        this.expectMinScore = min == null || min.isBlank() ? 0 : Integer.parseInt(min.trim());

        Thread watchdog = new Thread(() -> {
            try {
                Thread.sleep(MAX_WALL_MILLIS);
            } catch (InterruptedException e) {
                return;
            }
            System.out.println("SIM RESULT: FAIL watchdog: more than 10 minutes of wall time");
            System.exit(2);
        }, "sim-watchdog");
        watchdog.setDaemon(true);
        watchdog.start();
    }

    public void periodic() {
        double t = loops * LOOP;
        loops++;
        script(t);
        DriverStationSim.notifyNewData();
        measure(t);

        double end = fullMatch ? FULL_END : TELEOP_AT;
        if (t >= end) {
            finish(t, 0);
        } else if (t > MAX_SIM_SECONDS) {
            finish(t, 2);
        }
    }

    private void script(double t) {
        // Only the start of each match is scripted. RobotSim acts as FMS for the rest: it disables the robot when
        // AUTO ends, enables TELEOP 3 s later and disables it at the end; the checks below verify that.
        if (loops == 1) {
            DriverStationSim.setDsAttached(true);
            DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
            DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
            DriverStationSim.setEnabled(false);
        }
        DriverStationSim.setMatchTime(sim.match().matchTimeRemaining());
        if (!autoEnabled && t >= ENABLE_AT) {
            DriverStationSim.setEnabled(true);
            autoEnabled = true;
        }
        if (!checkedAutoEndDisable && t >= AUTO_END + 0.5) {
            fmsDisabledAtAutoEnd = !RobotState.isEnabled();
            checkedAutoEndDisable = true;
        }
        if (fullMatch && !checkedTeleopEnable && t >= TELEOP_AT + 1) {
            fmsEnabledTeleop = RobotState.isTeleopEnabled();
            checkedTeleopEnable = true;
        }
        if (fullMatch && !checkedMatchEndDisable && t >= TELEOP_END + 1) {
            fmsDisabledAtMatchEnd = !RobotState.isEnabled();
            checkedMatchEndDisable = true;
        }
        if (fullMatch && t >= TELEOP_AT && t < TELEOP_END) {
            // Human-player throws scatter FUEL and drain the CHUTE, so the second match's reset has work to do.
            if (humanThrows < HUMAN_THROWS && t >= HUMAN_THROWS_FROM + humanThrows * 0.5) {
                sim.match().throwAtHub(Alliance.BLUE);
                humanThrows++;
            }
            if (!intakeStarted && t >= INTAKE_CHECK_AT) {
                // Back bumper (the intake side) just touching the first neutral-zone column; yaw π faces it +x.
                double x = RebuiltField.LENGTH / 2 - 0.8382 - 0.075 - SimConstants.BUMPER_HALF_LENGTH - 0.005;
                sim.placeRobot(Pose3.ofXYYaw(x, RebuiltField.WIDTH / 2, 0, Math.PI));
                container.getIntakeSubsystem().setIntake(-0.9);
                intakeStarted = true;
            } else if (intakeStarted && intakeTaken < 0 && t >= INTAKE_CHECK_END) {
                intakeTaken = sim.intakeAcquired();
                container.getIntakeSubsystem().setIntake(0);
                // Clear the neutral zone: the next match's field reset respawns FUEL where the robot is parked.
                sim.placeRobot(SimConstants.startPose(Alliance.BLUE));
            }
        } else if (fullMatch && t >= TELEOP_END && t < SECOND_AUTO_AT) {
            if (t >= TELEOP_END + 3 && sim.match().phase() == MatchPhase.POST_MATCH) {
                postMatchReached = true;
            }
        } else if (fullMatch && t >= SECOND_AUTO_AT) {
            if (!secondAutoSelected) {
                DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
                secondAutoSelected = true;
            } else if (!secondAutoEnabled && t >= SECOND_AUTO_AT + 0.1) {
                DriverStationSim.setEnabled(true);
                secondAutoEnabled = true;
            }
        }
    }

    private void measure(double t) {
        Pose3 truth = sim.drive().pose();
        double robotYaw = container.getDriveSubsystem().getRotation2d().getRadians();
        double lag = Math.toDegrees(Math.abs(Math.IEEEremainder(robotYaw - sim.drive().gyro().yawRad(), 2 * Math.PI)));
        maxHeadingLagDeg = Math.max(maxHeadingLagDeg, lag);
        if (Double.isNaN(odometryError) && t >= AUTO_END) {
            Pose2d odom = container.getDriveSubsystem().getOdometry();
            odometryError = Math.hypot(odom.getX() - truth.position().x(), odom.getY() - truth.position().y());
            autoEndTruth = truth;
            autoEndOdometry = odom;
        }
        maxSpeed = Math.max(maxSpeed, Math.hypot(sim.drive().fieldVelocity().x(), sim.drive().fieldVelocity().y()));
    }

    private void finish(double t, int watchdogCode) {
        var match = sim.match();
        int blue = match.score(Alliance.BLUE);
        int red = match.score(Alliance.RED);
        MatchPhase phase = match.phase();
        MatchPhase expectedPhase = fullMatch ? MatchPhase.AUTO : MatchPhase.AUTO_TO_TELEOP;
        List<String> failures = new ArrayList<>();
        if (watchdogCode != 0) failures.add("watchdog: sim time exceeded " + MAX_SIM_SECONDS + " s");
        if (!fmsDisabledAtAutoEnd) failures.add("robot still enabled after AUTO ended");
        if (fullMatch && !fmsEnabledTeleop) failures.add("TELEOP was not enabled after AUTO");
        if (fullMatch && !fmsDisabledAtMatchEnd) failures.add("robot still enabled after the match ended");
        if (fullMatch) {
            if (!postMatchReached) failures.add("first match never reached POST_MATCH");
            if (intakeTaken < INTAKE_MIN_TAKEN) {
                failures.add("intake took " + intakeTaken + " FUEL backed against the neutral zone, expected >= "
                        + INTAKE_MIN_TAKEN);
            }
            String fresh = sim.fieldAtStartProblem();
            if (fresh != null) failures.add("second auto did not start on a fresh field: " + fresh);
        }
        if (blue < expectMinScore) failures.add("blue score " + blue + " < expected " + expectMinScore);
        if (phase != expectedPhase) failures.add("phase " + phase + " != " + expectedPhase);
        if (Double.isNaN(odometryError) || odometryError > MAX_ODOMETRY_ERROR) {
            failures.add(String.format(Locale.ROOT, "odometry error %.3f m > %.2f m", odometryError, MAX_ODOMETRY_ERROR));
        }
        String resetProblem = sim.checkResetRestoresField();
        if (resetProblem != null) {
            failures.add("reset field: " + resetProblem);
        }
        double wall = (System.nanoTime() - wallStart) / 1e9;
        double rtf = t / Math.max(wall, 1e-9);
        boolean pass = failures.isEmpty();
        if (maxHeadingLagDeg > HEADING_LAG_WARN_DEG) {
            System.out.printf(Locale.ROOT, "SIM WARNING: max heading lag %.2f deg%n", maxHeadingLagDeg);
        }

        String json = String.format(Locale.ROOT, """
                {
                  "result": "%s",
                  "mode": "%s",
                  "auto": "%s",
                  "blueScore": %d, "redScore": %d, "expectMinScore": %d,
                  "phase": "%s",
                  "odometryErrorM": %.3f,
                  "maxHeadingLagDeg": %.2f,
                  "simSeconds": %.2f, "wallSeconds": %.2f, "realTimeFactor": %.2f,
                  "shotsFired": %d,
                  "intakeTaken": %d,
                  "maxSpeedMps": %.2f,
                  "autoEndTruth": [%.3f, %.3f, %.1f],
                  "autoEndOdometry": [%.3f, %.3f, %.1f],
                  "fouls": {"blue": %d, "red": %d},
                  "failures": [%s]
                }
                """, pass ? "PASS" : "FAIL", fullMatch ? "full" : "auto", auto.replace("\"", "'"), blue, red,
                expectMinScore, phase, odometryError, maxHeadingLagDeg, t, wall, rtf, sim.shotsFired(), intakeTaken,
                maxSpeed,
                autoEndTruth == null ? Double.NaN : autoEndTruth.position().x(),
                autoEndTruth == null ? Double.NaN : autoEndTruth.position().y(),
                autoEndTruth == null ? Double.NaN : Math.toDegrees(autoEndTruth.rotation().yaw()),
                autoEndOdometry == null ? Double.NaN : autoEndOdometry.getX(),
                autoEndOdometry == null ? Double.NaN : autoEndOdometry.getY(),
                autoEndOdometry == null ? Double.NaN : autoEndOdometry.getRotation().getDegrees(),
                match.fouls(Alliance.BLUE), match.fouls(Alliance.RED),
                String.join(", ", failures.stream().map(f -> "\"" + f.replace("\"", "'") + "\"").toList()));
        try {
            Files.createDirectories(Path.of("build"));
            Files.writeString(Path.of("build", "sim-results.json"), json);
        } catch (IOException e) {
            failures.add("could not write build/sim-results.json: " + e.getMessage());
        }

        if (pass) {
            System.out.printf(Locale.ROOT, "SIM RESULT: PASS score=%d odomErr=%.2fm rtf=%.1fx%n", blue, odometryError, rtf);
        } else {
            System.out.println("SIM RESULT: FAIL " + String.join("; ", failures));
        }
        Logger.end();
        System.exit(watchdogCode != 0 ? watchdogCode : pass ? 0 : 1);
    }
}
