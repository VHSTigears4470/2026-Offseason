package frc.robot.sim;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.button.CommandGenericHID;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.RoboRioSim;

import com.ctre.phoenix6.sim.Pigeon2SimState;

import box3dsim.arena.Alliance;
import box3dsim.arena.Arena;
import box3dsim.arena.ArenaConfig;
import box3dsim.drive.SwerveDriveSim;
import box3dsim.math.Pose3;
import box3dsim.mechanism.FuelStorage;
import box3dsim.mechanism.IntakeSim;
import box3dsim.mechanism.ShooterSim;
import box3dsim.motor.LumpedMechanism;
import box3dsim.season.rebuilt2026.DsState;
import box3dsim.season.rebuilt2026.MatchPhase;
import box3dsim.season.rebuilt2026.RebuiltField;
import box3dsim.season.rebuilt2026.RebuiltFieldHandles;
import box3dsim.season.rebuilt2026.RebuiltFuelLayout;
import box3dsim.season.rebuilt2026.RebuiltMatch;

import frc.robot.RobotContainer;
import frc.robot.components.PIDMotor;
import frc.robot.components.SwerveModule;
import frc.robot.components.SwerveModuleIO;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Runs the robot against box3d-sim: each 20 ms loop is 5 physics substeps of REV controller iterate → motor volts →
 * arena step → true sensor values back to the Sparks, Pigeon and battery.
 */
public final class RobotSim {
    private static final int SUBSTEPS = 5;
    private static final double TWO_PI = 2 * Math.PI;
    private static final double RAD_PER_SEC_TO_RPM = 60 / TWO_PI;

    private final RobotContainer container;
    private final Arena arena;
    private final RebuiltMatch match;
    private final SwerveDriveSim drive;
    private final IntakeSim intake;
    private final FuelStorage storage;
    private final ShooterSim shooter;
    private final Pigeon2SimState pigeon;

    private final SparkBinding[] driveMotors = new SparkBinding[4];
    private final SparkBinding[] turnMotors = new SparkBinding[4];
    private final double[] turnOffsets = new double[4];
    private final SparkBinding flywheel;
    private final SparkBinding hood;
    private final SparkBinding feeder;
    private final SparkBinding indexer;
    private final SparkBinding intakeRoller;
    private final SparkBinding intakePivot;
    private final List<SparkBinding> all;

    private final LoggedNetworkBoolean resetFieldToggle = new LoggedNetworkBoolean("/Tuning/Sim/ResetField", false);
    private final LoggedNetworkBoolean releaseChuteToggle = new LoggedNetworkBoolean("/Tuning/Sim/ReleaseChute", false);
    private final LoggedNetworkBoolean refillChuteToggle = new LoggedNetworkBoolean("/Tuning/Sim/RefillChute", false);
    private final LoggedNetworkBoolean throwAtHubToggle = new LoggedNetworkBoolean("/Tuning/Sim/ThrowAtHub", false);
    private final LoggedNetworkBoolean freePlayToggle = new LoggedNetworkBoolean("/Tuning/Sim/FreePlay", false);

    /** Seconds FMS keeps the robot disabled between AUTO and TELEOP. */
    private static final double AUTO_TO_TELEOP_DELAY = 3;

    private final boolean actAsFms;
    private Alliance alliance = Alliance.BLUE;
    private boolean prevAutoEnabled;
    private MatchPhase lastPhase = MatchPhase.FREE_PLAY;
    private double teleopEnableAt = Double.NaN;
    private double stepMillis;

    /**
     * @param actAsFms disable the robot when AUTO and TELEOP end and start TELEOP after AUTO, as FMS would. The sim GUI
     *     mirrors HAL driver station state, so these changes show in the GUI and in AdvantageKit's logged DS state.
     */
    public RobotSim(RobotContainer container, boolean actAsFms) {
        this.container = container;
        this.actAsFms = actAsFms;
        arena = new Arena(ArenaConfig.defaults());
        RebuiltFieldHandles field = RebuiltField.build(arena.world());
        match = new RebuiltMatch(arena, field, RebuiltMatch.Config.defaults());
        alliance = currentAlliance();
        drive = new SwerveDriveSim(arena.world(), arena.battery(), SimConstants.swerve(), SimConstants.startPose(alliance));
        storage = new FuelStorage(arena.battery(), SimConstants.storage(), type -> shooter().launch(type));
        intake = new IntakeSim(arena, drive.body(), arena.battery(), SimConstants.intake(), storage);
        shooter = new ShooterSim(arena, drive, arena.battery(), SimConstants.shooter(),
                (piece, pose, v) -> match.onLaunch(alliance, piece, pose, SimConstants.BUMPER_HALF_LENGTH,
                        SimConstants.BUMPER_HALF_WIDTH));

        arena.addParticipant(drive);
        arena.addParticipant(intake);
        arena.addParticipant(storage);
        arena.addParticipant(shooter);
        arena.addParticipant(match);
        arena.addLoad(drive);
        arena.addLoad(intake);
        arena.addLoad(storage);
        arena.addLoad(shooter);

        DriveSubsystem driveSub = container.getDriveSubsystem();
        SwerveModule[] modules = driveSub.getModules();
        for (int i = 0; i < 4; i++) {
            SwerveModuleIO io = modules[i].getIO();
            driveMotors[i] = new SparkBinding(spark(io.getDriveMotor()), DCMotor.getNeoVortex(1),
                    SparkBinding.Feedback.RELATIVE, false);
            turnMotors[i] = new SparkBinding(spark(io.getTurnMotor()),
                    DCMotor.getNeo550(1).withReduction(SimConstants.STEER_GEARING), SparkBinding.Feedback.ABSOLUTE, false);
            turnOffsets[i] = io.getChassisAngularOffset();
        }
        flywheel = new SparkBinding(spark(container.getShooterSubsystem().getFlywheelMotor()), DCMotor.getNeoVortex(1),
                SparkBinding.Feedback.RELATIVE, true);
        hood = new SparkBinding(spark(container.getShooterSubsystem().getHoodMotor()), DCMotor.getNEO(1),
                SparkBinding.Feedback.RELATIVE, false);
        feeder = new SparkBinding(spark(container.getShooterSubsystem().getFeederMotor()), DCMotor.getNEO(1),
                SparkBinding.Feedback.RELATIVE, false);
        indexer = new SparkBinding(spark(container.getShooterSubsystem().getIndexerMotor()), DCMotor.getNEO(1),
                SparkBinding.Feedback.RELATIVE, true);
        intakeRoller = new SparkBinding(spark(container.getIntakeSubsystem().getIntakeMotor()), DCMotor.getNEO(1),
                SparkBinding.Feedback.RELATIVE, true);
        intakePivot = new SparkBinding(spark(container.getIntakeSubsystem().getDropMotor()), DCMotor.getNEO(1),
                SparkBinding.Feedback.RELATIVE, false);
        all = List.of(driveMotors[0], driveMotors[1], driveMotors[2], driveMotors[3], turnMotors[0], turnMotors[1],
                turnMotors[2], turnMotors[3], flywheel, hood, feeder, indexer, intakeRoller, intakePivot);

        pigeon = driveSub.getGyro().getSimState();
        driveSub.setPoseResetListener(
                p -> drive.teleport(p.getX(), p.getY(), p.getRotation().getRadians()));

        arena.reset();
        Pose2d start = PoseConversions.toPose2d(SimConstants.startPose(alliance));
        driveSub.resetOdometry(start);
        driveSub.resetPose(start);
        writeSensors();

        CommandGenericHID simOps = new CommandGenericHID(1);
        simOps.button(0).onTrue(new InstantCommand(this::resetField).ignoringDisable(true));
        simOps.button(1).onTrue(new InstantCommand(() -> match.releaseChute(alliance)).ignoringDisable(true));
        simOps.button(2).onTrue(new InstantCommand(() -> match.refillChute(alliance)).ignoringDisable(true));
        simOps.button(3).onTrue(new InstantCommand(() -> match.throwAtHub(alliance)).ignoringDisable(true));
    }

    private ShooterSim shooter() {
        return shooter;
    }

    private static com.revrobotics.spark.SparkBase spark(PIDMotor motor) {
        return motor.getIO().getSpark();
    }

    private static Alliance currentAlliance() {
        return MatchState.getAlliance()
                .map(a -> a == org.wpilib.driverstation.Alliance.RED ? Alliance.RED : Alliance.BLUE)
                .orElse(Alliance.BLUE);
    }

    SwerveDriveSim drive() {
        return drive;
    }

    RebuiltMatch match() {
        return match;
    }

    int shotsFired() {
        return shooter.shotsFired();
    }

    int intakeAcquired() {
        return intake.acquiredCount();
    }

    /** Places the simulated robot and seeds the robot's odometry to match. */
    void placeRobot(Pose3 pose) {
        drive.teleport(pose.position().x(), pose.position().y(), pose.rotation().yaw());
        Pose2d p = PoseConversions.toPose2d(pose);
        container.getDriveSubsystem().resetOdometry(p);
        container.getDriveSubsystem().resetPose(p);
    }

    /** Resets the field and returns null if every FUEL is back at its start, else a description. */
    String checkResetRestoresField() {
        resetField();
        return fieldAtStartProblem();
    }

    /** Null if every field FUEL is at its start position and the robot holds its preload, else a description. */
    String fieldAtStartProblem() {
        List<Pose3> poses = match.fuelDisplayPoses();
        List<box3dsim.math.Vec3> expected = new java.util.ArrayList<>(RebuiltFuelLayout.neutralZone());
        for (Alliance a : Alliance.values()) {
            expected.addAll(RebuiltFuelLayout.depot(a));
        }
        int matched = 0;
        for (box3dsim.math.Vec3 e : expected) {
            for (Pose3 p : poses) {
                if (p.position().minus(e).norm() < 1e-3) {
                    matched++;
                    break;
                }
            }
        }
        if (matched != expected.size()) {
            return matched + " of " + expected.size() + " field FUEL at start positions";
        }
        if (storage.totalCount() != 8) {
            return "robot holds " + storage.totalCount() + " FUEL, expected 8";
        }
        for (Alliance a : Alliance.values()) {
            if (match.chuteCount(a) != 24 || match.corralCount(a) != 0) {
                return a + " outpost holds " + match.chuteCount(a) + " chute / " + match.corralCount(a) + " corral";
            }
        }
        return null;
    }

    /** New match: field FUEL, scores and robot pose back to the start. */
    public void resetField() {
        arena.reset();
        Pose3 start = SimConstants.startPose(alliance);
        drive.teleport(start.position().x(), start.position().y(), start.rotation().yaw());
        Pose2d start2d = PoseConversions.toPose2d(start);
        container.getDriveSubsystem().resetOdometry(start2d);
        container.getDriveSubsystem().resetPose(start2d);
    }

    public void periodic() {
        alliance = currentAlliance();
        boolean autoEnabled = RobotState.isEnabled() && RobotState.isAutonomous();
        if (autoEnabled && !prevAutoEnabled && match.phase() == MatchPhase.POST_MATCH) {
            // A new match after a finished one starts on a fresh field. The robot isn't moved: the auto's own
            // odometry reset places it.
            arena.reset();
        }
        prevAutoEnabled = autoEnabled;
        match.setFreePlay(freePlayToggle.get());
        match.setDsState(new DsState(RobotState.isEnabled(), RobotState.isAutonomous(), RobotState.isTeleop()));
        boolean allowed = match.outputsAllowed();
        for (SparkBinding b : all) {
            b.setOutputsAllowed(allowed);
            b.detectRobotReset();
        }

        double dt = arena.config().physicsDt();
        long nanos = 0;
        for (int s = 0; s < SUBSTEPS; s++) {
            double vbus = arena.battery().voltage();
            applyControllerOutputs(vbus, dt);
            arena.step();
            nanos += arena.lastStepStats().totalNanos();
            writeSensors();
        }
        stepMillis = nanos / 1e6;
        followMatchPhase();

        double vbus = arena.battery().voltage();
        RoboRioSim.setVInVoltage(vbus);
        pigeon.setSupplyVoltage(vbus);
        pigeon.setRawYaw(Math.toDegrees(drive.gyro().yawRad()));
        pigeon.setAngularVelocityZ(Math.toDegrees(drive.gyro().yawRateRadPerSec()));

        handleTuningToggles();
        log();
    }

    /** Announces match transitions and, when acting as FMS, disables and enables the robot through the DS. */
    private void followMatchPhase() {
        MatchPhase phase = match.phase();
        double now = arena.time();
        if (phase != lastPhase) {
            if (phase == MatchPhase.AUTO_TO_TELEOP) {
                System.out.println("[box3d-sim] AUTO over: robot disabled"
                        + (actAsFms ? "; TELEOP starts in 3 s." : "."));
                teleopEnableAt = now + AUTO_TO_TELEOP_DELAY;
                setDsEnabled(false);
            } else if (phase == MatchPhase.POST_MATCH) {
                System.out.println("[box3d-sim] Match over: robot disabled. Enable Autonomous for a new match (the field"
                        + " resets), press Z in Teleoperated for teleop-only practice, or set /Tuning/Sim/FreePlay to"
                        + " drive without a match clock.");
                setDsEnabled(false);
            }
            lastPhase = phase;
        }
        if (actAsFms && phase == MatchPhase.AUTO_TO_TELEOP && now >= teleopEnableAt) {
            teleopEnableAt = Double.POSITIVE_INFINITY;
            DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();
        }
    }

    private void setDsEnabled(boolean enabled) {
        if (actAsFms) {
            DriverStationSim.setEnabled(enabled);
            DriverStationSim.notifyNewData();
        }
    }

    /** Step A + B: run each Spark's controller with the current mechanism speed and hand its volts to the model. */
    private void applyControllerOutputs(double vbus, double dt) {
        for (int i = 0; i < 4; i++) {
            double driveRpm = drive.wheelVelocityRadPerSec(i) * SimConstants.DRIVE_GEARING * RAD_PER_SEC_TO_RPM;
            double driveVolts = driveMotors[i].iterate(driveRpm, vbus, dt);

            // Keep the controller's position within half a turn of its setpoint, whether or not REV's sim wraps.
            double absolute = wrap01((drive.steerPositionRad(i) + turnOffsets[i]) / TWO_PI);
            double setpoint = turnMotors[i].setpoint();
            turnMotors[i].setControllerPosition(setpoint + wrapHalf(absolute - setpoint));
            double turnVolts = turnMotors[i].iterate(drive.steerVelocityRadPerSec(i) * RAD_PER_SEC_TO_RPM, vbus, dt);

            drive.setModuleInputs(i, driveVolts, driveMotors[i].coast(), turnVolts, turnMotors[i].coast());
        }
        shooter.setFlywheelInput(iterateMechanism(flywheel, shooter.flywheel(), vbus, dt), flywheel.coast());
        shooter.setHoodInput(iterateMechanism(hood, shooter.hood(), vbus, dt), hood.coast());
        storage.setFeederInput(iterateMechanism(feeder, storage.feeder(), vbus, dt), feeder.coast());
        storage.setIndexerInput(iterateMechanism(indexer, storage.indexer(), vbus, dt), indexer.coast());
        intake.setRollerInput(iterateMechanism(intakeRoller, intake.roller(), vbus, dt), intakeRoller.coast());
        intake.setPivotInput(iterateMechanism(intakePivot, intake.pivot(), vbus, dt), intakePivot.coast());
    }

    private static double iterateMechanism(SparkBinding b, LumpedMechanism m, double vbus, double dt) {
        return b.iterate(m.rotorVelocity() * RAD_PER_SEC_TO_RPM, vbus, dt);
    }

    /** Step D: true sensor values back to the Sparks. */
    private void writeSensors() {
        for (int i = 0; i < 4; i++) {
            double g = SimConstants.DRIVE_GEARING;
            driveMotors[i].writeRelative(drive.wheelPositionRad(i) * g / TWO_PI,
                    drive.wheelVelocityRadPerSec(i) * g * RAD_PER_SEC_TO_RPM);
            double steer = drive.steerPositionRad(i);
            double steerRpm = drive.steerVelocityRadPerSec(i) * RAD_PER_SEC_TO_RPM;
            turnMotors[i].writeAbsolute(wrap01((steer + turnOffsets[i]) / TWO_PI), steerRpm);
            turnMotors[i].writeRelative(steer * SimConstants.STEER_GEARING / TWO_PI, steerRpm * SimConstants.STEER_GEARING);
        }
        writeMechanism(flywheel, shooter.flywheel());
        writeMechanism(hood, shooter.hood());
        writeMechanism(feeder, storage.feeder());
        writeMechanism(indexer, storage.indexer());
        writeMechanism(intakeRoller, intake.roller());
        writeMechanism(intakePivot, intake.pivot());
    }

    private static void writeMechanism(SparkBinding b, LumpedMechanism m) {
        b.writeRelative(m.rotorPosition() / TWO_PI, m.rotorVelocity() * RAD_PER_SEC_TO_RPM);
    }

    private void handleTuningToggles() {
        if (resetFieldToggle.get()) {
            resetFieldToggle.set(false);
            resetField();
        }
        if (releaseChuteToggle.get()) {
            releaseChuteToggle.set(false);
            match.releaseChute(alliance);
        }
        if (refillChuteToggle.get()) {
            refillChuteToggle.set(false);
            match.refillChute(alliance);
        }
        if (throwAtHubToggle.get()) {
            throwAtHubToggle.set(false);
            match.throwAtHub(alliance);
        }
    }

    private void log() {
        Pose3 pose = drive.pose();
        Logger.recordOutput("FieldSimulation/RobotPose", PoseConversions.toPose3d(pose));
        Logger.recordOutput("FieldSimulation/RobotPose2d", PoseConversions.toPose2d(pose));
        Logger.recordOutput("FieldSimulation/Fuel",
                match.fuelDisplayPoses().stream().map(PoseConversions::toPose3d).toArray(Pose3d[]::new));
        double hoodPitch = shooter.launchPitch();
        Logger.recordOutput("FieldSimulation/Components",
                new Pose3d(new Translation3d(-0.35, 0, 0.20), new Rotation3d(0, intake.pivot().position(), 0)),
                new Pose3d(new Translation3d(0.10, 0, 0.55),
                        new Rotation3d(0, hoodPitch - Math.toRadians(SimConstants.LAUNCH_PITCH_DEG), 0)));

        Logger.recordOutput("FieldSimulation/Match/Phase", match.phase().name());
        Logger.recordOutput("FieldSimulation/Match/MatchTime", match.matchTimeRemaining());
        Logger.recordOutput("FieldSimulation/Match/PhaseTime", match.phaseTimeRemaining());
        Logger.recordOutput("FieldSimulation/Match/OutputsAllowed", match.outputsAllowed());
        Logger.recordOutput("FieldSimulation/Match/BlueScore", match.score(Alliance.BLUE));
        Logger.recordOutput("FieldSimulation/Match/RedScore", match.score(Alliance.RED));
        Logger.recordOutput("FieldSimulation/Match/BlueHubActive", match.hubActive(Alliance.BLUE));
        Logger.recordOutput("FieldSimulation/Match/RedHubActive", match.hubActive(Alliance.RED));
        Logger.recordOutput("FieldSimulation/Match/FirstInactive",
                match.firstInactive().map(Enum::name).orElse(""));
        Logger.recordOutput("FieldSimulation/Match/BlueFouls", match.fouls(Alliance.BLUE));
        Logger.recordOutput("FieldSimulation/Match/RedFouls", match.fouls(Alliance.RED));
        Logger.recordOutput("FieldSimulation/Match/LastFoul", match.lastFoul());
        Logger.recordOutput("FieldSimulation/Match/FreePlay", match.freePlay());
        Logger.recordOutput("FieldSimulation/Match/BlueFreePlayFuel", match.freePlayFuel(Alliance.BLUE));
        Logger.recordOutput("FieldSimulation/Match/RedFreePlayFuel", match.freePlayFuel(Alliance.RED));

        Logger.recordOutput("FieldSimulation/Outpost/BlueChute", match.chuteCount(Alliance.BLUE));
        Logger.recordOutput("FieldSimulation/Outpost/BlueCorral", match.corralCount(Alliance.BLUE));
        Logger.recordOutput("FieldSimulation/Outpost/RedChute", match.chuteCount(Alliance.RED));
        Logger.recordOutput("FieldSimulation/Outpost/RedCorral", match.corralCount(Alliance.RED));

        Logger.recordOutput("FieldSimulation/Robot/Hopper", storage.hopperCount());
        Logger.recordOutput("FieldSimulation/Robot/Staged", storage.staged());
        Logger.recordOutput("FieldSimulation/Robot/ShotsFired", shooter.shotsFired());
        Logger.recordOutput("FieldSimulation/Robot/IntakeAcquired", intake.acquiredCount());
        Logger.recordOutput("FieldSimulation/Robot/IntakeDeployProgress", intake.deployProgress());
        // Negated to match the robot's own Shooter/Actual RPM, which logs -flywheelMotor.getVelocity().
        Logger.recordOutput("FieldSimulation/Robot/FlywheelRPM", -shooter.flywheel().velocity() * RAD_PER_SEC_TO_RPM);
        Logger.recordOutput("FieldSimulation/Robot/LaunchPitchDeg", Math.toDegrees(hoodPitch));

        Logger.recordOutput("FieldSimulation/Battery/Volts", arena.battery().voltage());
        Logger.recordOutput("FieldSimulation/Battery/Amps", arena.battery().filteredCurrent());
        Logger.recordOutput("FieldSimulation/Battery/BrownedOut", arena.battery().isBrownedOut());
        Logger.recordOutput("FieldSimulation/Perf/StepMs", stepMillis);
    }

    private static double wrap01(double rotations) {
        double r = rotations - Math.floor(rotations);
        return r >= 1 ? 0 : r;
    }

    private static double wrapHalf(double rotations) {
        double r = rotations - Math.round(rotations);
        return r <= -0.5 ? r + 1 : r;
    }
}
