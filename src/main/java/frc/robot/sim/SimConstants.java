package frc.robot.sim;

import java.util.ArrayList;
import java.util.List;

import box3dsim.arena.Alliance;
import box3dsim.drive.GyroConfig;
import box3dsim.drive.SwerveDriveConfig;
import box3dsim.drive.SwerveModuleConfig;
import box3dsim.drive.TireConfig;
import box3dsim.math.Pose3;
import box3dsim.math.Vec3;
import box3dsim.mechanism.IntakeConfig;
import box3dsim.mechanism.RollerConfig;
import box3dsim.mechanism.ShooterConfig;
import box3dsim.mechanism.StorageConfig;
import box3dsim.motor.DcMotor;
import box3dsim.motor.LumpedMechanism;
import box3dsim.season.rebuilt2026.RebuiltField;
import box3dsim.season.rebuilt2026.RebuiltFuel;

import frc.robot.Constants.Drive;

/** Physical model of the 2026-Offseason robot. ASSUMPTION marks values the code doesn't pin down. */
final class SimConstants {
    private SimConstants() {}

    /** Matches DRIVING_FACTOR, so simulated encoders convert exactly like the real ones. */
    static final double DRIVE_GEARING = 3.625;
    static final double STEER_GEARING = 9424.0 / 203.0;
    static final double INDEXER_GEARING = 5;
    static final double FEEDER_GEARING = 3;
    static final double INTAKE_ROLLER_GEARING = 3;
    /** ASSUMPTION: −19 motor rotations = 105° of intake travel. */
    static final double PIVOT_GEARING = 65.14;
    static final double PIVOT_DEPLOYED_MOTOR_ROTATIONS = -19;
    static final double BUMPER_HALF_LENGTH = 0.45;
    static final double BUMPER_HALF_WIDTH = 0.45;
    static final double INTAKE_HALF_WIDTH = 0.265;
    /** Hood zip-tied at 0 (as raced): the flywheel/top-roller pinch exit angle, from CAD. */
    static final double LAUNCH_PITCH_DEG = 78;

    private static double motorRotationsToRad(double rotations, double gearing) {
        return rotations * 2 * Math.PI / gearing;
    }

    static SwerveDriveConfig swerve() {
        double half = Drive.Constants.WHEEL_BASE / 2;
        double[][] positions = {{half, half}, {half, -half}, {-half, half}, {-half, -half}};
        List<SwerveModuleConfig> modules = new ArrayList<>();
        for (double[] p : positions) {
            // ASSUMPTION: wheel MOI and friction torques.
            modules.add(new SwerveModuleConfig(p[0], p[1], DcMotor.neoVortex(1), DRIVE_GEARING,
                    Drive.ModuleConstants.WHEEL_DIAMETER / 2, 0.002, 0.15,
                    DcMotor.neo550(1), STEER_GEARING, 0.004, 0.1));
        }
        // Mass and MOI from PathPlanner settings.json. Robot height 0.55 m: the raced robot was filed down to fit the
        // 0.565 m TRENCH clearance. ASSUMPTION: cg height.
        return new SwerveDriveConfig(modules, 50, 6.883, 0.20, 2 * BUMPER_HALF_LENGTH, 2 * BUMPER_HALF_WIDTH,
                0.0635, 0.0825, 0.08, 0.55, TireConfig.defaults(), GyroConfig.pigeon2(), true);
    }

    static ShooterConfig shooter() {
        // CAD: Vortex belted 1:1 to two SDS 4 in flywheels (0.0023 kg·m²; rollers and shafts add an estimated
        // 0.0007). A 2 in top roller geared 1:1 counter-rotates, so backspin is (4 − 2)/(4 + 2) of the exit speed.
        // Exit ratio 0.55 (no-slip would be 0.75) is calibrated so the hub preset sinks from the auto end point.
        LumpedMechanism.Config flywheel =
                LumpedMechanism.Config.of(DcMotor.neoVortex(1), 1, 0.003).withFriction(0.05, 1e-4);
        // Hood belt cut at comp: the NEO spins unloaded and the launch pitch is fixed.
        LumpedMechanism.Config hood = LumpedMechanism.Config.of(DcMotor.neo(1), 1, 1e-4).withFriction(0.02, 0);
        // Exit point: the pinch (x 0.17, z 0.51 in CAD) moved 0.13 m along the launch line to clear the chassis box.
        return new ShooterConfig(flywheel, 0.0508, -1, 0.55, 0.33, 0.7, 2.0, hood,
                p -> Math.toRadians(LAUNCH_PITCH_DEG),
                new Vec3(0.195, 0, 0.64), 0, RebuiltFuel.TYPE);
    }

    static StorageConfig storage() {
        // CAD: NEO rollers; the feeder is 3:1 into 3 in rollers. ASSUMPTION: 40 FUEL capacity. Negative volts feed
        // toward the shooter.
        RollerConfig indexer = new RollerConfig(
                LumpedMechanism.Config.of(DcMotor.neo(1), INDEXER_GEARING, 5e-4), 0.025, -1, 0.8, 0.3);
        RollerConfig feeder = new RollerConfig(
                LumpedMechanism.Config.of(DcMotor.neo(1), FEEDER_GEARING, 3e-4), 0.0381, -1, 0.8, 0.5);
        return new StorageConfig(40, 8, indexer, feeder, RebuiltFuel.TYPE);
    }

    /**
     * The intake is on the back. FUEL touching the back bumper is taken while the roller spins in the intake
     * direction (negative output, as IntakeRun's −0.9); the pivot's deploy state doesn't matter.
     */
    static IntakeConfig intake() {
        // CAD: NEO with a 3:1 planetary into 2 in flex wheels. ASSUMPTION: pivot gearing.
        RollerConfig roller = new RollerConfig(
                LumpedMechanism.Config.of(DcMotor.neo(1), INTAKE_ROLLER_GEARING, 5e-4), 0.0254, -1, 0.7, 0.8);
        LumpedMechanism.Config pivot = LumpedMechanism.Config.of(DcMotor.neo(1), PIVOT_GEARING, 0.15)
                .withFriction(0.5, 0)
                .withLimits(motorRotationsToRad(-19.5, PIVOT_GEARING), motorRotationsToRad(0.3, PIVOT_GEARING));
        // Zone: 0.2 m deep behind the back bumper face (x = −0.45), the 0.53 m roller span from CAD, FUEL height.
        return new IntakeConfig(roller, pivot, motorRotationsToRad(PIVOT_DEPLOYED_MOTOR_ROTATIONS, PIVOT_GEARING), 0,
                0, new Vec3(-(BUMPER_HALF_LENGTH + 0.10), 0, 0.075), new Vec3(0.10, INTAKE_HALF_WIDTH, 0.10),
                RebuiltFuel.TYPE);
    }

    /** The Backup Center path start, mirrored for red. */
    static Pose3 startPose(Alliance alliance) {
        Pose3 blue = Pose3.ofXYYaw(3.5, 4.0545, 0, 0);
        if (alliance == Alliance.BLUE) {
            return blue;
        }
        Vec3 p = RebuiltField.mirror(blue.position());
        return Pose3.ofXYYaw(p.x(), p.y(), 0, Math.PI);
    }
}
