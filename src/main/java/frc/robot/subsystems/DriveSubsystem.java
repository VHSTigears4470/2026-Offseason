package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import org.wpilib.system.Timer;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.limelightvision.Limelight;
import com.limelightvision.Limelight.PoseEstimateType;

// import choreo.trajectory.SwerveSample;
// import org.wpilib.hardware.hal.FRCNetComm.tInstances;
// import org.wpilib.hardware.hal.FRCNetComm.tResourceType;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.Units;
import org.wpilib.system.RobotController;
import org.wpilib.command2.SubsystemBase;
import frc.robot.components.SwerveModule;
import frc.robot.components.SwerveModuleIONEO;
import frc.robot.Robot;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Configs;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.Constants.MotorLocation;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Operating;

public class DriveSubsystem extends SubsystemBase {
    private SwerveModule frontLeft = null;
    private SwerveModule frontRight = null;
    private SwerveModule backLeft = null;  
    private SwerveModule backRight = null;
    
    private SwerveModuleVelocity desiredStates[] = {new SwerveModuleVelocity(), new SwerveModuleVelocity(), new SwerveModuleVelocity(), new SwerveModuleVelocity()};

    private final Pigeon2 gyro = Operating.Constants.USING_GYRO ? new Pigeon2(IDs.DriveConstants.PIGEON_ID, CAN.Constants.PigeonCAN) : null;
    SwerveDriveOdometry odometry = null;
    SwerveDrivePoseEstimator poseEstimator = null;
    private double lastMatchLog = 0.0;
    private boolean lastTeleopEnabled = false;
    private boolean lastAutonomousEnabled = false;

    //No clue about these values - for Choreo
    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    private final Limelight limelight;

    public DriveSubsystem() {
            frontLeft = new SwerveModule(
                new SwerveModuleIONEO(
                    IDs.CANBUSConstants.DRIVE_CANBUS_ID,
                    IDs.CANBUSConstants.DRIVE_CANBUS_ID,
                    IDs.DriveConstants.FL_DRIVE_ID,
                    IDs.DriveConstants.FL_TURN_ID,
                    Drive.Constants.FL_ANGULAR_OFFSET,
                    Configs.SwerveModule.FL_CONFIG,
                    Configs.SwerveModule.TURNING_CONFIG),
                MotorLocation.FRONT_LEFT);
            frontRight = new SwerveModule(new SwerveModuleIONEO(
                    IDs.CANBUSConstants.DRIVE_CANBUS_ID,
                    IDs.CANBUSConstants.DRIVE_CANBUS_ID,
                    IDs.DriveConstants.FR_DRIVE_ID,
                    IDs.DriveConstants.FR_TURN_ID,
                    Drive.Constants.FR_ANGULAR_OFFSET,
                    Configs.SwerveModule.FR_CONFIG,
                    Configs.SwerveModule.TURNING_CONFIG),
                MotorLocation.FRONT_RIGHT);
            backLeft = new SwerveModule(new SwerveModuleIONEO(
                    IDs.CANBUSConstants.DRIVE_CANBUS_ID,
                    IDs.CANBUSConstants.DRIVE_CANBUS_ID,
                    IDs.DriveConstants.BL_DRIVE_ID,
                    IDs.DriveConstants.BL_TURN_ID,
                    Drive.Constants.BL_ANGULAR_OFFSET,
                    Configs.SwerveModule.BL_CONFIG,
                    Configs.SwerveModule.TURNING_CONFIG),
                MotorLocation.BACK_LEFT);
            backRight = new SwerveModule(new SwerveModuleIONEO(
                    IDs.CANBUSConstants.DRIVE_CANBUS_ID,
                    IDs.CANBUSConstants.DRIVE_CANBUS_ID,
                    IDs.DriveConstants.BR_DRIVE_ID,
                    IDs.DriveConstants.BR_TURN_ID,
                    Drive.Constants.BR_ANGULAR_OFFSET,
                    Configs.SwerveModule.BR_CONFIG,
                    Configs.SwerveModule.TURNING_CONFIG),
                 MotorLocation.BACK_RIGHT);

        odometry = new SwerveDriveOdometry(
            Drive.Constants.DRIVE_KINEMATICS,
            getRotation2d(),
            getSwerveModulePositions());

        poseEstimator = new SwerveDrivePoseEstimator(Drive.Constants.DRIVE_KINEMATICS, getRotation2d(), getSwerveModulePositions(), new Pose2d(), 
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        poseEstimator = new SwerveDrivePoseEstimator(Drive.Constants.DRIVE_KINEMATICS,
            getRotation2d(), 
            getSwerveModulePositions(), 
            new Pose2d()
        ); // todo: standard deviations? idrk how to do ill ask nathan
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        // TODO: FIX     
        // HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

        limelight = new Limelight("limelight-one");
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, String statusName) {
        double multiplier = 0.5; 
        double xSpeedDelivered = xSpeed * Drive.Constants.MAX_METERS_PER_SECOND * multiplier;
        double ySpeedDelivered = ySpeed * Drive.Constants.MAX_METERS_PER_SECOND * multiplier;
        double rotDelivered = rot * Drive.Constants.MAX_ANGULAR_SPEED * multiplier;

        ChassisVelocities chassisVelocities = new ChassisVelocities(xSpeedDelivered, ySpeedDelivered, rotDelivered);
        if(fieldRelative) {
			chassisVelocities = chassisVelocities.toRobotRelative(getRotation2d());
		}

        SwerveModuleVelocity[] SwerveModuleVelocities = Drive.Constants.DRIVE_KINEMATICS.toSwerveModuleVelocities(chassisVelocities);
        SwerveModuleVelocities = SwerveDriveKinematics.desaturateWheelVelocities(
            SwerveModuleVelocities, Drive.Constants.MAX_METERS_PER_SECOND);
        desiredStates = SwerveModuleVelocities;

        frontLeft.setDesiredVelocity(SwerveModuleVelocities[0]);
        frontRight.setDesiredVelocity(SwerveModuleVelocities[1]);
        backLeft.setDesiredVelocity(SwerveModuleVelocities[2]);
        backRight.setDesiredVelocity(SwerveModuleVelocities[3]);
    }

    // public void followTrajectory(SwerveSample sample) {
    //     Pose2d pose = getOdometry(); //getEstimatedPosition();
    //
    //     ChassisVelocities speeds = new ChassisVelocities(
    //         sample.vx + xController.calculate(pose.getX(), sample.x),
    //         sample.vy + yController.calculate(pose.getY(), sample.y),
    //         sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
    //     );
    //
    //     driveFieldRelative(speeds);
    // }
    //
    public void driveFieldRelative(ChassisVelocities fieldRelativeVelocities) {
        ChassisVelocities relativeSpeeds = fieldRelativeVelocities.toRobotRelative(getRotation2d());
        driveRobotRelative(relativeSpeeds);
    }

    public void driveRobotRelative(ChassisVelocities relativeVelocities) {
        ChassisVelocities targetSpeeds = relativeVelocities.discretize(0.02);
        SwerveModuleVelocity[] targetStates = Drive.Constants.DRIVE_KINEMATICS.toSwerveModuleVelocities(targetSpeeds);
        frontLeft.setDesiredVelocity(targetStates[0]);
        frontRight.setDesiredVelocity(targetStates[1]);
        backLeft.setDesiredVelocity(targetStates[2]);
        backRight.setDesiredVelocity(targetStates[3]);
    }

    public SwerveModuleVelocity[] getSwerveModuleVelocities() {
        return new SwerveModuleVelocity[] {
            frontLeft.getVelocity(),
            frontRight.getVelocity(),
            backLeft.getVelocity(),
            backRight.getVelocity()
        };
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public Rotation2d getRotation2d() {
        if(Operating.Constants.USING_GYRO) { 
            return new Rotation2d(gyro.getYaw().getValue()); //Verify; might have to invert
        }
        else {
            return new Rotation2d(0);
        }
    }

    public Pose2d getOdometry() {
        return odometry.getPose();
    }

    public Pose2d getEstimatedPosition() { 
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisVelocities getRobotRelativeSVelocities() {
        return Drive.Constants.DRIVE_KINEMATICS.toChassisVelocities(
            frontLeft.getVelocity(),
            frontRight.getVelocity(),
            backLeft.getVelocity(),
            backRight.getVelocity()
        );
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
            getRotation2d(),
            getSwerveModulePositions(),
            pose);
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            getRotation2d(),
            getSwerveModulePositions(),
            pose);
    }

    public void updateOdometry(Pose2d pose) {
        poseEstimator.update(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
    }

    public void resetEncoders() {
        frontLeft.resetDriveEncoder();
        frontRight.resetDriveEncoder();
        backLeft.resetDriveEncoder();
        backRight.resetDriveEncoder();
    }

    public void zeroHeading() {
        if(Operating.Constants.USING_GYRO) gyro.setYaw(0);
    }

    public void stopModules() {
        frontLeft.stopMotors();
        frontRight.stopMotors();
        backLeft.stopMotors();
        backRight.stopMotors();

        for(int i = 0; i < desiredStates.length; i++)
            desiredStates[i].velocity = 0;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Drive/Pose", poseEstimator.getEstimatedPosition());
        Logger.recordOutput("Drive/LimelightPose", limelight.getBotPose3d(PoseEstimateType.MT2_WPIBLUE));
        Logger.recordOutput("Drive/Pose/X", poseEstimator.getEstimatedPosition().getX());
        Logger.recordOutput("Drive/Pose/Y", poseEstimator.getEstimatedPosition().getY());
        Logger.recordOutput("Drive/Pose/Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        
        if (Operating.Constants.USING_GYRO) {
            Logger.recordOutput("Drive/Gyro/Yaw", gyro.getYaw().getValue());
            Logger.recordOutput("Drive/Gyro/Pitch", gyro.getPitch().getValue());
            Logger.recordOutput("Drive/Gyro/Roll", gyro.getRoll().getValue());
        }

        Logger.recordOutput("Drive/ModuleStates/Desired", desiredStates);
        Logger.recordOutput("Drive/ModuleStates/Actual", getSwerveModuleVelocities());

        Logger.recordOutput("Power/BatteryVoltage", RobotController.getBatteryVoltage());

        double now = Timer.getMonotonicTimestamp();
        if (now - lastMatchLog > 0.2) {
            lastMatchLog = now;
            Logger.recordOutput("Match/TimeRemaining", Timer.getMatchTime());
        }

        boolean teleop = Robot.isTeleopEnabled();
        boolean auton = Robot.isAutonomousEnabled();
        if (teleop != lastTeleopEnabled || auton != lastAutonomousEnabled) {
            lastTeleopEnabled = teleop;
            lastAutonomousEnabled = auton;
            Logger.recordOutput("Match/TeleopEnabled", teleop);
            Logger.recordOutput("Match/AutonomousEnabled", auton);
            String mode = auton ? "Autonomous" : teleop ? "Teleop" : "Disabled";
            Logger.recordOutput("Match/Mode", mode);
        }

        odometry.update(getRotation2d(), getSwerveModulePositions());
        poseEstimator.updateWithTime(Timer.getMonotonicTimestamp(), getRotation2d(), getSwerveModulePositions());

        //rename "limelight"
        if(Operating.Constants.USING_LIMELIGHT) {
            boolean useMegaTag2 = true; //set to false to use MegaTag1
            boolean doRejectUpdate = false;
            if(useMegaTag2 == false) {
                Limelight.PoseEstimate mt1 = limelight.getPoseEstimate(PoseEstimateType.MT1_WPIBLUE);
                if(mt1.reportedTagCount == 1 && mt1.rawFiducials.length == 1) {
                    if(mt1.rawFiducials[0].ambiguity > .7) {
                        doRejectUpdate = true;
                    }   
                    if(mt1.rawFiducials[0].getDistanceToCamera() > 3) {
                        doRejectUpdate = true;
                    }
                }
                if(mt1.reportedTagCount == 0) {
                    doRejectUpdate = true;
                }
                if(!doRejectUpdate) {
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
                    poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
                }
            } else if (useMegaTag2 == true) {
                limelight.setRobotOrientation(poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                Limelight.PoseEstimate mt2 = limelight.getPoseEstimate(PoseEstimateType.MT2_WPIBLUE);
                if(Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) {
                    // if our angular velocity is greater than 720 degrees per second, ignore vision updates
                    doRejectUpdate = true;
                }
                if(mt2.reportedTagCount == 0) {
                    doRejectUpdate = true;
                }
                if(!doRejectUpdate) {
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                    poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
                }
            }
        }
    }
}
