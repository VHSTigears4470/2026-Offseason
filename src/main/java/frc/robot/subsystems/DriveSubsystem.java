package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.hardware.Pigeon2;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.SwerveModule;
import frc.robot.components.SwerveModuleIONEO;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Configs;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.Constants.MotorLocation;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Operating;

public class DriveSubsystem extends SubsystemBase{
    private SwerveModule frontLeft = null;
    private SwerveModule frontRight = null;
    private SwerveModule backLeft = null;  
    private SwerveModule backRight = null;
    
    private SwerveModuleState desiredStates[] = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    private final Pigeon2 gyro = Operating.Constants.USING_GYRO ? new Pigeon2(IDs.DriveConstants.PIGEON_ID) : null;
    SwerveDriveOdometry odometry = null;
    SwerveDrivePoseEstimator poseEstimator = null;
    private double lastMatchLog = 0.0;
    private boolean lastTeleopEnabled = false;
    private boolean lastAutonomousEnabled = false;

    //No clue about these values - for Choreo
    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    public DriveSubsystem() {
            frontLeft = new SwerveModule(
                new SwerveModuleIONEO(
                    IDs.DriveConstants.FL_DRIVE_ID,
                    IDs.DriveConstants.FL_TURN_ID,
                    Drive.Constants.FL_ANGULAR_OFFSET,
                    Configs.SwerveModule.FL_CONFIG,
                    Configs.SwerveModule.TURNING_CONFIG),
                MotorLocation.FRONT_LEFT);
            frontRight = new SwerveModule(new SwerveModuleIONEO(
                    IDs.DriveConstants.FR_DRIVE_ID,
                    IDs.DriveConstants.FR_TURN_ID,
                    Drive.Constants.FR_ANGULAR_OFFSET,
                    Configs.SwerveModule.FR_CONFIG,
                    Configs.SwerveModule.TURNING_CONFIG),
                MotorLocation.FRONT_RIGHT);
            backLeft = new SwerveModule(new SwerveModuleIONEO(
                    IDs.DriveConstants.BL_DRIVE_ID,
                    IDs.DriveConstants.BL_TURN_ID,
                    Drive.Constants.BL_ANGULAR_OFFSET,
                    Configs.SwerveModule.BL_CONFIG,
                    Configs.SwerveModule.TURNING_CONFIG),
                MotorLocation.BACK_LEFT);
            backRight = new SwerveModule(new SwerveModuleIONEO(
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
        
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, String statusName) {
        double multiplier = 0.5; 
        double xSpeedDelivered = xSpeed * Drive.Constants.MAX_METERS_PER_SECOND * multiplier;
        double ySpeedDelivered = ySpeed * Drive.Constants.MAX_METERS_PER_SECOND * multiplier;
        double rotDelivered = rot * Drive.Constants.MAX_ANGULAR_SPEED * multiplier;

        SwerveModuleState[] swerveModuleStates = Drive.Constants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, Drive.Constants.MAX_METERS_PER_SECOND);
        desiredStates = swerveModuleStates;

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getEstimatedPosition(); 

        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        driveFieldRelative(speeds);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getRotation2d());
        driveRobotRelative(relativeSpeeds);
    }

    public void driveRobotRelative(ChassisSpeeds relativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(relativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = Drive.Constants.DRIVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);
        frontLeft.setDesiredState(targetStates[0]);
        frontRight.setDesiredState(targetStates[1]);
        backLeft.setDesiredState(targetStates[2]);
        backRight.setDesiredState(targetStates[3]);
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
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
        return odometry.getPoseMeters();
    }

    public Pose2d getEstimatedPosition() { 
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Drive.Constants.DRIVE_KINEMATICS.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
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
            desiredStates[i].speedMetersPerSecond = 0;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Drive/Pose", poseEstimator.getEstimatedPosition());
        Logger.recordOutput("Drive/LimelightPose", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-one").pose);
        Logger.recordOutput("Drive/Pose/X", poseEstimator.getEstimatedPosition().getX());
        Logger.recordOutput("Drive/Pose/Y", poseEstimator.getEstimatedPosition().getY());
        Logger.recordOutput("Drive/Pose/Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        
        if (Operating.Constants.USING_GYRO) {
            Logger.recordOutput("Drive/Gyro/Yaw", gyro.getYaw().getValue());
            Logger.recordOutput("Drive/Gyro/Pitch", gyro.getPitch().getValue());
            Logger.recordOutput("Drive/Gyro/Roll", gyro.getRoll().getValue());
        }

        Logger.recordOutput("Drive/ModuleStates/Desired", desiredStates);
        Logger.recordOutput("Drive/ModuleStates/Actual", getSwerveModuleStates());

        Logger.recordOutput("Power/BatteryVoltage", RobotController.getBatteryVoltage());

        double now = Timer.getFPGATimestamp();
        if (now - lastMatchLog > 0.2) {
            lastMatchLog = now;
            Logger.recordOutput("Match/TimeRemaining", DriverStation.getMatchTime());
        }

        boolean teleop = DriverStation.isTeleopEnabled();
        boolean auton = DriverStation.isAutonomousEnabled();
        if (teleop != lastTeleopEnabled || auton != lastAutonomousEnabled) {
            lastTeleopEnabled = teleop;
            lastAutonomousEnabled = auton;
            Logger.recordOutput("Match/TeleopEnabled", teleop);
            Logger.recordOutput("Match/AutonomousEnabled", auton);
            String mode = auton ? "Autonomous" : teleop ? "Teleop" : "Disabled";
            Logger.recordOutput("Match/Mode", mode);
        }

        odometry.update(getRotation2d(), getSwerveModulePositions());
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation2d(), getSwerveModulePositions());

        //rename "limelight"
        if(Operating.Constants.USING_LIMELIGHT) {
            boolean useMegaTag2 = true; //set to false to use MegaTag1
            boolean doRejectUpdate = false;
            if(useMegaTag2 == false) {
                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-one");
                if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                    if(mt1.rawFiducials[0].ambiguity > .7) {
                        doRejectUpdate = true;
                    }   
                    if(mt1.rawFiducials[0].distToCamera > 3) {
                        doRejectUpdate = true;
                    }
                }
                if(mt1.tagCount == 0) {
                    doRejectUpdate = true;
                }
                if(!doRejectUpdate) {
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
                    poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
                }
            } else if (useMegaTag2 == true) {
                LimelightHelpers.SetRobotOrientation("limelight-one", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-one");
                if(Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) {
                    // if our angular velocity is greater than 720 degrees per second, ignore vision updates
                    doRejectUpdate = true;
                }
                if(mt2.tagCount == 0) {
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