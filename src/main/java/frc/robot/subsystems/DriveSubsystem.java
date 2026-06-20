package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.hardware.Pigeon2;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.SwerveModule;
import frc.robot.components.SwerveModuleIONEO;
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
        
    private double lastMatchLog = 0.0;
    private boolean lastTeleopEnabled = false;
    private boolean lastAutonomousEnabled = false;

    //No clue about these values
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
            getSwerveModulePosition());

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
        Pose2d pose = getOdometry(); //Change to getPose() after implementing limelight

        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        driveFieldRelative(speeds);
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = Drive.Constants.DRIVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);
        frontLeft.setDesiredState(targetStates[0]);
        frontRight.setDesiredState(targetStates[1]);
        backLeft.setDesiredState(targetStates[2]);
        backRight.setDesiredState(targetStates[3]);
    }

    //Verify
    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(fieldRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = Drive.Constants.DRIVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);
        frontLeft.setDesiredState(targetStates[0]);
        frontRight.setDesiredState(targetStates[1]);
        backLeft.setDesiredState(targetStates[2]);
        backRight.setDesiredState(targetStates[3]);
    }

    public SwerveModuleState[] getSwerveModuleState() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    public SwerveModulePosition[] getSwerveModulePosition() {
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

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Drive.Constants.DRIVE_KINEMATICS.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            getRotation2d(),
            getSwerveModulePosition(),
            pose);
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
        if (Operating.Constants.USING_GYRO) {
            Logger.recordOutput("Drive/Gyro/Yaw", gyro.getYaw().getValue());
            Logger.recordOutput("Drive/Gyro/Pitch", gyro.getPitch().getValue());
            Logger.recordOutput("Drive/Gyro/Roll", gyro.getRoll().getValue());
        }

        Logger.recordOutput("Drive/ModuleStates/Desired", desiredStates);
        Logger.recordOutput("Drive/ModuleStates/Actual", getSwerveModuleState());

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

        odometry.update(getRotation2d(), getSwerveModulePosition());
    }
}