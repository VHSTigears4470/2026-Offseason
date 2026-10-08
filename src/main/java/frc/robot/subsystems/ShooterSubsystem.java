package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command2.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.ControlType;

import frc.robot.Constants.IDs.ShooterIDs;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Configs.ShooterConfigs;
import frc.robot.Constants.Shooter;
import frc.robot.components.PIDMotor;
import frc.robot.components.PIDMotorIOSparkFlex;
import frc.robot.components.PIDMotorIOSparkMax;

public class ShooterSubsystem extends SubsystemBase {
    private final PIDMotor flywheelMotor; 
    private final PIDMotor hoodMotor;
    private final PIDMotor feederMotor;
    private final PIDMotor indexerMotor;
    
    private double toHubPosition; 
    private boolean shooterActive;
    private boolean shootingManual;

    public ShooterSubsystem() {
        flywheelMotor = new PIDMotor(new PIDMotorIOSparkFlex(CAN.Constants.ShooterCAN, ShooterIDs.FLYWHEEL_ID, ShooterConfigs.FLYWHEEL_CONFIG, 1, 1));
        hoodMotor = new PIDMotor(new PIDMotorIOSparkMax(CAN.Constants.ShooterCAN, ShooterIDs.HOOD_ID, ShooterConfigs.HOOD_CONFIG, 1, 1));
        feederMotor = new PIDMotor(new PIDMotorIOSparkMax(CAN.Constants.FeederCAN, ShooterIDs.FEEDER_ID, ShooterConfigs.FEEDER_CONFIG, 1, 1));
        indexerMotor = new PIDMotor(new PIDMotorIOSparkMax(CAN.Constants.FeederCAN, ShooterIDs.INDEXER_ID, ShooterConfigs.INDEXER_CONFIG, 1, 1));

        toHubPosition = 0;
        shooterActive = false;
    }

    public void setFlywheel(double speed) {
        flywheelMotor.setVelocity(speed, 0);
    }

    public boolean flywheelReady() {
        return (Math.abs(flywheelMotor.getVelocity() - Shooter.Constants.SHOOTER_RPM) < Math.abs(Shooter.Constants.SHOOTER_RPM * 0.05));
    }
        
    public void toggleShooter(boolean shootingManual) {
        shooterActive = !shooterActive;

        if (!shooterActive) {
            flywheelMotor.setVoltage(0);
            setFeeder(0);
            setIndexer(0);
            //hoodMotor.setSetpoint(0, ControlType.kMAXMotionPositionControl, 0);
        } else {
            flywheelMotor.setVelocity(Shooter.Constants.SHOOTER_RPM, 0); //-4.236
        }

        this.shootingManual = shootingManual;
    }

    public boolean isShooting() {
        return shooterActive;
    }

    public void setFeeder(double voltage) {
        feederMotor.setVoltage(voltage);
        Logger.recordOutput("Shooter/Feeder", voltage);
    }

    public void setIndexer(double voltage) {
        indexerMotor.setVoltage(voltage);
        Logger.recordOutput("Shooter/Hopper", voltage);
    }

    public void setHoodSetpoint(double setpoint) {
        hoodMotor.setSetpoint(setpoint, ControlType.kPosition, 0);
        Logger.recordOutput("Shooter/Hood Desired Setpoint", setpoint);
    }

    public void updateDesiredPosition(double distance) {
        toHubPosition = Shooter.Constants.HOOD_ANGLE_TABLE.get(distance);
        Logger.recordOutput("Shooter/Aligned Position", toHubPosition);
    }

    public void stopMotors() {
        flywheelMotor.stopMotors();
        indexerMotor.stopMotors();
        hoodMotor.stopMotors();
        feederMotor.stopMotors();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/Actual RPM", -flywheelMotor.getVelocity());
        Logger.recordOutput("Shooter/Flywheel Temp", flywheelMotor.getMotorTemperature());

        if (shooterActive) {
            if (flywheelReady()) {
                setFeeder(-11);
                setIndexer(-8);
            } else {
                 setFeeder(0);
                 setIndexer(0);
            }

            /*if(shootingManual) {
                hoodMotor.setSetpoint(Shooter.Constants.MANUAL_SHOT_POSITION, ControlType.kPosition, 0);
                Logger.recordOutput("Shooter/Desired Position", Shooter.Constants.MANUAL_SHOT_POSITION);
            } else {
                hoodMotor.setSetpoint(Math.max(Shooter.Constants.HOOD_THRESHOLD, toHubPosition), ControlType.kPosition, 0);
                Logger.recordOutput("Shooter/Desired Position", toHubPosition);
            }*/
        } 
        
        Logger.recordOutput("Shooter/Flywheel Ready", flywheelReady());
        Logger.recordOutput("Shooter/Flywheel Amperage", flywheelMotor.getOutputCurrent());
        Logger.recordOutput("RPM/Actual", flywheelMotor.getVelocity());
    }
}
