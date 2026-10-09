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
    
    private double desiredRPM; 
    private boolean shooterActive;

    public ShooterSubsystem() {
        flywheelMotor = new PIDMotor(new PIDMotorIOSparkFlex(CAN.Constants.ShooterCAN, ShooterIDs.FLYWHEEL_ID, ShooterConfigs.FLYWHEEL_CONFIG, 1, 1));
        hoodMotor = new PIDMotor(new PIDMotorIOSparkMax(CAN.Constants.ShooterCAN, ShooterIDs.HOOD_ID, ShooterConfigs.HOOD_CONFIG, 1, 1));
        feederMotor = new PIDMotor(new PIDMotorIOSparkMax(CAN.Constants.FeederCAN, ShooterIDs.FEEDER_ID, ShooterConfigs.FEEDER_CONFIG, 1, 1));
        indexerMotor = new PIDMotor(new PIDMotorIOSparkMax(CAN.Constants.FeederCAN, ShooterIDs.INDEXER_ID, ShooterConfigs.INDEXER_CONFIG, 1, 1));

        desiredRPM = 0;
        shooterActive = false;
    }

    public void setFlywheel(double speed) {
        flywheelMotor.setVelocity(speed, 0);
    }

    public boolean flywheelReady() {
        return (Math.abs(flywheelMotor.getVelocity() - desiredRPM) < Math.abs(Shooter.Constants.HUB_RPM * 0.05));
    }
        
    public void toggleShooter(double flywheelVelocity, double hoodSetpoint) {
        shooterActive = !shooterActive;

        if (!shooterActive) {
            flywheelMotor.setVoltage(0);
            setFeeder(0);
            setIndexer(0);
            hoodMotor.setSetpoint(-0.2, ControlType.kPosition, 0);
        } else {
            flywheelMotor.setVelocity(flywheelVelocity, 0); //-4.236
            hoodMotor.setSetpoint(hoodSetpoint, ControlType.kPosition, 0);
            desiredRPM = flywheelVelocity;
        }
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
        } 
        
        Logger.recordOutput("Shooter/Flywheel Ready", flywheelReady());
        Logger.recordOutput("Shooter/Flywheel Amperage", flywheelMotor.getOutputCurrent());
        Logger.recordOutput("RPM/Actual", flywheelMotor.getVelocity());
        Logger.recordOutput("Shooter/Hood Amperage", hoodMotor.getOutputCurrent());
    }
}
