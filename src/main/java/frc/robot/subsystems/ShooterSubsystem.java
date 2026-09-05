package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command2.SubsystemBase;

import frc.robot.Constants.IDs.CANBUSIDs;
import frc.robot.Constants.IDs.ShooterIDs;
import frc.robot.Constants.Configs.ShooterConfigs;
import frc.robot.Constants.Shooter;
import frc.robot.components.PIDMotor;
import frc.robot.components.PIDMotorIOSparkFlex;
import frc.robot.components.PIDMotorIOSparkMax;

public class ShooterSubsystem extends SubsystemBase {
    private final PIDMotor flywheelMotor; 
    private final PIDMotor hoodMotor;
    private final PIDMotor feederMotor;
    private final PIDMotor hopperMotor;
    
    private double toHubPosition; 
    private boolean shooterActive;
    private boolean shootingManual;

    public ShooterSubsystem() {
        flywheelMotor = new PIDMotor(new PIDMotorIOSparkFlex(CANBUSIDs.SHOOTER_CANBUS_ID, ShooterIDs.FLYWHEEL_ID, ShooterConfigs.FLYWHEEL_CONFIG));
        hoodMotor = new PIDMotor(new PIDMotorIOSparkMax(CANBUSIDs.SHOOTER_CANBUS_ID, ShooterIDs.HOOD_ID, ShooterConfigs.HOOD_CONFIG));
        feederMotor = new PIDMotor(new PIDMotorIOSparkMax(CANBUSIDs.SHOOTER_CANBUS_ID, ShooterIDs.FEEDER_ID, ShooterConfigs.FEEDER_CONFIG));
        hopperMotor = new PIDMotor(new PIDMotorIOSparkMax(CANBUSIDs.SHOOTER_CANBUS_ID, ShooterIDs.HOPPER_ID, ShooterConfigs.HOPPER_CONFIG));

        toHubPosition = 0;
        shooterActive = false;
    }

    //public void updateDesiredRPM(double distance){}

    public boolean flywheelReady() {
        return (shooterActive && 
            Math.abs(flywheelMotor.getRPM() - Shooter.Constants.SHOOTER_RPM) < Shooter.Constants.SHOOTER_RPM * 0.15);
    }
        
    public void toggleShooter(boolean shootingManual) {
        shooterActive = !shooterActive;
        this.shootingManual = shootingManual;
    }

    public boolean isShooting() {
        return shooterActive;
    }

    public void setFeeder(double speed) {
        feederMotor.set(speed);
        Logger.recordOutput("Shooter/Feeder", speed);
    }

    public void setHopper(double speed) {
        hopperMotor.set(speed);
        Logger.recordOutput("Shooter/Hopper", speed);
    }

    public void updateDesiredRPM(double distance) {
        toHubPosition = Shooter.Constants.HOOD_ANGLE_TABLE.get(distance);
        Logger.recordOutput("Shooter/Aligned Position", toHubPosition);
    }

    public void stopMotors() {
        flywheelMotor.stopMotors();
        hopperMotor.stopMotors();
        feederMotor.stopMotors();
        feederMotor.stopMotors();
    }

    @Override
    public void periodic() {
        if (!shooterActive) {
            if (Math.abs(flywheelMotor.getRPM()) < 200) {
                flywheelMotor.set(0);
            } else {
                flywheelMotor.setVelocity(0, 0.00020352);
            }
            hoodMotor.setSetpoint(0, 0);
        } else {
            flywheelMotor.setVelocity(Shooter.Constants.SHOOTER_RPM, 0.00020352);
            if(shootingManual) {
                hoodMotor.setSetpoint(Shooter.Constants.MANUAL_SHOT_POSITION, 0);
                Logger.recordOutput("Shooter/Desired Position", Shooter.Constants.MANUAL_SHOT_POSITION);
            } else {
                hoodMotor.setSetpoint(toHubPosition, 0);
                Logger.recordOutput("Shooter/Desired Position", toHubPosition);
            }
        }

        if (shooterActive && flywheelReady()) {
            setFeeder(0.95);
            setHopper(0.5);
        } else {
            setFeeder(0);
            setHopper(0);
        }

        Logger.recordOutput("RPM/Actual", flywheelMotor.getRPM());
    }
}
