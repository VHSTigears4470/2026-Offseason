package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Configs.Shooter;
import frc.robot.Constants.IDs.ShooterConstants;
import frc.robot.components.PIDMotor;
import frc.robot.components.PIDMotorIOSparkFlex;
import frc.robot.components.PIDMotorIOSparkMax;

public class ShooterSubsystem extends SubsystemBase {
    private final PIDMotor flywheelRight; //leader
    private final PIDMotor flywheelLeft;
    private final PIDMotor feeder;
    private final PIDMotor hopper;
    private double RPMToHub; //needs to be calculated properly
    
    private double desiredRPM;
    private boolean shooterActive;

    
    public ShooterSubsystem() {
        flywheelRight = new PIDMotor(new PIDMotorIOSparkFlex(ShooterConstants.FLYWHEEL_RIGHT_ID, Shooter.FLYWHEEL_RIGHT_CONFIG));
        flywheelLeft = new PIDMotor(new PIDMotorIOSparkFlex(ShooterConstants.FLYWHEEL_LEFT_ID, Shooter.FLYWHEEL_RIGHT_CONFIG));
        feeder = new PIDMotor(new PIDMotorIOSparkMax(ShooterConstants.FEEDER_ID, Shooter.FEEDER_CONFIG));
        hopper = new PIDMotor(new PIDMotorIOSparkMax(ShooterConstants.HOPPER_ID, Shooter.HOPPER_CONFIG));

        RPMToHub = 0;
        desiredRPM = 0;
        shooterActive = false;
    }

    public void setDesiredRPM(double desiredRPM){
        this.desiredRPM = desiredRPM;
        Logger.recordOutput("Shooter/Desired RPM", desiredRPM);
    }

    //public void updateDesiredRPM(double distance){}

    public boolean flywheelReady(){
        return (desiredRPM != 0)
            ? Math.abs(flywheelRight.getRPM() - desiredRPM) < desiredRPM * 0.15 
            : Math.abs(flywheelRight.getRPM() - RPMToHub) < RPMToHub * 0.15;
    }
        
    public void toggleShooter() {
        shooterActive = !shooterActive;
        if (!shooterActive)
            desiredRPM = 0;
    }

    public boolean isShooting(){
        return shooterActive;
    }

    public void setFeeder(double speed) {
        feeder.set(speed);
        Logger.recordOutput("Shooter/Feeder", speed);
    }

    public void setHopper(double speed) {
        hopper.set(speed);
        Logger.recordOutput("ShooterHopper", speed);
    }

    public void stopMotors() {
        flywheelRight.stopMotors();
        flywheelLeft.stopMotors();
        feeder.stopMotors();
        hopper.stopMotors();
    }

    @Override
    public void periodic() {
        if (!shooterActive)
        {
            if (Math.abs(flywheelRight.getRPM()) < 200)
            {
                flywheelRight.set(0);
            } 
            else
            {
                flywheelRight.setVelocity(0, 0.00020352);
            }
        }
        else if (desiredRPM != 0)
        {
            flywheelRight.setVelocity(desiredRPM, 0.00020352);
        }
        else
        {
            flywheelRight.setVelocity(RPMToHub, 0.00020352);
        }
        if (shooterActive && flywheelReady())
        {
            setFeeder(0.95);
            setHopper(0.5);
        }
        else
        {
            setFeeder(0);
            setHopper(0);
        }
        Logger.recordOutput("RPM/Actual", flywheelRight.getRPM());
    }
}
