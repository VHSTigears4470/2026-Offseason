package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.ControlType;

import frc.robot.Constants.CAN;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Configs.ShooterConfigs.IntakeConfigs;
import frc.robot.components.PIDMotor;
import frc.robot.components.PIDMotorIOSparkMax;

public class GroundIntakeSubsystem extends SubsystemBase {
    private final PIDMotor intakeMotor;
    private final PIDMotor dropMotor;

    private boolean isIntakeDown;
    private boolean isRunning;

    public GroundIntakeSubsystem() {
        intakeMotor = new PIDMotor(new PIDMotorIOSparkMax(
            CAN.Constants.IntakeCAN,
            IDs.IntakeIDs.INTAKE_ID, 
            IntakeConfigs.INTAKE_CONFIG,
            1,
            1)
        );

        dropMotor = new PIDMotor(new PIDMotorIOSparkMax(
            CAN.Constants.IntakeCAN, 
            IDs.IntakeIDs.DROP_ID, 
            IntakeConfigs.DROP_CONFIG,
            1, 
            1)
        );
        
        // we can assume its always starting at up
        isIntakeDown = false;
    }

    public void setIntake(double speed) {
        isRunning = (speed != 0);
        intakeMotor.set(speed);
    }

    public void extend() {
        isIntakeDown = true;
        dropMotor.setSetpoint(Intake.Constants.downwardsEncoderAngle, ControlType.kMAXMotionPositionControl, 0);
        setIntake(0.2);    
    }

    public void retract() {
        isIntakeDown = false;
        dropMotor.setSetpoint(Intake.Constants.upwardsEncoderAngle, ControlType.kMAXMotionPositionControl, 0); //should always be up
        setIntake(-0.2);
    }

    public boolean isIntakeDown(){
        return isIntakeDown;
    }

    public void toggleIntake() {
        isRunning = !isRunning;
    }
    
    public boolean isRunning() {
        return isRunning;
    }

    public boolean isAtSetpoint(){
        double currentSetpoint = (isIntakeDown) ? Intake.Constants.downwardsEncoderAngle : Intake.Constants.upwardsEncoderAngle;
        return Math.abs(dropMotor.getEncoder() - currentSetpoint) < 0.5;
    }

    public void stopMotors() {
        intakeMotor.stopMotors();
        dropMotor.stopMotors();
    }

    public double getRotation() {
        return dropMotor.getEncoderAbs();
    }

    public void periodic() {
        Logger.recordOutput("IntakeSubsystem/Is Extended", isIntakeDown);
        Logger.recordOutput("IntakeSubsystem/Rotate Value", getRotation());   
    }   
}