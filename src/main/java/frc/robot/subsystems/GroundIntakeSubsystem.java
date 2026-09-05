package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Configs;
import frc.robot.Constants.IDs.IntakeConstants;
import frc.robot.components.PIDMotor;
import frc.robot.components.PIDMotorIO;
import frc.robot.components.PIDMotorIOSparkMax;

public class GroundIntakeSubsystem extends SubsystemBase {
    
    private final PIDMotor intakeMotor;
    private final PIDMotor rotateMotor;
    private boolean isRetracted;
    private boolean isIntaking;

    public GroundIntakeSubsystem() {
        intakeMotor = new PIDMotor(new PIDMotorIOSparkMax(IntakeConstants.INTAKE_MOTOR_ID, Configs.Intake.INTAKE_CONFIG));
        rotateMotor = new PIDMotor(new PIDMotorIOSparkMax(IntakeConstants.ROTATE_MOTOR_ID, Configs.Intake.ROTATE_CONFIG));
        isRetracted = true;
        isIntaking = false;
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
        Logger.recordOutput("GroundIntake/IntakeSpeed", speed);
    }

    public void setRotateSpeed(double speed) {
        rotateMotor.set(speed);
        Logger.recordOutput("GroundIntake/RotateSpeed", speed);
    }

    public void toggleIntake() {
        isIntaking = !isIntaking;
    }

    public void toggleRetract() {
        isRetracted = !isRetracted;
    }

    public void extend() {
        rotateMotor.setSetpoint(17.0, 0);
    }

    public void retract() {
        rotateMotor.setSetpoint(-3.0, 0);
    }

    public boolean isIntaking() {
        return isIntaking;
    }

    public boolean isRetracted() {
        return isRetracted;
    }

    public void stopMotors() {
        intakeMotor.stopMotors();
        rotateMotor.stopMotors();
    }

    public double getRotation() {
        return rotateMotor.getEncoder();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("IntakeSubsystem/Is Retracted", isRetracted);
        Logger.recordOutput("IntakeSubsystem/Rotate Value", getRotation());
        Logger.recordOutput("IntakeSubsystem/IntakeMotorRPM", intakeMotor.getRPM());
        Logger.recordOutput("IntakeSubsystem/RotateMotorRPM", rotateMotor.getRPM());
    }
}