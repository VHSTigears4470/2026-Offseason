package frc.robot.components;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class PIDMotorIOSparkMax implements PIDMotorIO {
    private SparkMax motor = null;
    private RelativeEncoder encoder = null;
    private SparkClosedLoopController controller = null;

    public PIDMotorIOSparkMax(int ID, SparkMaxConfig config) {
        motor = new SparkMax(ID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
    }

   @Override public void updateInputs(PIDMotorIOInputsAutoLogged inputs) {
        inputs.RPM = encoder.getVelocity();
    }

    @Override public void resetEncoder() {
        encoder.setPosition(0);
    }

    @Override public void setSetpoint(double setpoint, ControlType controlType, double FF) {
        controller.setSetpoint(setpoint, controlType, ClosedLoopSlot.kSlot0, FF);
    }

    @Override public void setVelocity(double RPM, double FF) {
        controller.setSetpoint(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0, FF);
    }

    @Override public void set(double speed){
        motor.set(speed);
    }

    @Override public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    @Override public void stopMotors(){
        motor.stopMotor();
    }

    @Override public double getEncoder(){
        return encoder.getPosition();
    }
}
