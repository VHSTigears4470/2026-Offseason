package frc.robot.components;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

public class PIDMotorIOSparkFlex implements PIDMotorIO {
    private SparkFlex motor = null;
    private RelativeEncoder encoder = null;
    private SparkClosedLoopController controller = null;

    public PIDMotorIOSparkFlex(int ID, SparkFlexConfig config) {
        
    motor = new SparkFlex(ID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();
    
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
