package frc.robot.components;

import org.wpilib.hardware.bus.CANPort;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;

public class PIDMotorIOSparkFlex implements PIDMotorIO {
    private SparkFlex motor = null;
    private RelativeEncoder encoder = null;
    private AbsoluteEncoder encoderAbs = null;
    private SparkClosedLoopController controller = null;
    private double encoderConversionFactor = 1;
    private double velocityConversionFactor = 1;

    public PIDMotorIOSparkFlex(CANPort busID, int ID, SparkFlexConfig config, double encoderConversionFactor, double velocityConversionFactor) {
        motor = new SparkFlex(busID, ID, SparkLowLevel.MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.encoderConversionFactor = encoderConversionFactor;
        this.velocityConversionFactor = velocityConversionFactor;
        encoder = motor.getEncoder();
        encoderAbs = motor.getAbsoluteEncoder();
        controller = motor.getClosedLoopController();
    }

   @Override public void updateInputs(PIDMotorIOInputsAutoLogged inputs) {
        inputs.velocity = getVelocity();
    }

    @Override public void resetEncoder() {
        encoder.setPosition(0);
    }

    @Override public void setSetpoint(double setpoint, SparkLowLevel.ControlType controlType, double FF) {
        controller.setSetpoint(setpoint / encoderConversionFactor, controlType, ClosedLoopSlot.kSlot0, FF);
    }

    @Override public void setVelocity(double velocity, double FF) {
        controller.setSetpoint(velocity / velocityConversionFactor, SparkLowLevel.ControlType.kVelocity, ClosedLoopSlot.kSlot0, FF);
    }

    @Override public void set(double speed){
        motor.setThrottle(speed);
    }

    @Override public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    @Override public void setEncoder(double position){
        encoder.setPosition(position / encoderConversionFactor);
    }

    @Override public void stopMotors(){
        motor.stopMotor();
    }

    @Override public double getEncoder(){
        return encoder.getPosition().get() * encoderConversionFactor;
    }

    @Override public double getVelocity(){
        return encoder.getVelocity().get() * velocityConversionFactor;
    }

    @Override public double getAppliedOutput(){
        return motor.getAppliedOutput().get();
    }

    @Override public double getBusVoltage(){
        return motor.getBusVoltage().get();
    }

    @Override public double getOutputCurrent(){
        return motor.getOutputCurrent().get();
    }
}
