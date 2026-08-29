package frc.robot.components;

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
    private SparkClosedLoopController controller = null;

    public PIDMotorIOSparkFlex(int busID, int ID, SparkFlexConfig config) {
        motor = new SparkFlex(busID, ID, SparkLowLevel.MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
    }

   @Override public void updateInputs(PIDMotorIOInputsAutoLogged inputs) {
        inputs.RPM = encoder.getVelocity().get();
    }

    @Override public void resetEncoder() {
        encoder.setPosition(0);
    }

    @Override public void setSetpoint(double setpoint, SparkLowLevel.ControlType controlType, double FF) {
        controller.setSetpoint(setpoint, controlType, ClosedLoopSlot.kSlot0, FF);
    }

    @Override public void setVelocity(double RPM, double FF) {
        controller.setSetpoint(RPM, SparkLowLevel.ControlType.kVelocity, ClosedLoopSlot.kSlot0, FF);
    }

    @Override public void set(double speed){
        motor.setThrottle(speed);
    }

    @Override public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    @Override public void stopMotors(){
        motor.stopMotor();
    }

    @Override public double getEncoder(){
        return encoder.getPosition().get();
    }
}
