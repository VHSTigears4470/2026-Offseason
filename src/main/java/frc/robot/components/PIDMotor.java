package frc.robot.components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;

public class PIDMotor extends SubsystemBase {

  private PIDMotorIO io;
  private final PIDMotorIOInputsAutoLogged inputs = new PIDMotorIOInputsAutoLogged();
   
  public PIDMotor(PIDMotorIO io) {
    this.io = io;
    io.resetEncoder();
  }

  @Override public void periodic() {
    io.updateInputs(inputs);
  }

  public void setSetpoint(double setpoint, double FF) {
    io.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl, FF);
    Logger.recordOutput("PIDMotor/Setpoint", setpoint);
  }

  public void setVelocity(double RPM, double FF) {
    io.setVelocity(RPM, FF);
    Logger.recordOutput("PIDMotor/SetVelocity", RPM);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
    Logger.recordOutput("PIDMotor/SetVoltage", voltage);
  }

  public void set(double speed) {
    io.set(speed);
    Logger.recordOutput("PIDMotor/Set", speed);
  }

  public double getRPM() {
    return inputs.RPM;
  }

  public void stopMotors() {
    io.stopMotors();
    Logger.recordOutput("PIDMotor/StopMotors", true);
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  public double getEncoder() {
    return io.getEncoder();
  }
}