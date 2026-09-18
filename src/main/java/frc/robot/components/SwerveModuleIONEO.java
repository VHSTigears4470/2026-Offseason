package frc.robot.components;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.Drive;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;


public class SwerveModuleIONEO implements SwerveModuleIO {

  private PIDMotor driveMotor = null;
  private PIDMotor turnMotor = null;

  private double chassisAngularOffset = 0;

  public SwerveModuleIONEO(CANPort driveBusID, CANPort turnBusID, int driveID, int turnID, double offset, SparkFlexConfig driveConfig, SparkMaxConfig turnConfig) {
    driveMotor = new PIDMotor(new PIDMotorIOSparkFlex(driveBusID, driveID, driveConfig, Drive.ModuleConstants.DRIVING_FACTOR, Drive.ModuleConstants.DRIVING_FACTOR / 60));
    turnMotor = new PIDMotor(new PIDMotorIOSparkMax(driveBusID, turnID, turnConfig, Drive.ModuleConstants.TURNING_FACTOR, Drive.ModuleConstants.TURNING_FACTOR / 60));
    chassisAngularOffset = offset;
  }

  @Override public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    inputs.drivePositionMeters = driveMotor.getEncoder();
    inputs.driveVelocityMetersPerSec = driveMotor.getVelocity();
    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

    inputs.turnPositionRad = turnMotor.getEncoder() - chassisAngularOffset;
    inputs.turnVelocityRadPerSec = turnMotor.getVelocity();
    inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
    inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
  }

  @Override public void setDesiredVelocity(SwerveModuleVelocity desiredVelocity)  {
    //Apply chassis offset to the desired state.
    SwerveModuleVelocity correctedDesiredVelocity = new SwerveModuleVelocity();
    correctedDesiredVelocity.velocity = desiredVelocity.velocity;
    correctedDesiredVelocity.angle = desiredVelocity.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    //Optimize the reference state as to not turn more than 90 degrees.
    correctedDesiredVelocity = correctedDesiredVelocity.optimize(new Rotation2d(turnMotor.getEncoder()));
    
    //Command driving and turning SPARKS toward their respective setpoints.
    driveMotor.setVelocity(correctedDesiredVelocity.velocity,  0);
    turnMotor.setSetpoint(correctedDesiredVelocity.angle.getRadians(), 0);
  }

  @Override public void resetDriveEncoder() {
    driveMotor.setEncoder(0);
  }
}
