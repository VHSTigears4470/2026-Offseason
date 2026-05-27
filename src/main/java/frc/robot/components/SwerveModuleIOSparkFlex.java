package frc.robot.components;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleIOSparkFlex implements SwerveModuleIO {

  private SparkMax driveMotor = null;
  private SparkMax turnMotor = null;

  private RelativeEncoder driveEncoder = null;
  private AbsoluteEncoder turnEncoder = null;

  private SparkClosedLoopController driveController = null;
  private SparkClosedLoopController turnController = null;

  private double chassisAngularOffset = 0;

  public SwerveModuleIOSparkFlex(int driveID, int turnID, double offset, SparkFlexConfig driveConfig, SparkFlexConfig turnConfig) {
    driveMotor = new SparkMax(driveID, MotorType.kBrushless);
    turnMotor = new SparkMax(turnID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder();

    driveController = driveMotor.getClosedLoopController();
    turnController = turnMotor.getClosedLoopController();

    chassisAngularOffset = offset;
    
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMetersPerSec = driveEncoder.getVelocity();
    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

    inputs.turnPositionRad = turnEncoder.getPosition() - chassisAngularOffset;
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
    inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
  }

  @Override public void setDesiredState(SwerveModuleState desiredState)  {
    //Apply chassis offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    //Optimize the reference state as to not turn more than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(turnEncoder.getPosition()));
    
    //Command driving and turning SPARKS toward their respective setpoints.
    driveController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    turnController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
  }

  @Override public void resetDriveEncoder() {
    driveEncoder.setPosition(0);
  }
}