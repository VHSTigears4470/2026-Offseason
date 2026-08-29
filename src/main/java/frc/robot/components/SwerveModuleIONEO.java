package frc.robot.components;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;


public class SwerveModuleIONEO implements SwerveModuleIO {

  private SparkFlex driveMotor = null;
  private SparkMax turnMotor = null;

  private RelativeEncoder driveEncoder = null;
  private AbsoluteEncoder turnEncoder = null;

  private SparkClosedLoopController driveController = null;
  private SparkClosedLoopController turnController = null;

  private double chassisAngularOffset = 0;

  public SwerveModuleIONEO(int driveBusID, int turnBusID, int driveID, int turnID, double offset, SparkFlexConfig driveConfig, SparkMaxConfig turnConfig) {
    driveMotor = new SparkFlex(driveBusID, driveID, MotorType.kBrushless);
    turnMotor = new SparkMax(driveBusID, turnID, MotorType.kBrushless);
    
    System.out.println(driveMotor.configAccessor.getInverted());

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder();

    driveController = driveMotor.getClosedLoopController();
    turnController = turnMotor.getClosedLoopController();

    chassisAngularOffset = offset;
    
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    inputs.drivePositionMeters = driveEncoder.getPosition().get();
    inputs.driveVelocityMetersPerSec = driveEncoder.getVelocity().get();
    inputs.driveAppliedVolts = driveMotor.getAppliedOutput().get() * driveMotor.getBusVoltage().get();
    inputs.driveCurrentAmps = driveMotor.getOutputCurrent().get();

    inputs.turnPositionRad = turnEncoder.getPosition().get() - chassisAngularOffset;
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity().get();
    inputs.turnAppliedVolts = turnMotor.getAppliedOutput().get() * turnMotor.getBusVoltage().get();
    inputs.turnCurrentAmps = turnMotor.getOutputCurrent().get();
  }

  @Override public void setDesiredVelocity(SwerveModuleVelocity desiredVelocity)  {
    //Apply chassis offset to the desired state.
    SwerveModuleVelocity correctedDesiredVelocity = new SwerveModuleVelocity();
    correctedDesiredVelocity.velocity = desiredVelocity.velocity;
    correctedDesiredVelocity.angle = desiredVelocity.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    //Optimize the reference state as to not turn more than 90 degrees.
    correctedDesiredVelocity = correctedDesiredVelocity.optimize(new Rotation2d(turnEncoder.getPosition().get()));
    
    //Command driving and turning SPARKS toward their respective setpoints.
    driveController.setSetpoint(correctedDesiredVelocity.velocity, ControlType.kVelocity);
    turnController.setSetpoint(correctedDesiredVelocity.angle.getRadians(), ControlType.kPosition);
  }

  @Override public void resetDriveEncoder() {
    driveEncoder.setPosition(0);
  }
}