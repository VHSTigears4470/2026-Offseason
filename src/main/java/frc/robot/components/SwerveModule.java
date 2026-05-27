package frc.robot.components;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive.Constants.MotorLocation;

import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {

  private SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
  private final MotorLocation motorLocation;
   
  public SwerveModule(SwerveModuleIO io, MotorLocation location) {
    this.io = io;
    io.resetDriveEncoder();
    motorLocation = location;
  }

  @Override public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("SwerveModule/" + motorLocation.getFieldDescription(), inputs);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    io.setDesiredState(desiredState);
  }

  public SwerveModuleState getState() {
    //Apply chassis offset to the encoder position to get the position relative to the chassis.
    return new SwerveModuleState(
      inputs.driveVelocityMetersPerSec,
      new Rotation2d(inputs.turnPositionRad));
  }

  public SwerveModulePosition getPosition() {
    //Apply chassis offset to the encoder position to get the position relative to the chassis.
    return new SwerveModulePosition(
      inputs.drivePositionMeters,
      new Rotation2d(inputs.turnPositionRad));
 }

  public void resetDriveEncoder() {
    io.resetDriveEncoder();
  }

  public void stopMotors() {
    setDesiredState(new SwerveModuleState());
  }
}