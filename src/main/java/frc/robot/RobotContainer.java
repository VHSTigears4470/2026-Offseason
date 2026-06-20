// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OI;
import frc.robot.Constants.Operating;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(OI.Constants.DRIVE_CONTROLLER_PORT);

  private DriveSubsystem driveSub;

  private final AutoFactory autoFactory;

  public RobotContainer() {
    initSubystems();
     autoFactory = new AutoFactory(
            driveSub::getOdometry, // A function that returns the current robot pose - might have to implement limelignt first
            driveSub::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            driveSub::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            driveSub // The drive subsystem
        );
    configureBindings();
  }

  public void initSubystems() {
    if(Operating.Constants.USING_DRIVE)
      driveSub = new DriveSubsystem();

      driveSub.setDefaultCommand(new RunCommand(
          () -> {
            double y = OI.Constants.DRIVER_AXIS_Y_INVERTED * MathUtil
                .applyDeadband(driverController.getRawAxis(OI.Constants.DRIVER_AXIS_Y), OI.Constants.DRIVE_DEADBAND);
            double x = OI.Constants.DRIVER_AXIS_X_INVERTED * MathUtil
                .applyDeadband(driverController.getRawAxis(OI.Constants.DRIVER_AXIS_X), OI.Constants.DRIVE_DEADBAND);
            double rot = OI.Constants.DRIVER_AXIS_ROT_INVERTED * MathUtil
                .applyDeadband(driverController.getRawAxis(OI.Constants.DRIVER_AXIS_ROT), OI.Constants.DRIVE_DEADBAND);

            // Add logging for buttons

            // Record operator inputs with the project logger
            Logger.recordOutput("Operator/Drive/Y", y);
            Logger.recordOutput("Operator/Drive/X", x);
            Logger.recordOutput("Operator/Drive/Rot", rot);
            Logger.recordOutput("Operator/Drive/LeftTrigger", driverController.leftTrigger().getAsBoolean());
            Logger.recordOutput("Operator/Drive/RightTrigger", driverController.rightTrigger().getAsBoolean());

            driveSub.drive(y, x, rot, true, "Default / Field Oriented");
          },
          driveSub));
    }

  private void configureBindings() {}
}
