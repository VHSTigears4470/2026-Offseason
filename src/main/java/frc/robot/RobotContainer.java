// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

// import choreo.auto.AutoFactory;
// import choreo.auto.AutoRoutine;
// import choreo.auto.AutoTrajectory;
// import choreo.trajectory.Trajectory;
import org.wpilib.math.util.MathUtil;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.RunCommand;
import org.wpilib.command2.button.CommandGamepad;

import frc.robot.Constants.OI;
import frc.robot.Constants.Operating;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final CommandGamepad driverController = new CommandGamepad(OI.Constants.DRIVE_CONTROLLER_PORT);

  private DriveSubsystem driveSub;

  public RobotContainer() {

      initSubystems();
      configureBindings();
  }

  public void initSubystems() {
    if(Operating.Constants.USING_DRIVE)

      driveSub = new DriveSubsystem();
      // driverController.y().onTrue(myTrajectoryMeterAuto().cmd());
      driveSub.setDefaultCommand(new RunCommand(
          () -> {
            double y = OI.Constants.DRIVER_AXIS_Y_INVERTED * MathUtil
                .applyDeadband(driverController.getAxis(OI.Constants.DRIVER_AXIS_Y), OI.Constants.DRIVE_DEADBAND);
            double x = OI.Constants.DRIVER_AXIS_X_INVERTED * MathUtil
                .applyDeadband(driverController.getAxis(OI.Constants.DRIVER_AXIS_X), OI.Constants.DRIVE_DEADBAND);
            double rot = OI.Constants.DRIVER_AXIS_ROT_INVERTED * MathUtil
                .applyDeadband(driverController.getAxis(OI.Constants.DRIVER_AXIS_ROT), OI.Constants.DRIVE_DEADBAND);

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

    public Command getAutonomousCommand() { 
      return null;
    }

    private void configureBindings() {}
  }
