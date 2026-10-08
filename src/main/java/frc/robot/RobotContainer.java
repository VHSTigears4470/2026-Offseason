// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.spline.SplineHelper;
// import choreo.auto.AutoFactory;
// import choreo.auto.AutoRoutine;
// import choreo.auto.AutoTrajectory;
// import choreo.trajectory.Trajectory;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.TunableRegistry;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.RunCommand;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.driverstation.XboxController;
import org.wpilib.command2.CommandScheduler;


import frc.robot.Constants.OI;
import frc.robot.Constants.Operating;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.IntakeToggle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private final CommandGamepad driverController = new CommandGamepad(OI.Constants.DRIVE_CONTROLLER_PORT);

  private final Selectable <Command> autoChooser;

  private DriveSubsystem driveSub = null;
  private ShooterSubsystem shooterSub = null;
  private GroundIntakeSubsystem intakeSub = null;

  public RobotContainer() {
      initSubystems();
      
      if (Operating.Constants.USING_AUTO) {
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.add("meter", new PathPlannerAuto("meter"));
        // TunableRegistry.publish("Auto", autoChooser);
      } else {
        autoChooser = null;
      }

      configureBindings();
      //CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand()); // changed after wpilib alpha 5 removing .schedule() on cmd v2
   }

  public void initSubystems() {
    if(Operating.Constants.USING_DRIVE) {
      driveSub = new DriveSubsystem();
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
      if(Operating.Constants.USING_INTAKE) {
        intakeSub = new GroundIntakeSubsystem();
      }
       if(Operating.Constants.USING_SHOOTER) {
        shooterSub = new ShooterSubsystem();
      }
    }
  

    public Command getAutonomousCommand() { 
      if (Operating.Constants.USING_AUTO) {
        return autoChooser.getSelected();
      }

      return null;
    }

    private void configureBindings() {
      //1 - motor testing
      //defualt - Competition Controller Scheme
      int preset = 1; 
      switch (preset) {
        case 0:
          if(Operating.Constants.USING_INTAKE) {
            driverController.button(XboxController.Button.X.value).onTrue(new IntakeRun(intakeSub));
            driverController.button(XboxController.Button.A.value).onTrue(new IntakeToggle(intakeSub));
            driverController.button(XboxController.Button.LEFT_BUMPER.value).onTrue(new InstantCommand(() -> {intakeSub.extend();}));
            driverController.button(XboxController.Button.RIGHT_BUMPER.value).onTrue(new InstantCommand(() -> {intakeSub.retract();}));
          }
          if(Operating.Constants.USING_SHOOTER) {
            //might be overriden by periodic()
            driverController.dpadLeft().onTrue(new InstantCommand(() -> {shooterSub.setFeeder(11);}));
            driverController.dpadRight().onTrue(new InstantCommand(() -> {shooterSub.setFeeder(-11);}));
            driverController.dpadDown().onTrue(new InstantCommand(() -> {shooterSub.setIndexer(8);}));
            driverController.dpadUp().onTrue(new InstantCommand(() -> {shooterSub.setIndexer(-8);}));
            driverController.button(XboxController.Button.MENU.value).onTrue(new InstantCommand(() -> {shooterSub.setIndexer(0); shooterSub.setFeeder(0);}));
            driverController.button(XboxController.Button.B.value).onTrue(new ParallelCommandGroup(new InstantCommand(() -> {shooterSub.toggleShooter(true);}), new IntakeRun(intakeSub)));
            //driverController.button(XboxController.Button.Y.value).onTrue(new InstantCommand(() -> {})); align + shoot
          }
          break;
        default:
          if(Operating.Constants.USING_INTAKE) {
            driverController.button(XboxController.Button.X.value).onTrue(new IntakeRun(intakeSub));
            driverController.button(XboxController.Button.A.value).onTrue(new IntakeToggle(intakeSub));
          }
          if(Operating.Constants.USING_SHOOTER) {
            driverController.button(XboxController.Button.B.value).onTrue(new ParallelCommandGroup(new InstantCommand(() -> {shooterSub.toggleShooter(true);}), new IntakeRun(intakeSub)));
            //driverController.button(XboxController.Button.Y.value).onTrue(new InstantCommand(() -> {})); align + shoot

            driverController.dpadLeft().onTrue(new InstantCommand(() -> {shooterSub.setFeeder(11);}));
            driverController.dpadRight().onTrue(new InstantCommand(() -> {shooterSub.setFeeder(-11);}));
            driverController.dpadDown().onTrue(new InstantCommand(() -> {shooterSub.setIndexer(8);}));
            driverController.dpadUp().onTrue(new InstantCommand(() -> {shooterSub.setIndexer(-8);}));
            driverController.button(XboxController.Button.MENU.value).onTrue(new InstantCommand(() -> {shooterSub.setIndexer(0); shooterSub.setFeeder(0);}));
          }
          break;
      }
    }
  }
