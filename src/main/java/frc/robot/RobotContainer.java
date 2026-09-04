// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandGamepad;

import frc.robot.commands.DriveMotors;
import frc.robot.subsystems.TestMotorsSubsystem;


public class RobotContainer {
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandGamepad m_driverController =
      new CommandGamepad(0);

  private TestMotorsSubsystem testMotor;
      
  public RobotContainer() {
    initSubsystems();    
    // Configure the trigger bindings
    configureBindings();
  }

  private void initSubsystems() {
      testMotor = new TestMotorsSubsystem(0, 13, false);
  }

  private void configureBindings() {
    int preset = 1;
    switch (preset) {
      default:
        controllerPresetMain();
        break;
    }
  }

  public Command getAutonomousCommand() {
        return null;
  }

  public void controllerPresetMain() { //subject to change (while/on true)
        double speed = 0.35;
        m_driverController.dpadUp().whileTrue(new DriveMotors(testMotor, speed));
  }

}