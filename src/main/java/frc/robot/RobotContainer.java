// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OI;

public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(OI.Constants.DRIVE_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OI.Constants.OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    initSubystems();
    configureBindings();

  }

  public void initSubystems() {
  }

  private void configureBindings() {}
}
