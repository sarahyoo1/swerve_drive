// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.swerve_joystick_commands;
import frc.robot.subsystems.swerve_subsystem;

public class RobotContainer {
  private final XboxController controller = new XboxController(constants.ids.controller);
  private final swerve_subsystem swerve_subsystem = new swerve_subsystem();
  
  public RobotContainer() {
    swerve_subsystem.setDefaultCommand(
      new swerve_joystick_commands(
      swerve_subsystem, 
      () -> controller.getLeftX(), 
      () -> controller.getLeftY(), 
      () -> controller.getRightX(), 
      () -> controller.getAButton())
    );
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
