// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.commands;
import frc.robot.commands.swerve_joystick_commands;
import frc.robot.subsystems.swerve_subsystem;

public class RobotContainer {
  private final XboxController controller = new XboxController(constants.ids.controller); //TODO: what is CommandXboxController?
  private final swerve_subsystem swerve_subsystem = new swerve_subsystem();
  private final commands commands = new commands();

  public RobotContainer() {
    swerve_subsystem.setDefaultCommand(
      new swerve_joystick_commands(
      swerve_subsystem, 
      () -> controller.getLeftX(), 
      () -> controller.getLeftY(),
      () -> controller.getRightX() 
      )
    );
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(controller, XboxController.Button.kA.value).whileTrue(commands.set_robot_heading_zero(swerve_subsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
