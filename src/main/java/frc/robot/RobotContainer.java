// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drivetrain.Drivetrain;

public class RobotContainer {
  Drivetrain mainDrive;
  CommandXboxController controller;
  public RobotContainer() {
    mainDrive = new Drivetrain(true);
    controller = new CommandXboxController(0);
    configureBindings();
  }

  private void configureBindings() {
    mainDrive.setDefaultCommand(mainDrive.Drive(() -> MathUtil.applyDeadband(-controller.getLeftY(), 0.1), () -> MathUtil.applyDeadband(-controller.getLeftX(), 0.1), () -> MathUtil.applyDeadband(-controller.getRightX(), 0.1)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
