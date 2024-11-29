// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Subsystems.Drivetrain.Swerve.swerveDrive;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  DrivetrainBase drivetrain;
  boolean FieldRelative = true;
  public Drivetrain(boolean IsFieldRelative) {
    switch (RobotConstants.currentDrivetrain) {
      case Swerve:
        drivetrain = new swerveDrive();
        break;
      default:
        System.out.println(RobotConstants.currentDrivetrain.toString() + " is not supported");
        break;
    }
    FieldRelative = IsFieldRelative;
  }
  
  public Command Drive(DoubleSupplier Joystick1H, DoubleSupplier Joystick1V, DoubleSupplier Joystick2H) {
    return new RunCommand(() -> {
      this.drive(Joystick1H.getAsDouble(), Joystick1V.getAsDouble(), Joystick2H.getAsDouble());
    },
    this);
  }

  private void drive(Double jStick1H, Double jStick1V, Double jStick2H) {
    ChassisSpeeds desiredSpeeds = FieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(jStick1H*RobotConstants.maxSpeed, jStick1V*RobotConstants.maxSpeed, jStick2H*RobotConstants.maxTurnSpeed, drivetrain.getRobotRotation()) : new ChassisSpeeds(jStick1H*RobotConstants.maxSpeed, jStick1V*RobotConstants.maxSpeed, jStick2H*RobotConstants.maxTurnSpeed);
    drivetrain.driveChassisSpeeds(desiredSpeeds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
