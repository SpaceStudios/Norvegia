// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain.Swerve.ModuleSwerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public interface ModuleSwerve {
    public abstract void moduleStateDrive(SwerveModuleState setState); // Sets Swerve Module State in Meters;
    public abstract double getDrivePositionMeters(); // Returns Drive Motor's current Position in meters
    public abstract double getDriveVelocityMetersPerSecond(); // Returns Drive Motor's current Velocity in Meter per a second
    public abstract Rotation2d getTurnPosition(); // Returns Turn Motors Position in Radians
    public abstract SwerveModulePosition getModulePosition();
    public abstract SwerveModuleState getModuleState();
    public abstract SwerveModulePosition getDifference();
}
