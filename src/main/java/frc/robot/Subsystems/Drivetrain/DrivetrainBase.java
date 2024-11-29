// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public interface DrivetrainBase {
    public class drivetrainData {
        public Pose2d currentPosition = new Pose2d();
    }

    public abstract void driveChassisSpeeds(ChassisSpeeds speeds);
    public abstract void getData(drivetrainData data);
    public abstract Rotation2d getRobotRotation();
}
