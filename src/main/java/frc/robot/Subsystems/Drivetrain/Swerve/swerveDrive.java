// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain.Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants.*;
import frc.robot.Subsystems.Drivetrain.DrivetrainBase;
import frc.robot.Subsystems.Drivetrain.Swerve.ModuleSwerve.ModuleSwerve;
import frc.robot.Subsystems.Drivetrain.Swerve.ModuleSwerve.ModuleSwerve_SIM;

/** Add your docs here. */
public class swerveDrive implements DrivetrainBase{
    SwerveDriveKinematics kinematics;
    ModuleSwerve fLModuleSwerve;
    ModuleSwerve fRModuleSwerve;
    ModuleSwerve rLModuleSwerve;
    ModuleSwerve rRModuleSwerve;
    Rotation2d GyroRotation;
    SwerveDriveOdometry swerveOdometry;

    public swerveDrive() {
        kinematics = new SwerveDriveKinematics(ModuleFLConstants.Position, ModuleFRConstants.Position, ModuleRLConstants.Position, ModuleRRConstants.Position);
        switch (RobotConstants.currentMode) {
            case SIM:
                fLModuleSwerve = new ModuleSwerve_SIM(ModuleConstants.DriveMotor, ModuleConstants.SteerMotor, ModuleFLConstants.Drive, ModuleFLConstants.Steer);
                fRModuleSwerve = new ModuleSwerve_SIM(ModuleConstants.DriveMotor, ModuleConstants.SteerMotor, ModuleFRConstants.Drive, ModuleFRConstants.Steer);
                rLModuleSwerve = new ModuleSwerve_SIM(ModuleConstants.DriveMotor, ModuleConstants.SteerMotor, ModuleRLConstants.Drive, ModuleRLConstants.Steer);
                rRModuleSwerve = new ModuleSwerve_SIM(ModuleConstants.DriveMotor, ModuleConstants.SteerMotor, ModuleRRConstants.Drive, ModuleRRConstants.Steer);
                break;
            default:
                break;
        }
        SwerveModulePosition[] newModulePositions = new SwerveModulePosition[] {fLModuleSwerve.getModulePosition(),fRModuleSwerve.getModulePosition(),rLModuleSwerve.getModulePosition(),rRModuleSwerve.getModulePosition()};
        GyroRotation =  new Rotation2d();
        swerveOdometry = new SwerveDriveOdometry(kinematics, GyroRotation, newModulePositions);
    }
    @Override
    public void driveChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] SetModuleStates = kinematics.toSwerveModuleStates(speeds);
        fLModuleSwerve.moduleStateDrive(SetModuleStates[0]);
        fRModuleSwerve.moduleStateDrive(SetModuleStates[1]);
        rLModuleSwerve.moduleStateDrive(SetModuleStates[2]);
        rRModuleSwerve.moduleStateDrive(SetModuleStates[3]);
        Logger.recordOutput("Intended States", SetModuleStates);
        Logger.recordOutput("Actual States", new SwerveModuleState[] {fLModuleSwerve.getModuleState(),fRModuleSwerve.getModuleState(),rLModuleSwerve.getModuleState(),rRModuleSwerve.getModuleState()});
        SwerveModulePosition[] swerveDeltas = new SwerveModulePosition[] {fLModuleSwerve.getDifference(),fRModuleSwerve.getDifference(),rLModuleSwerve.getDifference(),rRModuleSwerve.getDifference()};
        SwerveModulePosition[] swervePos = new SwerveModulePosition[] {fLModuleSwerve.getModulePosition(),fRModuleSwerve.getModulePosition(),rLModuleSwerve.getModulePosition(),rRModuleSwerve.getModulePosition()};
        Twist2d currentTwist = kinematics.toTwist2d(swerveDeltas);
        GyroRotation = GyroRotation.plus(new Rotation2d(currentTwist.dtheta));
        Logger.recordOutput("Gyro Rotation", GyroRotation);
        Logger.recordOutput("Intended Angle", SetModuleStates[0].angle.getRadians());
        Logger.recordOutput("Actual Angle", fLModuleSwerve.getTurnPosition().getRadians());
        Logger.recordOutput("Intended Speed", SetModuleStates[0].speedMetersPerSecond);
        Logger.recordOutput("Actual Speed", fLModuleSwerve.getDriveVelocityMetersPerSecond());
        swerveOdometry.update(GyroRotation, swervePos);
    }

    @Override
    public void getData(drivetrainData data) {
        data.currentPosition = swerveOdometry.getPoseMeters();
    }
    @Override
    public Rotation2d getRobotRotation() {
        return GyroRotation;
    }
    @Override
    public void periodic() {
        fLModuleSwerve.update();
        fRModuleSwerve.update();
        rLModuleSwerve.update();
        rRModuleSwerve.update();
    }
}