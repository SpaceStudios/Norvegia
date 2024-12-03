// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain.Swerve.ModuleSwerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.RobotConstants.Motor;
import frc.robot.Constants.SwerveConstants.ModuleConstants;

/** Add your docs here. */
public class ModuleSwerve_SIM implements ModuleSwerve {
    DCMotorSim driveMotorSim;
    DCMotorSim steerMotorSim;

    PIDController driveController;
    PIDController steerController;

    private double distance;

    public ModuleSwerve_SIM(Motor driveMotor, Motor steerMotor, int driveID, int SteerID) {
        driveMotorSim = new DCMotorSim(convertMotorEnum(driveMotor), ModuleConstants.driveGearRatio, 0.025);
        steerMotorSim = new DCMotorSim(convertMotorEnum(steerMotor), ModuleConstants.steerGearRatio, 0.004);

        driveController = new PIDController(ModuleConstants.drivekP,ModuleConstants.drivekI,ModuleConstants.drivekD, 0.020);
        steerController = new PIDController(ModuleConstants.steerkP, ModuleConstants.steerkI, ModuleConstants.steerkP, 0.020);

        steerController.enableContinuousInput(-Math.PI, Math.PI);
        
        distance = 0.0;
    }

    private DCMotor convertMotorEnum(Motor motorConvert) {
        switch (motorConvert) {
            case SparkMax:
                return DCMotor.getNEO(1);
            case TalonFX:
                return DCMotor.getKrakenX60(1);
            default:
                return null;
        }
    }

    @Override
    public void moduleStateDrive(SwerveModuleState setState) {
        setState = SwerveModuleState.optimize(setState, getTurnPosition());
        steerMotorSim.setInputVoltage(MathUtil.clamp(steerController.calculate(steerMotorSim.getAngularPositionRotations(),setState.angle.getRotations()), -12, 12));
        driveMotorSim.setInputVoltage(MathUtil.clamp(driveController.calculate(driveMotorSim.getAngularVelocityRadPerSec()*(RobotConstants.wheelSize/2),setState.speedMetersPerSecond*Math.cos(steerController.getPositionError())), -12, 12));
    }

    @Override
    public double getDrivePositionMeters() {
        return driveMotorSim.getAngularPositionRad()*(RobotConstants.wheelSize/2);
    }

    @Override
    public double getDriveVelocityMetersPerSecond() {
        return driveMotorSim.getAngularVelocityRadPerSec()*(RobotConstants.wheelSize/2);
    }

    @Override
    public Rotation2d getTurnPosition() {
        return Rotation2d.fromRadians(steerMotorSim.getAngularPositionRad());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePositionMeters(),Rotation2d.fromRadians(steerMotorSim.getAngularPositionRad()));
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getTurnPosition());
    }

    @Override
    public SwerveModulePosition getDifference() {
        SwerveModulePosition currentDifference = new SwerveModulePosition(getDrivePositionMeters()-distance, getTurnPosition());
        distance = getDrivePositionMeters();
        return currentDifference;
    }

    @Override
    public void update() {
        steerMotorSim.update(0.020);
        driveMotorSim.update(0.020);
    }
}
