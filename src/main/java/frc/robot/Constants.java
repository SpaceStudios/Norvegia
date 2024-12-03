// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotConstants.Motor;

/** Add your docs here. */
public class Constants {
    public class SwerveConstants {
        public class ModuleFLConstants {
            public static final Translation2d Position = new Translation2d(RobotConstants.robotWidth/2,RobotConstants.robotLength/2);
            public static final int Steer = 1;
            public static final int Drive = 2;
        }

        public class ModuleFRConstants {
            public static final Translation2d Position = new Translation2d(RobotConstants.robotWidth/2,-RobotConstants.robotLength/2);
            public static final int Steer = 3;
            public static final int Drive = 4;
        }

        public class ModuleRLConstants {
            public static final Translation2d Position = new Translation2d(-RobotConstants.robotWidth/2,RobotConstants.robotLength/2);
            public static final int Steer = 5;
            public static final int Drive = 6;
        }

        public class ModuleRRConstants {
            public static final Translation2d Position = new Translation2d(-RobotConstants.robotWidth/2,-RobotConstants.robotLength/2);
            public static final int Steer = 7;
            public static final int Drive = 8;
        }

        public class ModuleConstants {
            public static final Motor DriveMotor = Motor.TalonFX;
            public static final Motor SteerMotor = Motor.SparkMax;

            // Gear Ratios
            public static final double driveGearRatio = 150.0/7.0;
            public static final double steerGearRatio = 150.0/7.0;

            public static final double drivekP = 0.9;
            public static final double drivekI = 0.0;
            public static final double drivekD = 0.0;

            public static final double steerkP = 2.5;
            public static final double steerkI = 0.0;
            public static final double steerkD = 0.0;

            public static final double steerT = 0.0;
            public static final double steerTE = 0.0;
        }
    }

    public class RobotConstants {
        public static enum Mode {
            REAL,
            SIM
        }

        public static enum Motor {
            TalonFX,
            SparkMax
        }

        public static enum driveTrainType {
            Swerve,
            Differential
        }

        public static Mode currentMode = Mode.SIM;
        public static final driveTrainType currentDrivetrain = driveTrainType.Swerve;
        public static final double wheelSize = Units.inchesToMeters(4);
        public static final double maxTurnSpeed = 2*Math.PI; //Max Turn Speed in Radians
        public static final double maxSpeed = 80; //Max Speed in Meters per a Second
        public static final double robotWidth = 1; // Robot Width in Meters
        public static final double robotLength = 1; // Robot Length in Meters
    } 
}
