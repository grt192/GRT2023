// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static enum DrivetrainType {
        KOP,
        SWERVE,
        SWERVE_2020
    }

    public static final DrivetrainType DT_TYPE = DrivetrainType.KOP;

    public static final class RollerConstants {
        public static final int LEFT_ID, RIGHT_ID, OPEN_ID;

        static {
            switch (DT_TYPE) {
                case KOP:
                    LEFT_ID = 15;
                    RIGHT_ID = 14;
                    OPEN_ID = 16;
                    break;
                case SWERVE:
                default:
                    LEFT_ID = 12;
                    RIGHT_ID = 13;
                    OPEN_ID = 14;
            }
        }
    }

    public static final class GripperConstants {
        public static final int PFFT_FORWARD_ID = 0;
        public static final int PFFT_REVERSE_ID = 1;
    }

    public static final class PivotElevatorConstants {
        public static final int ROTATION_MOTOR_PORT = 4;
        public static final int ROTATION_FOLLOWER_MOTOR_PORT = 5;
        public static final int EXTENSION_MOTOR_PORT = 19;

        public static final double ROTATION_GEAR_RATIO = 1/155.91;
        public static final double EXTENSION_GEAR_RATIO = 1/27;

        public static final double ANGLE_OFFSET_SPEED = 0.001;
        public static final double EXTENSION_OFFSET_SPEED = 0.001;
    }

    public static final class TiltedElevatorConstants {
        public static final int EXTENSION_ID = 3;
        public static final int EXTENSION_FOLLOW_ID = 2;
        public static final float EXTENSION_LIMIT = (float) 26; 

        public static final int ZERO_LIMIT_ID = 1;
        
        public static final double EXT_GR = 14.0 / 64.0;
        public static final double EXT_CIRCUM = Units.inchesToMeters(Math.PI * 0.500); // approx circumference of winch
        public static final double EXTENSION_ROT_TO_M = EXT_GR * EXT_CIRCUM * 2.0 * (15.0 / 13.4); // rotations to meters
        
    }

    public static final class TankConstants {
        public static final int LEFT_MAIN = 2;
        public static final int LEFT_FOLLOW = 3;
        public static final int RIGHT_MAIN = 0;
        public static final int RIGHT_FOLLOW = 1;
    }

    public static final class SwerveConstants {
        public static int tlDrive, trDrive, blDrive, brDrive;
        public static int tlSteer, trSteer, blSteer, brSteer;
        public static double tlOffsetRads, trOffsetRads, blOffsetRads, brOffsetRads;
        public static Translation2d tlPos, trPos, blPos, brPos;

        static {
            switch (DT_TYPE) {
                case SWERVE:
                    tlDrive = 2;
                    tlSteer = 1;
                    tlOffsetRads = 0.57522836526;
                    tlPos = new Translation2d(
                            Units.inchesToMeters(12.0),
                            Units.inchesToMeters(8.75));

                    trDrive = 4;
                    trSteer = 3;
                    trOffsetRads = -0.842838776116;
                    trPos = new Translation2d(
                            Units.inchesToMeters(12.0),
                            Units.inchesToMeters(-8.75));

                    // TODO: reflash sparkmaxes to fix this
                    blDrive = 8;
                    blSteer = 7;
                    blOffsetRads = -0.793255341057;
                    blPos = new Translation2d(
                            Units.inchesToMeters(-12.0),
                            Units.inchesToMeters(8.75));

                    brDrive = 6;
                    brSteer = 5;
                    brOffsetRads = 0.561284053322;
                    brPos = new Translation2d(
                            Units.inchesToMeters(-12.0),
                            Units.inchesToMeters(-8.75));
                    break;
                case SWERVE_2020:
                default:
                    tlDrive = 14;
                    tlSteer = 10;
                    tlOffsetRads = 1.53398078789;
                    tlPos = new Translation2d(
                            Units.inchesToMeters(13.1365),
                            Units.inchesToMeters(10.3865));

                    trDrive = 4;
                    trSteer = 5;
                    trOffsetRads = 1.31922347758 + Math.PI;
                    trPos = new Translation2d(
                            Units.inchesToMeters(13.1365),
                            Units.inchesToMeters(-10.3865));

                    blDrive = 13;
                    blSteer = 11;
                    blOffsetRads = 2.28256341237 + Math.PI;
                    blPos = new Translation2d(
                            Units.inchesToMeters(-13.1365),
                            Units.inchesToMeters(10.3865));

                    brDrive = 1;
                    brSteer = 12;
                    brOffsetRads = 3.135456730438264;
                    brPos = new Translation2d(
                            Units.inchesToMeters(-13.1365),
                            Units.inchesToMeters(-10.3865));
                    break;
            }
        }
    }
}
