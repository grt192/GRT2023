// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

import java.util.Arrays;
import java.util.List;

import org.photonvision.PhotonCamera;

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
    public static final class TankConstants{
        public static final int LEFT_MAIN = 19;
        public static final int LEFT_FOLLOW = 18;
        public static final int RIGHT_MAIN = 20;
        public static final int RIGHT_FOLLOW = 1;
    }

    public static final class SwerveConstants {
        public static final int tlDrive = 2;
        public static final int tlSteer = 3;
        public static final double tlOffsetRads = 0.57522836526;
        public static final Translation2d tlPos = new Translation2d(
            Units.inchesToMeters(12.0),
            Units.inchesToMeters(8.75)
        );

        public static final int trDrive = 16;
        public static final int trSteer = 17;
        public static final double trOffsetRads = -0.842838776116;
        public static final Translation2d trPos = new Translation2d(
            Units.inchesToMeters(12.0),
            Units.inchesToMeters(-8.75)
        );

        // TODO: reflash sparkmaxes to fix this
        public static final int blDrive = 20;
        public static final int blSteer = 1;
        public static final double blOffsetRads = -0.793255341057;
        public static final Translation2d blPos = new Translation2d(
            Units.inchesToMeters(-12.0),
            Units.inchesToMeters(8.75)
        );

        public static final int brDrive = 18;
        public static final int brSteer = 19;
        public static final double brOffsetRads = 0.561284053322;
        public static final Translation2d brPos = new Translation2d(
            Units.inchesToMeters(-12.0),
            Units.inchesToMeters(-8.75)
        );
    }

    public static final class SwerveConstants2020 {
        public static final int tlDrive = 14;
        public static final int tlSteer = 10;
        public static final double tlOffsetRads = 1.53398078789;
        public static final Translation2d tlPos =  new Translation2d(
            Units.inchesToMeters(13.1365),
            Units.inchesToMeters(10.3865)
        );

        public static final int trDrive = 4;
        public static final int trSteer = 5;
        public static final double trOffsetRads = 1.31922347758 + Math.PI;
        public static final Translation2d trPos = new Translation2d(
            Units.inchesToMeters(13.1365),
            Units.inchesToMeters(-10.3865)
        );

        public static final int blDrive = 13;
        public static final int blSteer = 11;
        public static final double blOffsetRads = 2.28256341237 + Math.PI;
        public static final Translation2d blPos = new Translation2d(
            Units.inchesToMeters(-13.1365),
            Units.inchesToMeters(10.3865)
        );

        public static final int brDrive = 1;
        public static final int brSteer = 12;
        public static final double brOffsetRads = 3.135456730438264;
        public static final Translation2d brPos = new Translation2d(
            Units.inchesToMeters(-13.1365),
            Units.inchesToMeters(-10.3865)
        );
    }

    public static final class TiltedElevatorConstants {
        public static final int EXTENSION_ID = 7;
        public static final int EXTENSION_FOLLOW_ID = 8;
        public static final int EXTENSION_FOLLOW_B_ID = 9;
        public static final int ZERO_LIMIT_ID = 1;
    }

    public static final class VisionConstants {
        public static PhotonCamera FRONT_CAMERA = new PhotonCamera("Arducam_OV9281_USB_Camera");
        public static Transform3d FRONT_CAMERA_POSE = new Transform3d(
            // new Translation3d(Units.inchesToMeters(10.375), Units.inchesToMeters(10.597), Units.inchesToMeters(22.638875)),
            new Translation3d(Units.inchesToMeters(8.25), Units.inchesToMeters(10.597), Units.inchesToMeters(22.638875)),
            new Rotation3d(0, 0, 0)
        );
    
        public static PhotonCamera BACK_CAMERA = new PhotonCamera("HD_USB_Camera");
        public static Transform3d BACK_CAMERA_POSE = new Transform3d(
            // new Translation3d(Units.inchesToMeters(7.375), Units.inchesToMeters(10.597), Units.inchesToMeters(22.638875)),
            new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(10.597), Units.inchesToMeters(22.638875)),
            new Rotation3d(0, 0, Math.PI)
        );
    }

    public static final class GripperConstants {
        public static final int PFFT_FORWARD_IDL = 0;
        public static final int PFFT_REVERSE_IDL = 1;
        public static final int PFFT_FORWARD_IDR = 2;
        public static final int PFFT_REVERSE_IDR = 3;
    }

    public static final class RollerConstants {
        public static final int OPEN_ID = 12;
        public static final int LEFT_ID = 13;
        public static final int RIGHT_ID = 14; 
        public static final int LIMIT_SWITCH_ID = 0;
    }

    public static final class MoverConstants {
        public static final int ROTATION_MOTOR_PORT = 4;
        public static final int ROTATION_FOLLOWER_MOTOR_PORT = 5;
        public static final int EXTENSION_MOTOR_PORT = 19;

        public static final double ROTATION_GEAR_RATIO = 1/155.91;
        public static final double EXTENSION_GEAR_RATIO = 1/27;

        public static final double ANGLE_OFFSET_SPEED = 0.001;
        public static final double EXTENSION_OFFSET_SPEED = 0.001;
    }
}
