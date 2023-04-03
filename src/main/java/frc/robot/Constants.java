// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

import frc.robot.sensors.HallEffectSensor;

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
    public static final boolean IS_R1 = false;
    public static final boolean GLOBAL_SHUFFLEBOARD_ENABLE = false;

    public static final class TankConstants {
        public static final int LEFT_MAIN = 19;
        public static final int LEFT_FOLLOW = 18;
        public static final int RIGHT_MAIN = 20;
        public static final int RIGHT_FOLLOW = 1;
    }

    public static final class SwerveConstants {
        public static final double TL_OFFSET_RADS = IS_R1 
            ? -2.9255307594961444 // 0.21606189409364873 
            : -2.4960072914785663;
        public static final double TR_OFFSET_RADS = IS_R1 
            ? -1.1199431538621365 // 2.0216494997276566
            : -0.3756485104600369; //2.7659441431297562;
        public static final double BL_OFFSET_RADS = IS_R1
            ? -2.374440566697393
            : -0.0563106973979135 ; //3.0852819561918796
        public static final double BR_OFFSET_RADS = IS_R1
            ? -2.8609512726492206 // 0.28064138094057256
            :  0.6332055648141584; //-2.5083870887756348

        public static final int TL_DRIVE = 2;
        public static final int TL_STEER = 3;
        public static final Translation2d TL_POS = IS_R1 ? new Translation2d(
            Units.inchesToMeters(12.0),
            Units.inchesToMeters(8.75)
        ) : new Translation2d(
            Units.inchesToMeters(13.625),
            Units.inchesToMeters(10.25)
        );

        public static final int TR_DRIVE = 16;
        public static final int TR_STEER = 17;
        public static final Translation2d TR_POS = IS_R1 ? new Translation2d(
            Units.inchesToMeters(12.0),
            Units.inchesToMeters(-8.75)
        ) : new Translation2d(
            Units.inchesToMeters(13.625),
            Units.inchesToMeters(-10.25)
        );

        public static final int BL_DRIVE = 20;
        public static final int BL_STEER = 1;
        public static final Translation2d BL_POS = IS_R1 ? new Translation2d(
            Units.inchesToMeters(-12.0),
            Units.inchesToMeters(8.75)
        ) : new Translation2d(
            Units.inchesToMeters(-13.625),
            Units.inchesToMeters(10.25)
        );

        public static final int BR_DRIVE = 18;
        public static final int BR_STEER = 19;
        public static final Translation2d BR_POS = IS_R1 ? new Translation2d(
            Units.inchesToMeters(-12.0),
            Units.inchesToMeters(-8.75)
        ) : new Translation2d(
            Units.inchesToMeters(-13.625),
            Units.inchesToMeters(-10.25)
        );
    }

    public static final class SwerveConstants2020 {
        public static final int TL_DRIVE = 14;
        public static final int TL_STEER = 10;
        public static final double TL_OFFSET_RADS = 1.53398078789;
        public static final Translation2d TL_POS = new Translation2d(
            Units.inchesToMeters(13.1365),
            Units.inchesToMeters(10.3865)
        );

        public static final int TR_DRIVE = 4;
        public static final int TR_STEER = 5;
        public static final double TR_OFFSET_RADS = 1.31922347758 + Math.PI;
        public static final Translation2d TR_POS = new Translation2d(
            Units.inchesToMeters(13.1365),
            Units.inchesToMeters(-10.3865)
        );

        public static final int BL_DRIVE = 13;
        public static final int BL_STEER = 11;
        public static final double BL_OFFSET_RADS = 2.28256341237 + Math.PI;
        public static final Translation2d BL_POS = new Translation2d(
            Units.inchesToMeters(-13.1365),
            Units.inchesToMeters(10.3865)
        );

        public static final int BR_DRIVE = 1;
        public static final int BR_STEER = 12;
        public static final double BR_OFFSET_RADS = 3.135456730438264;
        public static final Translation2d BR_POS = new Translation2d(
            Units.inchesToMeters(-13.1365),
            Units.inchesToMeters(-10.3865)
        );
    }

    public static final class TiltedElevatorConstants {
        public static final int EXTENSION_ID = 7;
        public static final int EXTENSION_FOLLOW_ID = 8;
        public static final int EXTENSION_FOLLOW_B_ID = 9;

        public static final float EXTENSION_LIMIT_METERS = (float) Units.inchesToMeters(60.25 + 0.75); // extra 3/4" to account for steady-state error in PID
        public static final double EXTENSION_TOLERANCE_METERS = Units.inchesToMeters(1);

        public static final int ZERO_LIMIT_ID = 1;
        public static final int LEFT_HALL_ID = 4;
        public static final HallEffectSensor.Magnet[] LEFT_MAGNETS = {
            new HallEffectSensor.Magnet(EXTENSION_LIMIT_METERS) // TODO: find actual location of hall effect!
        };
    }

    public static final class VisionConstants {
        public static final PhotonCamera FRONT_CAMERA = new PhotonCamera("Arducam_OV9281_USB_Camera");
        public static final Transform3d FRONT_CAMERA_POSE = new Transform3d(
            // new Translation3d(Units.inchesToMeters(10.375), Units.inchesToMeters(10.597), Units.inchesToMeters(22.638875)),
            new Translation3d(Units.inchesToMeters(10.125), Units.inchesToMeters(7.767605), Units.inchesToMeters(22.25)),
            new Rotation3d(Math.PI, 0, 0)
        );

        public static final PhotonCamera RIGHT_CAMERA = new PhotonCamera("HD_USB_Camera");
        public static final Transform3d RIGHT_CAMERA_POSE = new Transform3d(
            // new Translation3d(Units.inchesToMeters(7.375), Units.inchesToMeters(10.597), Units.inchesToMeters(22.638875)),
            new Translation3d(Units.inchesToMeters(10.125), Units.inchesToMeters(-7.767605), Units.inchesToMeters(22.25)),
            new Rotation3d(Math.PI, 0, 0)
        );

        // public static final PhotonCamera BACK_CAMERA = new PhotonCamera("HD_USB_Camera");
        // public static final Transform3d BACK_CAMERA_POSE = new Transform3d(
        //     // new Translation3d(Units.inchesToMeters(7.375), Units.inchesToMeters(10.597), Units.inchesToMeters(22.638875)),
        //     new Translation3d(Units.inchesToMeters(7.625), Units.inchesToMeters(7.767605), Units.inchesToMeters(22.25)),
        //     new Rotation3d(Math.PI, 0, Math.PI)
        // );
    }

    public static final class LEDConstants {
        public static final int LED_PWM_PORT = 1;
        public static final int LED_LENGTH = 144;
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

        public static final double ALLOW_OPEN_EXTENSION_METERS = Units.inchesToMeters(17);
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

    public static final class BalancerConstants {
        public static final double GRT_CHARGING_STATION_KP = 0.3 / 35;
        public static final double COMP_CHARGING_STATION_KP = 0.007;
    }

    public static final class FieldConstants {
        public static final double ROBOT_LENGTH_INCHES = 38.5;

        public static final double GRID_X_INCHES = 53.938 + (ROBOT_LENGTH_INCHES / 2.0);
        public static final double ALIGNMENT_OFFSET_INCHES = 12.885;

        public static final double C3_Y_INCHES = 196.19;
        public static final double C2_Y_INCHES = 174.19;
        public static final double C1_Y_INCHES = 152.19;

        public static final double B3_Y_INCHES = 130.19;
        public static final double B2_Y_INCHES = 108.19;
        public static final double B1_Y_INCHES = 86.19;

        public static final double A3_Y_INCHES = 64.19;
        public static final double A2_Y_INCHES = 42.19;
        public static final double A1_Y_INCHES = 20.19;
    }
}
