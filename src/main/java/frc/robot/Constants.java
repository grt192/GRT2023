// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
    public static final class SwerveConstants {
        public static final int tlDrive = 1;
        public static final int tlSteer = 0;
        public static final double tlOffsetRads = 0;
        public static final Translation2d tlPos = new Translation2d(
            Units.inchesToMeters(11.0),
            Units.inchesToMeters(11.0)
        );

        public static final int trDrive = 0;
        public static final int trSteer = 0;
        public static final double trOffsetRads = 0;
        public static final Translation2d trPos = new Translation2d(
            Units.inchesToMeters(11.0),
            Units.inchesToMeters(-11.0)
        );

        public static final int blDrive = 0;
        public static final int blSteer = 0;
        public static final double blOffsetRads = 0;
        public static final Translation2d blPos = new Translation2d(
            Units.inchesToMeters(-11.0),
            Units.inchesToMeters(11.0)
        );

        public static final int brDrive = 0;
        public static final int brSteer = 0;
        public static final double brOffsetRads = 0;
        public static final Translation2d brPos = new Translation2d(
            Units.inchesToMeters(-11.0),
            Units.inchesToMeters(-11.0)
        );
    }

    // TODO
    public static final class VisionConstants {
        public static final String LIFECAM_NAME = "Microsoft_LifeCam_HD-3000";
        public static final Transform3d cameraPos = new Transform3d(
            new Translation3d(0, 0, 48),
            new Rotation3d()
        );

        public static final String COPROCESSOR_IP = "10.1.92.12";
        public static final int[] COPROCESSOR_IP_PORTS = {
            1182
        };
    }

    public static final class ShuffleboardConstants {
        public static final double UPDATE_TIME = 0.5; // seconds between each update
    }
}
