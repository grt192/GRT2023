// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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
        public static final Translation2d tlPos = new Translation2d();

        public static final int trDrive = 0;
        public static final int trSteer = 0;
        public static final double trOffsetRads = 0;
        public static final Translation2d trPos = new Translation2d();

        public static final int blDrive = 0;
        public static final int blSteer = 0;
        public static final double blOffsetRads = 0;
        public static final Translation2d blPos = new Translation2d();

        public static final int brDrive = 0;
        public static final int brSteer = 0;
        public static final double brOffsetRads = 0;
        public static final Translation2d brPos = new Translation2d();
    }

    public static final class ShuffleboardConstants {
        public static final double UPDATE_TIME = 0.5; // seconds between each update
    }
}
