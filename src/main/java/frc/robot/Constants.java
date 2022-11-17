// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


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
        public static final int left_main = 1;
        public static final int left_secondary = 2;
        public static final int right_main = 3;
        public static final int right_secondary = 4;

    }
    public static final class IntakeConstants{
        public static final int front_motor = 5;
        public static final int right_motor = 6;
        public static final int left_motor = 7;
    }
    public static final class CarriageConstants{
        public static final int carriage_solenoid = 1;
        public static final int top_servo = 1;
        public static final int bottom_servo = 1;


    }


    public static final class ShuffleboardConstants {
        public static final double UPDATE_TIME = 0.5; // seconds between each update
    }
}
