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
        public static final int LEFT_MAIN = 12;
        public static final int LEFT_SECONDARY = 13;
        public static final int RIGHT_MAIN = 14;
        public static final int RIGHT_SECONDARY = 15;

    }
    public static final class IntakeConstants{
        public static final int FRONT_MOTOR = 16;
        public static final int FRONT_OPP_MOTOR = 1;
        public static final int RIGHT_MOTOR = 2;
        public static final int LEFT_MOTOR = 3;

    }
    public static final class CarriageConstants{
        public static final int CARRIAGE_SOLENOID_F = 6;
        public static final int CARRIAGE_SOLENOID_R = 7;
        public static final int TOP_SERVO = 4;
        public static final int BOTTOM_SERVO = 5;

        public static final int TOP_OPEN = 90; // servo position when the door is open
        public static final int TOP_CLOSED = 0; // servo position when the door is closed

        public static final int BOTTOM_OPEN = 90; // servo position when the door is open
        public static final int BOTTOM_CLOSED = 0; // servo position when the door is closed


    }


    public static final class ShuffleboardConstants {
        public static final double UPDATE_TIME = 0.5; // seconds between each update
    }
}
