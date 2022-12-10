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

    public static final class TankConstants {
        public static final int MOTOR_BACKLEFT = 14;
        public static final int MOTOR_FRONTLEFT = 15;
        public static final int MOTOR_BACKRIGHT = 12;
        public static final int MOTOR_FRONTRIGHT = 13;

    }

    public static final class ElevatorConstants {
        public static final int MOTORWINCH = 2;

        //encoder values, STUB https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html
        public static final double GROUNDHEIGHT = 0; //0 inches
        public static final double LOWHEIGHT = -244000; //1 inch
        //public static final double BLOCKHEIGHT = 600; //6 inches
        public static final double HIGHHEIGHT = -321000; //7 inches

        public static final double WINCHUPSPEED = .4;
        //this is swapped rn
        public static final double WINCHDOWNSPEED = -.7;
        public static final double WINCHSTOPSPEED = 0;
        public static final double WINCHTOLERANCE = 10000;

        public static final double WINCHMAX = 170000;
    }

    public static final class GripperConstants {
        public static final int SOLENOIDPORT = 0;
        public static final int SOLENOIDPORT2 = 1;
    }

    public static final class AlignerConstants {
        public static final int SLAPID = 0;
        public static final int ANGLEID = 1;

        public static final double TRIGGERCUTOFF = .2;

        // open position target encoder values
        public static final double OPENSLAP = 108940;// should be 0, as motor starts from this position
        public static final double OPENANGLER = -83339;// should be 0, as motor starts from this position

        // slap slapper target encoder values
        public static final  double SLAPSLAP = 190000;// 

        // closed position target encoder values
        public static final double CLOSEDSLAP = 162720;// 
        public static final double CLOSEDANGLER = -149329;// 
    }

}
