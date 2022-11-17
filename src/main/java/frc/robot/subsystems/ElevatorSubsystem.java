package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.motorWinch;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorcontrol.MotorUtil;

public class ElevatorSubsystem extends SubsystemBase {

    public enum Height {
        GROUND {
            public Height previous() {
                return Height.HIGH;
            }
        }, // sitting on the ground
        LOW, // a few inches above the ground so that you can move
        BLOCK, // exactly 6 inches up (for setting blocks on another after being on HIGH)
        HIGH {
            public Height next() {
                return Height.GROUND;
            }
        }; // above 6 inches up so that you can clear the block

        public Height next() {
            return values()[ordinal() + 1];
        }

        public Height previous() {
            return values()[ordinal() - 1];
        }
    }

    public Height height;
    private final WPI_TalonSRX winchmotor = MotorUtil.createTalonSRX(motorWinch);

    public ElevatorSubsystem() {
        height = Height.GROUND;
    }

    public void periodic() {
        // figure this out
    }
}
