package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorcontrol.MotorUtil;
import frc.robot.shuffleboard.GRTNetworkTableEntry;
import frc.robot.shuffleboard.GRTShuffleboardTab;

public class ElevatorSubsystem extends SubsystemBase {

    public double speed = 0;
    private final WPI_TalonSRX motorElevator = MotorUtil.createTalonSRX(MOTORWINCH);

    private final GRTShuffleboardTab shuffleboardTab = new GRTShuffleboardTab("Elevator");
    private final GRTNetworkTableEntry elevatorPositionEntry = shuffleboardTab.addEntry("ElevatorPosition", 0).at(0, 0);

    public enum Height {
        GROUND {
            public Height previous() {
                return Height.GROUND;
            }
            public double get(){
                return GROUNDHEIGHT;
            }
        }, // sitting on the ground
        LOW {
            public double get(){
                return LOWHEIGHT;
            }
        }, // a few inches above the ground so that you can move
        // BLOCK {
        //     public double get(){
        //         return BLOCKHEIGHT;
        //     }
        // }, // exactly 6 inches up (for setting blocks on another after being on HIGH)
        HIGH {
            public Height next() {
                return Height.HIGH;
            }
            public double get(){
                return HIGHHEIGHT;
            }
        }; // above 6 inches up so that you can clear the block

        public double get(){
            return 0;
        }

        public Height next() {
            System.out.println(ordinal());
            return values()[ordinal() + 1];
        }

        public Height previous() {
            return values()[ordinal() - 1];
        }
    }

    public Height height;

    public ElevatorSubsystem() {
        height = Height.GROUND;

        motorElevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        motorElevator.setSensorPhase(false);
        motorElevator.setSelectedSensorPosition(0);
        motorElevator.setNeutralMode(NeutralMode.Brake);
        motorElevator.setInverted(true);

        
        motorElevator.configForwardSoftLimitThreshold(WINCHMAX, 0);
        motorElevator.configForwardSoftLimitEnable(true, 0);

        motorElevator.configReverseSoftLimitThreshold(0, 0);
        motorElevator.configReverseSoftLimitEnable(true, 0);

    }

    public double speedfromdistance(double x){

        return (((-1.) / ( (1/4096) * Math.abs(x) + 1)) + 1) * (x / Math.abs(x));
    }

    public void periodic() {
        elevatorPositionEntry.setValue(motorElevator.getSelectedSensorPosition());
        
        double distance = height.get() - motorElevator.getSelectedSensorPosition();

        // motorElevator.set(speed);



        if(Math.abs(distance) > WINCHTOLERANCE){
            if(distance > 0){
                motorElevator.set(WINCHUPSPEED);
            } else {
                motorElevator.set(WINCHDOWNSPEED);
            }
        } else {
            motorElevator.set(WINCHSTOPSPEED);
        }

        // if(Math.abs(distance) < WINCHTOLERANCE){
            
        //     motorElevator.set(0);
        //     System.out.println("LJKSFDA");
        // } else {
        //     if(distance > 50000){
        //         distance = 50000;
        //     }
            // motorElevator.set(distance / 100000);

            // System.out.println(distance / 100000);
        // }
        // System.out.println(height);
    }
}
