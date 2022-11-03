package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.InternalsConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.motorcontrol.MotorUtil;

public class InternalsSubsystem extends SubsystemBase {

    private final double ENTRANCE_THRESHOLD = .38;
    private final double STORAGE_THRESHOLD = .30;
    private final double STAGING_THRESHOLD = .123;

    private final double CONVEYOR_SPEED = .6;
    private final double FLYWHEEL_SPEED = .3;

    private enum InternalsState {
        NO_BALLS,
        MOVE_BALL_1_UP,
        ONE_BALL_STORAGE,
        MOVE_BALL_2_UP,
        TWO_BALLS
    }

    private InternalsState state;

    private WPI_TalonSRX conveyor;
    private CANSparkMax flywheelMain;
    private CANSparkMax flywheelFollow;

    private AnalogPotentiometer entranceIR;
    private AnalogPotentiometer storageIR;
    private AnalogPotentiometer exitIR;

    
    public InternalsSubsystem() {
        this.state = InternalsState.NO_BALLS;

        this.conveyor = MotorUtil.createTalonSRX(conveyorID);
        this.flywheelMain = MotorUtil.createSparkMax(flywheelMainID);
        this.flywheelFollow =  MotorUtil.createSparkMax(flywheelFollowID);

        this.conveyor.setInverted(true);
        this.flywheelMain.setInverted(true);
        flywheelFollow.follow(flywheelMain);
    
        this.entranceIR = new AnalogPotentiometer(entranceIRID);
        this.storageIR = new AnalogPotentiometer(storageIRID);
        this.exitIR = new AnalogPotentiometer(exitIRID);
    }

    @Override
    public void periodic() {

        
        flywheelMain.set(FLYWHEEL_SPEED);
        
        System.out.println(entranceIR.get());

        switch(state) {
            case NO_BALLS:
                conveyor.set(0);
                if (entranceIR.get() > ENTRANCE_THRESHOLD) {
                    System.out.println("ball at entrance");
                    state = InternalsState.MOVE_BALL_1_UP;
                }
                break;

            case MOVE_BALL_1_UP:
                System.out.println("MOVE_BALL_1_UP");
                conveyor.set(CONVEYOR_SPEED);
                if (storageIR.get() > STORAGE_THRESHOLD) {
                    System.out.println("ball at storage");
                    state = InternalsState.ONE_BALL_STORAGE;
                }
                break;

            case ONE_BALL_STORAGE:
                conveyor.set(0);
                if (entranceIR.get() > ENTRANCE_THRESHOLD) {
                    state = InternalsState.MOVE_BALL_2_UP;
                }
                break;

            case MOVE_BALL_2_UP:
                conveyor.set(CONVEYOR_SPEED);
                if (!(entranceIR.get() > ENTRANCE_THRESHOLD)) {
                    state = InternalsState.TWO_BALLS;   
                }
                break;
            case TWO_BALLS:
                break;
                
        }
        // SHOOTING balls
        // if two balls or ball 1 in storage
            // if shot requested
            // set flywheel speed
            // state = move ball 2 up

        // if exit ir detects then it doesn't, flywheel power to 0 
        
    }
    
}
