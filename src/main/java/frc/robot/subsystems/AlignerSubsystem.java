package frc.robot.subsystems;

import static frc.robot.Constants.AlignerConstants.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorcontrol.MotorUtil;
import frc.robot.shuffleboard.GRTNetworkTableEntry;
import frc.robot.shuffleboard.GRTShuffleboardTab;

public class AlignerSubsystem extends SubsystemBase {
    //manual pid state
    public int motorrange = 6000;
    // current target position for motors
    public double current_slaptarget = OPENSLAP;
    public double current_anglertarget = OPENANGLER;
    // current action state
    public boolean to_slap;
    public boolean right_pressed = false;
    // state of angler(open or closed)
    public boolean angler_toclose = false;
    public boolean tozero = false;

    public double speed;
    public double speed2;

    private final WPI_TalonSRX motorSlapper = MotorUtil.createTalonSRX(SLAPID);
    private final WPI_TalonSRX motorAngler = MotorUtil.createTalonSRX(ANGLEID);

    private final GRTShuffleboardTab shuffleboardTab = new GRTShuffleboardTab("Aligner");
    private final GRTNetworkTableEntry slapperPositionEntry = shuffleboardTab.addEntry("Slapperposition", 0).at(0, 0);
    private final GRTNetworkTableEntry anglerPositionEntry = shuffleboardTab.addEntry("Anglerposition", 0).at(1, 0);

    // variables for motor positions
    public double slappos;
    public double anglerpos;

    public AlignerSubsystem() {
        // set motors to brake
        motorSlapper.setNeutralMode(NeutralMode.Brake);
        motorAngler.setNeutralMode(NeutralMode.Brake);

        motorSlapper.setInverted(true);

        // setup encoders for motors
        motorSlapper.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        motorSlapper.setSensorPhase(false);
        motorSlapper.setSelectedSensorPosition(0);
        //open limit
        motorSlapper.configReverseSoftLimitThreshold(0, 0);
        motorSlapper.configReverseSoftLimitEnable(true, 0);
        //closed limit   
        motorSlapper.configForwardSoftLimitThreshold(SLAPSLAP, 0);
        motorSlapper.configForwardSoftLimitEnable(true, 0);

        motorAngler.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        motorAngler.setSensorPhase(true);
        motorAngler.setSelectedSensorPosition(0);
        //open limit
        motorAngler.configForwardSoftLimitThreshold(0, 0);
        motorAngler.configForwardSoftLimitEnable(true, 0);
        //closed limit
        motorAngler.configReverseSoftLimitThreshold(CLOSEDANGLER, 0);
        motorAngler.configReverseSoftLimitEnable(true, 0);

    }
    // as of now:
    // motor can potentially get stuck going back and forth depending on speed
    // no code for catching/fixing motor over-turning

    public void periodic() {

        motorSlapper.set(speed);
        motorAngler.set(speed2);
    
        // get current motor positions
        slappos = motorSlapper.getSelectedSensorPosition();
        anglerpos = motorAngler.getSelectedSensorPosition();

        if (angler_toclose) {
            current_slaptarget = CLOSEDSLAP;
            current_anglertarget = CLOSEDANGLER;
        }
        // else check if slapper trigger is pressed
        else if (to_slap) {
                current_slaptarget = SLAPSLAP;
            }

        else if (tozero){
            current_slaptarget = 0;
            current_anglertarget = 0;
        }
            
        // else return to open state
        else {
            current_slaptarget = OPENSLAP;
            current_anglertarget = OPENANGLER;
        }
        System.out.println(current_slaptarget);

        // get slapper to current target
        if (Math.abs(slappos - current_slaptarget) <= motorrange) {
            motorSlapper.set(0); // move left
        } else if (slappos < current_slaptarget) {
             motorSlapper.set(.2); // move right
        } else if (slappos > current_slaptarget){
            motorSlapper.set(-.2);
        }

        // get angler to current target
        if (Math.abs(anglerpos - current_anglertarget) <= motorrange ) {
            motorAngler.set(0);
        } else if (anglerpos < current_anglertarget) {
            motorAngler.set(.3); // move right
        } else if (anglerpos > current_anglertarget){
            motorAngler.set(-.3);
        }

        slapperPositionEntry.setValue(motorSlapper.getSelectedSensorPosition());
        anglerPositionEntry.setValue(motorAngler.getSelectedSensorPosition());

    }

}
