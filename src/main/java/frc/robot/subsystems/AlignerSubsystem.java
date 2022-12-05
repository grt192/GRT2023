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
    // current target position for motors
    public double current_slaptarget = OPENSLAP;
    public double current_anglertarget = OPENANGLER;
    // current action state
    public boolean to_slap;
    public boolean right_pressed;
    // state of angler(open or closed)
    public boolean angler_open = true;

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

        // setup encoders for motors
        motorSlapper.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        motorSlapper.setSensorPhase(true);

        motorAngler.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        motorAngler.setSensorPhase(true);

    }
    // as of now:
    // motor can potentially get stuck going back and forth depending on speed
    // no code for catching/fixing motor over-turning

    public void periodic() {

        // get current motor positions
        slappos = motorSlapper.getSelectedSensorPosition();
        anglerpos = motorAngler.getSelectedSensorPosition();

        // check if slapper trigger is pressed
        if (to_slap) {
            current_slaptarget = SLAPSLAP;
        }
        // check if grab trigger is pressed and angler is open
        else if (right_pressed && angler_open) {
            current_slaptarget = CLOSEDSLAP;
            current_anglertarget = CLOSEDANGLER;
        }
        // check if grab trigger is pressed and angler is closed
        else if (right_pressed && !angler_open) {
            current_slaptarget = OPENSLAP;
            current_anglertarget = OPENANGLER;
        }
        // else return to open state
        else {
            current_slaptarget = OPENSLAP;
            current_anglertarget = OPENANGLER;
        }

        // get slapper to current target
        if (slappos > current_slaptarget) {
            motorSlapper.set(-.5); // move left
        } else if (slappos < current_slaptarget) {
            motorSlapper.set(.5); // move right
        } else {
            motorSlapper.set(0);
        }

        // get angler to current target
        if (anglerpos > current_anglertarget) {
            motorAngler.set(-.5); // move left
        } else if (anglerpos < current_anglertarget) {
            motorAngler.set(.5); // move right
        } else {
            motorAngler.set(0);
        }

        slapperPositionEntry.setValue(motorSlapper.getSelectedSensorPosition());
        anglerPositionEntry.setValue(motorAngler.getSelectedSensorPosition());

    }

}