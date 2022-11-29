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

    public boolean to_open;
    public boolean to_slap;
    public boolean to_grab;

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
        to_open = false;
        
        // get current motor positions
        slappos = motorSlapper.getSelectedSensorPosition();
        anglerpos = motorAngler.getSelectedSensorPosition();

        // check if slapper trigger is pressed (and angler is not pressed)
        if (to_slap) {
            // move slapper to target position
            if (slappos < SLAPSLAP) {
                motorSlapper.set(.5); // turn motor on ; potentially pid
            }
            // else keep slapper at target position
            else {
                motorSlapper.set(0);
            }

        }

        // check if both triggers are pressed
        else if (to_grab) {
            // if slap trigger was already held down
            if (slappos > CLOSEDSLAP) {
                motorSlapper.set(-.5); // move left
            }
            // else if slap trigger was not already held down
            else if (slappos < CLOSEDSLAP) {
                motorSlapper.set(.5); // move right
            }
            // else keep slapper at target position
            else {
                motorSlapper.set(0);
            }

            // move angler to target closed position
            if (anglerpos > CLOSEDANGLER) {
                motorAngler.set(-.5);
            }
            // else keep angler at target position
            else {
                motorAngler.set(0);
            }

        }
        // else return to open state
        else {
            to_open = true;
        }
        // if position back to open is true
        if (to_open) {
            // if slapper is not at open position, get there
            if (slappos > OPENSLAP) {
                motorSlapper.set(-.5);
            }
            // else keep slapper at target position
            else {
                motorSlapper.set(0);
            }
            // if angler is not at open position get there
            if (anglerpos < OPENANGLER) {
                motorAngler.set(.5);
            }
            // else keep angler at target position
            else {
                motorAngler.set(0);
            }
        }

        slapperPositionEntry.setValue(motorSlapper.getSelectedSensorPosition());
        anglerPositionEntry.setValue(motorAngler.getSelectedSensorPosition());

    }

}
