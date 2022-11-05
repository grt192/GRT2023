package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.InternalsConstants.*;

import java.security.KeyStore.Entry;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.motorcontrol.MotorUtil;
import frc.robot.shuffleboard.GRTNetworkTableEntry;
import frc.robot.shuffleboard.GRTShuffleboardTab;

public class InternalsSubsystem extends SubsystemBase {

    private final double ENTRANCE_THRESHOLD = .38;
    private final double STORAGE_THRESHOLD = .30;
    private final double STAGING_THRESHOLD = .34;
    private final double STAGING_THRESHOLD_SHOT = .31;

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
    private AnalogPotentiometer stagingIR;

    private boolean shotRequested;
    private boolean shotMade;
    private boolean stagingBall;

    private Timer exitTimer;

    private final GRTShuffleboardTab shuffleboardTab;
    private final GRTNetworkTableEntry entranceEntry, storageEntry, stagingEntry;
    private final GRTNetworkTableEntry stateEntry, shotRequestedEntry, stagingBallEntry;
    private final GRTNetworkTableEntry entranceRawEntry, storageRawEntry, stagingRawEntry;

    public InternalsSubsystem() {
        this.state = InternalsState.NO_BALLS;
        this.shotRequested = false;
        this.shotMade = false;
        this.stagingBall = false;

        this.exitTimer = new Timer();

        this.conveyor = MotorUtil.createTalonSRX(conveyorID);
        this.flywheelMain = MotorUtil.createSparkMax(flywheelMainID);
        this.flywheelFollow = MotorUtil.createSparkMax(flywheelFollowID);

        this.conveyor.setInverted(true);
        this.flywheelMain.setInverted(true);
        flywheelFollow.follow(flywheelMain);

        this.entranceIR = new AnalogPotentiometer(entranceIRID);
        this.storageIR = new AnalogPotentiometer(storageIRID);
        this.stagingIR = new AnalogPotentiometer(stagingIRID);

        // shuffleboard
        shuffleboardTab = new GRTShuffleboardTab("Internals");
        entranceEntry = shuffleboardTab.addEntry("Entrance", entranceIR.get() > ENTRANCE_THRESHOLD).at(0, 0);
        storageEntry = shuffleboardTab.addEntry("Storage", storageIR.get() > STORAGE_THRESHOLD).at(1, 0);
        stagingEntry = shuffleboardTab.addEntry("Staging", stagingIR.get() > STAGING_THRESHOLD).at(2, 0);

        stateEntry = shuffleboardTab.addEntry("STATE", state.toString()).at(0, 2);
        shotRequestedEntry = shuffleboardTab.addEntry("Shot requested", shotRequested).at(1, 2);
        stagingBallEntry = shuffleboardTab.addEntry("Staging ball ", stagingBall).at(1, 2);

        entranceRawEntry = shuffleboardTab.addEntry("Entrance raw", entranceIR.get()).at(0, 1);
        storageRawEntry = shuffleboardTab.addEntry("Storage raw", storageIR.get()).at(2, 1);
        stagingRawEntry = shuffleboardTab.addEntry("Staging raw", stagingIR.get()).at(4, 1);
    }

    @Override
    public void periodic() {

        // flywheelMain.set(FLYWHEEL_SPEED);

        switch (state) {
            case NO_BALLS:
                conveyor.set(0);
                if (entranceIR.get() > ENTRANCE_THRESHOLD) {
                    // System.out.println("ball at entrance");
                    state = InternalsState.MOVE_BALL_1_UP;
                }
                break;

            case MOVE_BALL_1_UP:
                // System.out.println("MOVE_BALL_1_UP");
                conveyor.set(CONVEYOR_SPEED);
                if (storageIR.get() > STORAGE_THRESHOLD) {
                    // System.out.println("ball at storage");
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
                
                if (stagingIR.get() > STAGING_THRESHOLD) { // && storageIR.get() > STORAGE_THRESHOLD) {
                    state = InternalsState.TWO_BALLS;
                }
                break;
            case TWO_BALLS:
                conveyor.set(0);

                break;

        }

        // System.out.println(stagingIR.get());
        // System.out.println(state);

        // System.out.println(shotRequested);

        if (shotRequested) {
            // System.out.println(state);
            // System.out.println(shotMade);

            // if ball in staging
            if (stagingIR.get() > STAGING_THRESHOLD) {
                exitTimer.reset();
                exitTimer.start();

                flywheelMain.set(FLYWHEEL_SPEED);
            }
            // no ball in staging
            else
            {
                if (state != InternalsState.NO_BALLS)
                {
                    conveyor.set(CONVEYOR_SPEED);
                }
            }
            
            // if exit time elapsed, mark shot as completed
            if (exitTimer.hasElapsed(0.5)) {
                exitTimer.stop();
                exitTimer.reset();

                flywheelMain.set(0);
                shotRequested = false;
                if (state == InternalsState.TWO_BALLS) {
                    state = InternalsState.ONE_BALL_STORAGE;
                }
                if (state == InternalsState.ONE_BALL_STORAGE) {
                    state = InternalsState.NO_BALLS;
                }
                
            }
        }

        entranceEntry.setValue(entranceIR.get() > ENTRANCE_THRESHOLD);
        storageEntry.setValue(storageIR.get() > STORAGE_THRESHOLD);
        stagingEntry.setValue(stagingIR.get() > STAGING_THRESHOLD);

        stateEntry.setValue(state.toString());
        shotRequestedEntry.setValue(shotRequested);
        stagingBallEntry.setValue(stagingBall);
        entranceRawEntry.setValue(entranceIR.get());
        storageRawEntry.setValue(storageIR.get());
        stagingRawEntry.setValue(stagingIR.get());
        ;

    }

    public void requestShot() {
        this.shotRequested = true;
    }

}
