package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
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
    private final double STAGING_THRESHOLD = .32;

    private final double CONVEYOR_SPEED = .6;
    private double flywheelSpeed;

    private double periodicFlywheelSpeed;
    private double periodicConveyorSpeed;

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
    private boolean stagingBall;
    private boolean isShooting;

    private Timer exitTimer;

    private final GRTShuffleboardTab shuffleboardTab;
    private final GRTNetworkTableEntry entranceEntry, storageEntry, stagingEntry;
    private final GRTNetworkTableEntry stateEntry, shotRequestedEntry, stagingBallEntry;
    private final GRTNetworkTableEntry entranceRawEntry, storageRawEntry, stagingRawEntry;
    private final GRTNetworkTableEntry exitTimerEntry;
    private final GRTNetworkTableEntry flywheelEntry;

    public InternalsSubsystem() {
        this.state = InternalsState.NO_BALLS;
        this.shotRequested = false;
        this.stagingBall = false;

        this.flywheelSpeed = 0.8; // starting flywheel speed

        this.periodicFlywheelSpeed = 0;
        this.periodicConveyorSpeed = 0;

        this.isShooting = false;

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
        stagingBallEntry = shuffleboardTab.addEntry("Staging ball ", stagingBall).at(2, 2);

        entranceRawEntry = shuffleboardTab.addEntry("Entrance raw", entranceIR.get()).at(0, 1);
        storageRawEntry = shuffleboardTab.addEntry("Storage raw", storageIR.get()).at(2, 1);
        stagingRawEntry = shuffleboardTab.addEntry("Staging raw", stagingIR.get()).at(4, 1);

        exitTimerEntry = shuffleboardTab.addEntry("exit timer", this.exitTimer.get()).at(1, 3);

        flywheelEntry = shuffleboardTab.addEntry("flywheel power", flywheelSpeed);
    }

    @Override
    public void periodic() {

        // flywheelMain.set(flywheelSpeed);

        flywheelMain.set(periodicFlywheelSpeed);
        conveyor.set(periodicConveyorSpeed);

        switch (state) {
            case NO_BALLS:
                periodicConveyorSpeed = 0;

                if (entranceIR.get() > ENTRANCE_THRESHOLD) {
                    // System.out.println("ball at entrance");
                    state = InternalsState.MOVE_BALL_1_UP;
                }
                break;

            case MOVE_BALL_1_UP:
                // System.out.println("MOVE_BALL_1_UP");
                periodicConveyorSpeed = CONVEYOR_SPEED;
                if (storageIR.get() > STORAGE_THRESHOLD) {
                    // System.out.println("ball at storage");
                    state = InternalsState.ONE_BALL_STORAGE;
                }
                break;

            case ONE_BALL_STORAGE:
                periodicConveyorSpeed = 0;
                if (entranceIR.get() > ENTRANCE_THRESHOLD) {
                    state = InternalsState.MOVE_BALL_2_UP;
                }
                break;

            case MOVE_BALL_2_UP:
                periodicConveyorSpeed = CONVEYOR_SPEED;

                if (stagingIR.get() > STAGING_THRESHOLD) { // && storageIR.get() > STORAGE_THRESHOLD) {
                    state = InternalsState.TWO_BALLS;
                }
                break;
            case TWO_BALLS:
                periodicConveyorSpeed = 0;
                break;

        }

        // System.out.println(stagingIR.get());
        // System.out.println(state);

        // System.out.println(shotRequested);

        if (shotRequested) {
            // System.out.println(state);
            // System.out.println(shotMade);

            // if ball in staging and we aren't shooting yet, initiate shoot sequence
            // if (stagingIR.get() > STAGING_THRESHOLD) {
            if (state == InternalsState.ONE_BALL_STORAGE || state != InternalsState.TWO_BALLS) {

                if (!isShooting) {

                    exitTimer.reset();
                    exitTimer.start();

                    isShooting = true;

                    periodicConveyorSpeed = 0;
                    periodicFlywheelSpeed = flywheelSpeed;
                }
            }
            /*
             * // no ball in staging
             * else {
             * periodicFlywheelSpeed = 0;
             * 
             * if (state != InternalsState.NO_BALLS) {
             * periodicConveyorSpeed = CONVEYOR_SPEED;
             * }
             * else {
             * periodicConveyorSpeed = 0;
             * }
             * }
             */

            // if flywheel up to speed + conveyor was run, mark shot as completed
            if (exitTimer.hasElapsed(1.8)) {
                exitTimer.stop();
                exitTimer.reset();

                periodicConveyorSpeed = 0;
                periodicFlywheelSpeed = 0;

                shotRequested = false;
                isShooting = false;
                if (state == InternalsState.TWO_BALLS) {
                    state = InternalsState.ONE_BALL_STORAGE;
                } else if (state == InternalsState.ONE_BALL_STORAGE) {
                    state = InternalsState.NO_BALLS;
                }

            } else if (exitTimer.hasElapsed(1.0)) {
                periodicConveyorSpeed = CONVEYOR_SPEED;
                periodicFlywheelSpeed = flywheelSpeed;
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

        exitTimerEntry.setValue(exitTimer.get());

        flywheelEntry.setValue(flywheelSpeed);

    }

    public void setFlywheelPower(double power) {
        this.flywheelSpeed = power;
    }

    public double getFlywheelPower() {
        return flywheelSpeed;
    }

    public void requestShot() {
        this.shotRequested = true;
    }

    public void resetState() {
        this.state = InternalsState.NO_BALLS;
    }

}
