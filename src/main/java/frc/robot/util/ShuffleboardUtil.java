package frc.robot.util;

import java.util.Optional;
import java.util.function.Consumer;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableValue;

public class ShuffleboardUtil {
    /**
     * Polls a Shuffleboard entry for a `double` change, updating the provided SparkMax PID controller's P gain if it is found.
     * @param entry The shuffleboard entry to poll.
     * @param pidController The PID controller to update.
     */
    public static void pollShuffleboardP(GenericEntry entry, SparkMaxPIDController pidController) {
        pollShuffleboardValue(entry, (value) -> pidController.setP(value.getDouble()));
    }

    /**
     * Polls a Shuffleboard entry for a `double` change, updating the provided SparkMax PID controller's I gain if it is found.
     * @param entry The shuffleboard entry to poll.
     * @param pidController The PID controller to update.
     */
    public static void pollShuffleboardI(GenericEntry entry, SparkMaxPIDController pidController) {
        pollShuffleboardValue(entry, (value) -> pidController.setI(value.getDouble()));
    }

    /**
     * Polls a Shuffleboard entry for a `double` change, updating the provided SparkMax PID controller's D gain if it is found.
     * @param entry The shuffleboard entry to poll.
     * @param pidController The PID controller to update.
     */
    public static void pollShuffleboardD(GenericEntry entry, SparkMaxPIDController pidController) {
        pollShuffleboardValue(entry, (value) -> pidController.setD(value.getDouble()));
    }

    /**
     * Polls a Shuffleboard entry for a `double` change, updating the provided SparkMax PID controller's FF gain if it is found.
     * @param entry The shuffleboard entry to poll.
     * @param pidController The PID controller to update.
     */
    public static void pollShuffleboardFF(GenericEntry entry, SparkMaxPIDController pidController) {
        pollShuffleboardValue(entry, (value) -> pidController.setFF(value.getDouble()));
    }

    /**
     * Polls a Shuffleboard entry for a `double` change, updating the provided SparkMax PID controller's
     * Smart Motion max velocity if it is found.
     * 
     * @param entry The shuffleboard entry to poll.
     * @param pidController The PID controller to update.
     */
    public static void pollShuffleboardMaxVel(GenericEntry entry, SparkMaxPIDController pidController) {
        pollShuffleboardValue(entry, (value) -> pidController.setSmartMotionMaxVelocity(value.getDouble(), 0));
    }

    /**
     * Polls a Shuffleboard entry for a `double` change, updating the provided SparkMax PID controller's
     * Smart Motion max acceleration if it is found.
     * 
     * @param entry The shuffleboard entry to poll.
     * @param pidController The PID controller to update.
     */
    public static void pollShuffleboardMaxAccel(GenericEntry entry, SparkMaxPIDController pidController) {
        pollShuffleboardValue(entry, (value) -> pidController.setSmartMotionMaxAccel(value.getDouble(), 0));
    }

    /**
     * Polls a Shuffleboard entry for a `double` change, updating the provided SparkMax PID controller's
     * SmartMotion allowed closed loop error if it is found.
     * 
     * @param entry The shuffleboard entry to poll.
     * @param pidController The PID controller to update.
     */
    public static void pollShuffleboardSmartMotionTolerance(GenericEntry entry, SparkMaxPIDController pidController) {
        pollShuffleboardValue(entry, (value) -> pidController.setSmartMotionAllowedClosedLoopError(value.getDouble(), 0));
    }

    /**
     * Polls a Shuffleboard entry for a `double` change, calling the provided callback if it is found.
     * @param entry The shuffleboard entry to poll.
     * @param callback The callback to run on a new `double` value.
     */
    private static void pollShuffleboardValue(GenericEntry entry, Consumer<NetworkTableValue> callback) {
        Optional<NetworkTableValue> update = getLatestUpdate(entry);
        update.ifPresent(callback);
    }

    /**
     * Reads a Shuffleboard entry's queue and returns the latest updated value, if it exists.
     * @param entry The entry to query.
     * @return The optional last updated value.
     */
    private static Optional<NetworkTableValue> getLatestUpdate(GenericEntry entry) {
        NetworkTableValue[] updates = entry.readQueue();
        if (updates.length == 0) return Optional.empty();
        return Optional.of(updates[updates.length - 1]); // TODO: make sure array is last-to-first
    }
}
