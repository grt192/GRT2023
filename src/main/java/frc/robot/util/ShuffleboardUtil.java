package frc.robot.util;

import java.util.EnumSet;
import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

public class ShuffleboardUtil {
    private static final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

    /**
     * Sets up a listener on a Shuffleboard entry for a `double` change, calling the provided callback on new values.
     * @param entry The shuffleboard entry to poll.
     * @param callback The callback to run on a new `double` value.
     */
    public static void addDoubleListener(GenericEntry entry, Consumer<Double> callback) {
        addValueListener(entry, (value) -> callback.accept(value.getDouble()));
    }

    /**
     * Sets up a listener on a Shuffleboard entry for a value change, calling the provided callback on new values.
     * @param entry The shuffleboard entry to poll.
     * @param callback The callback to run on a new `NetworkTableValue` value.
     */
    public static void addValueListener(GenericEntry entry, Consumer<NetworkTableValue> callback) {
        networkTableInstance.addListener(
            entry, 
            EnumSet.of(NetworkTableEvent.Kind.kImmediate, NetworkTableEvent.Kind.kValueAll), 
            (event) -> callback.accept(event.valueData.value)
        );
    }

    /**
     * Polls a Shuffleboard entry for a `double` change, calling the provided callback if it is found.
     * @param entry The shuffleboard entry to poll.
     * @param callback The callback to run on a new `double` value.
     */
    public static void pollShuffleboardDouble(GenericEntry entry, Consumer<Double> callback) {
        pollShuffleboardValue(entry, (value) -> callback.accept(value.getDouble()));
    }

    /**
     * Polls a Shuffleboard entry for a value change, calling the provided callback if it is found.
     * @param entry The shuffleboard entry to poll.
     * @param callback The callback to run on a new `NetworkTableValue` value.
     */
    public static void pollShuffleboardValue(GenericEntry entry, Consumer<NetworkTableValue> callback) {
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
        return Optional.of(updates[updates.length - 1]);
    }
}
