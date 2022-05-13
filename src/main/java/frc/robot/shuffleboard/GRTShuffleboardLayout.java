package frc.robot.shuffleboard;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * A utility class to wrap common shuffleboard patterns. Contains methods for adding listener
 * and regular network table entries for reading and displaying data on Shuffleboard.
 */
public class GRTShuffleboardLayout {
    private final ShuffleboardLayout shuffleboardLayout;

    public GRTShuffleboardLayout(ShuffleboardTab shuffleboardTab, String name, BuiltInLayouts type) {
        shuffleboardLayout = shuffleboardTab.getLayout(name, type);
    }

    /**
     * Positions this layout at the specified column and row.
     * 
     * @param col The column of the top left cell of the layout.
     * @param row The row of the top left cell of the layout.
     * @return The shuffleboard layout, for call chaining.
     */
    public GRTShuffleboardLayout at(int col, int row) {
        shuffleboardLayout.withPosition(col, row);
        return this;
    }

    /**
     * Sizes this layout to the specified width and height.
     * 
     * @param width The width of the layout.
     * @param height The height of the layout.
     * @return The shuffleboard layout, for call chaining.
     */
    public GRTShuffleboardLayout withSize(int width, int height) {
        shuffleboardLayout.withSize(width, height);
        return this;
    }

    /**
     * Adds a network table shuffleboard entry to the layout with the supplied name and value.
     * This is typically for displaying data. For reading data, see `addListener`.
     * 
     * @param name The name of the entry.
     * @param value The initial value of the entry.
     * @return The created GRTNetworkTableEntry.
     */
    public GRTNetworkTableEntry addEntry(String name, Object value) {
        return new GRTNetworkTableEntry(shuffleboardLayout, name, value);
    }

    /**
     * Adds a listener shuffleboard entry to the layout with the supplied name and value.
     * The entry will call the callback according to the set flags (defaults to every value update).
     * Returns this layout for call chaining.
     * 
     * @param name The name of the entry.
     * @param value The initial value of the entry.
     * @param callback The callback for when the value changes.
     * @param flags The flags controlling when to call the callback. Defaults to `kUpdate`.
     * @return The shuffleboard layout, for call chaining.
     */
    public GRTShuffleboardLayout addListener(String name, Object value, Consumer<EntryNotification> callback, int flags) {
        shuffleboardLayout.add(name, value).getEntry().addListener(callback, flags);
        return this;
    }

    public GRTShuffleboardLayout addListener(String name, Object value, Consumer<EntryNotification> callback) {
        return addListener(name, value, callback, EntryListenerFlags.kUpdate);
    }

    /**
     * Adds a complex widget to the shuffleboard layout. This is used for things like the
     * field, and buttons that run commands.
     * 
     * @param name The name of the widget.
     * @param widget The Sendable widget.
     * @return The shuffleboard layout, for call chaining.
     */
    public GRTShuffleboardLayout addWidget(String name, Sendable widget) {
        shuffleboardLayout.add(name, widget);
        return this;
    }
}
