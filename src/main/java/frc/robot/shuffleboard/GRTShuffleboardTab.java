package frc.robot.shuffleboard;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * A utility class to wrap common shuffleboard patterns. Contains methods for adding listener
 * and regular network table entries for reading and displaying data on Shuffleboard.
 */
public class GRTShuffleboardTab {
    private final ShuffleboardTab shuffleboardTab;

    public GRTShuffleboardTab(String name) {
        shuffleboardTab = Shuffleboard.getTab(name);
    }

    /**
     * Groups entries into a list, returning the GRTShuffleboardLayout created.
     * @param name The name of the list.
     * @return The created GRTShuffleboardLayout.
     */
    public GRTShuffleboardLayout list(String name) {
        return new GRTShuffleboardLayout(shuffleboardTab, name, BuiltInLayouts.kList);
    }

    /**
     * Adds a network table shuffleboard entry to the tab with the supplied name and value.
     * This is typically for displaying data. For reading data, see `addListener`.
     * 
     * @param name The name of the entry.
     * @param value The initial value of the entry.
     * @return The created GRTNetworkTableEntry.
     */
    public GRTNetworkTableEntry addEntry(String name, Object value) {
        return new GRTNetworkTableEntry(shuffleboardTab, name, value);
    }

    /**
     * Adds a listener shuffleboard entry to the tab with the supplied name and value.
     * The entry will call the callback according to the set flags (defaults to every value update).
     * Returns this tab for call chaining.
     * 
     * @param name The name of the entry.
     * @param value The initial value of the entry.
     * @param callback The callback for when the value changes.
     * @param flags The flags controlling when to call the callback. Defaults to `kUpdate`.
     * @return The shuffleboard tab, for call chaining.
     */
    public GRTShuffleboardTab addListener(String name, Object value, Consumer<EntryNotification> callback, int flags) {
        shuffleboardTab.add(name, value).getEntry().addListener(callback, flags);
        return this;
    }

    public GRTShuffleboardTab addListener(String name, Object value, Consumer<EntryNotification> callback) {
        return addListener(name, value, callback, EntryListenerFlags.kUpdate);
    }

    public GRTShuffleboardTab addListener(String name, Object value, Consumer<EntryNotification> callback, int col, int row, int flags) {
        shuffleboardTab.add(name, value)
            .withPosition(col, row).getEntry()
            .addListener(callback, flags);
        return this;
    }

    public GRTShuffleboardTab addListener(String name, Object value, Consumer<EntryNotification> callback, int col, int row) {
        return addListener(name, value, callback, col, row, EntryListenerFlags.kUpdate);
    }

    /**
     * Adds a boolean listener shuffleboard entry to the tab, displayed as a toggle.
     * 
     * @param name The name of the toggle.
     * @param value The initial value of the toggle.
     * @param callback The callback for when the switch is toggled.
     * @param flags The flags controlling when to call the callback.
     * @return The shuffleboard tab, for call chaining.
     */
    public GRTShuffleboardTab addToggle(String name, boolean value, Consumer<EntryNotification> callback, int flags) {
        shuffleboardTab.add(name, value)
            .withWidget(BuiltInWidgets.kToggleSwitch).getEntry()
            .addListener(callback, flags);
        return this;
    }

    public GRTShuffleboardTab addToggle(String name, boolean value, Consumer<EntryNotification> callback) {
        return addToggle(name, value, callback, EntryListenerFlags.kUpdate);
    }

    public GRTShuffleboardTab addToggle(String name, boolean value, Consumer<EntryNotification> callback, int col, int row, int flags) {
        shuffleboardTab.add(name, value)
            .withPosition(col, row)
            .withWidget(BuiltInWidgets.kToggleSwitch).getEntry()
            .addListener(callback, flags);
        return this;
    }

    public GRTShuffleboardTab addToggle(String name, boolean value, Consumer<EntryNotification> callback, int col, int row) {
        return addToggle(name, value, callback, col, row, EntryListenerFlags.kUpdate);
    }

    /**
     * Adds a complex widget to the shuffleboard tab. This is used for things like the
     * field, and buttons that run commands.
     * 
     * @param name The name of the widget.
     * @param widget The Sendable widget.
     * @return The ComplexWidget object representing the sendable on shuffleboard.
     */
    public GRTShuffleboardTab addWidget(String name, Sendable widget) {
        shuffleboardTab.add(name, widget);
        return this;
    }

    public GRTShuffleboardTab addWidget(String name, Sendable widget, int col, int row) {
        shuffleboardTab.add(name, widget).withPosition(col, row);
        return this;
    }
}
