package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

public class LEDLayer {
    private final Color[] ledArray;

    public LEDLayer(int length) {
        ledArray = new Color[length];
    }

    /**
     * Sets an LED at a specified index.
     * @param i The LED index to set.
     * @param color The color to set the LED at index i to (null is equivalent to transparent).
     */
    public void setLED(int i, Color color) {
        ledArray[i] = color;
    }

    /**
     * Gets the color of an LED at a specified index.
     * @param i The LED index to retrieve.
     * @return The color of the LED at index i.
     */
    public Color getLED(int i) {
        return ledArray[i];
    }

    /**
     * Moves the leds up by an increment
     * @param inc the number of leds to move up by
     * @param color the color to set at the bottom
     */
    public void incrementColors(int inc, Color color) {
        for (int i = 0; i < ledArray.length - inc; i++) {
            setLED(i, getLED(i + inc));
        }
        for (int i = ledArray.length - inc; i < ledArray.length; i++) {
            setLED(i, color);
        }
    }

    /**
     * Fills the layer with a solid color.
     * @param color The color to fill the layer with.
     */
    public void fillColor(Color color) {
        for (int i = 0; i < ledArray.length; i++) {
            setLED(i, color);
        }
    }

    /**
     * Fills the layer with alternating groups of "on" and "off" LEDs. "off" leds are set to null (transparent).
     * @param onGroupLength The length of the "on" group.
     * @param offGroupLength The length of the "off" group.
     * @param color The color to set the "on" LEDs.
     */
    public void fillGrouped(int onGroupLength, int offGroupLength, Color color) {
        for (int i = 0; i < ledArray.length; i++) {
            if (i % (onGroupLength + offGroupLength) < onGroupLength) {
                setLED(i, color);
            } else {
                setLED(i, null);
            }
        }
    }

    /**
     * Resets the layer by setting all LEDs to null (transparent).
     */
    public void reset() {
        fillColor(null);
    }
}
