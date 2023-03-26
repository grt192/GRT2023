package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDStrip {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBuffer overlayBuffer;

    private final Timer ledTimer;

    private static final int LEDS_PER_SEC = 150;
    private static final int LED_GROUP_LENGTH = 5;

    public LEDStrip(int ledPort, int ledLength) {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledLength);
        overlayBuffer = new AddressableLEDBuffer(ledLength);

        led.setLength(ledBuffer.getLength());
        led.start();

        ledTimer = new Timer();
        ledTimer.start();
    }

    //increment the colors in the cont buffer, does not set the leds
    public void updateContinuousColor(Color color) {
        int inc = Math.min((int) Math.ceil(ledTimer.get() * LEDS_PER_SEC), ledBuffer.getLength());

        for (int i = 0; i < ledBuffer.getLength() - inc; i++){
            ledBuffer.setLED(i, ledBuffer.getLED(i + inc));
        }
        for(int i = ledBuffer.getLength() - inc; i < ledBuffer.getLength(); i++){
            ledBuffer.setLED(i, color);
        }
        ledTimer.reset();
        ledTimer.start();
    }

    /**
     * Sets the LED data to the current buffer.
     */
    public void setBuffer() {
        led.setData(ledBuffer);
    }

    /**
     * Sets the LED data to the overlay buffer
     */
    public void setOverlay() {
        led.setData(overlayBuffer);
    }

    /**
     * Fills the LED buffer with a solid color.
     * @param color The color to fill the buffer with.
     */
    public void fillSolidColor(Color color) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
    }

    /**
     * Fills the LED buffer with a solid color, excluding pixels of the provided ignore color.
     * @param color The color to fill the buffer with.
     * @param ignoreColor The color to ignore when filling.
     */
    public void fillSolidColorIgnoringColor(Color color, Color ignoreColor) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (ledBuffer.getLED(i).equals(ignoreColor)) continue;
            ledBuffer.setLED(i, color);
        }
    }

    /**
     * Fills the LED buffer with alternating groups of two colors.
     * @param color1 The first color to set.
     * @param color2 The second color to set.
     */
    public void fillGroupedColors(Color color1, Color color2) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i % (LED_GROUP_LENGTH * 2) < LED_GROUP_LENGTH){
                ledBuffer.setLED(i, color1);
            } else {
                ledBuffer.setLED(i, color2);
            }
        }
    }
    
    /**
     * Copies the ledBuffer to the overlayBuffer
     * this is done so that the overlay buffer does not affect the continuous led mode
     */
    public void copyToOverlay(){
        for (int i = 0; i < ledBuffer.getLength(); i++){
            overlayBuffer.setLED(i, ledBuffer.getLED(i));
        }
    }

    /**
     * Fills LED buffer with a color that is in groups, the on to off ratio is
     * onGroupLength:offGroupLength
     * @param color The color you are filling
     * @param onGroupLength The length of leds to be turned to color
     * @param offGroupLength The length of leds to be skipped
     */
    public void fillGroupedWithBlanks(Color color, int onGroupLength, int offGroupLength) {
        copyToOverlay();
        for (int i = 0; i < overlayBuffer.getLength(); i++) {
            if (i % (onGroupLength + offGroupLength) < onGroupLength){
                overlayBuffer.setLED(i, color);
            }
        }
    }
}
