package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDStrip {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private int ledLength;

    public LEDStrip(int ledPort, int ledLength) {
        this.ledLength = ledLength;
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledLength);
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    /**
     * Sets the LED data to the current buffer.
     */
    public void setBuffer() {
        led.setData(ledBuffer);
    }

    /**
     * Applies an `LEDLayer` on top of the LED buffer.
     * @param layer The layer to be added on top of the buffer. Null leds are considered transparent and "fall through" to the previous layer's color.
     */
    public void addLayer(LEDLayer layer) {
        for (int i = 0; i < ledLength; i++) {
            if (layer.getLED(i) != null) {
                ledBuffer.setLED(i, layer.getLED(i));
            }
        }
    }
}
