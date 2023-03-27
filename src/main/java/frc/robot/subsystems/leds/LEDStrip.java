package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

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
     * Add a layer to the ledBuffer
     * @param layer the layer to be added ontop
     * null leds are considered a transparent led
     */
    public void addLayer(LEDLayer layer) {
        for(int i = 0; i < ledLength; i++){
            if(layer.getLED(i) != null){
                ledBuffer.setLED(i, layer.getLED(i));
            }
        }
    }
}
