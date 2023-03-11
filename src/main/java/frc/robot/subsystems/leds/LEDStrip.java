package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDStrip {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final Timer ledTimer;

    public LEDStrip(int ledPort, int ledLength) {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledLength);

        led.setLength(ledBuffer.getLength());
        led.start();
        ledTimer = new Timer();
        ledTimer.start();
    }

    /**
     * Sets the color of this LED strip to a provided solid color.
     * @param color The color to set the LED strip to.
     */
    public void setSolidColor(Color color) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        led.setData(ledBuffer);
    }

    public void setContinuousColor(Color color){
        if(ledTimer.hasElapsed(0.1)){
            for (int i = ledBuffer.getLength() - 1; i > 0; i--){
                ledBuffer.setLED(i, ledBuffer.getLED(i - 1));
            }
            ledBuffer.setLED(0, color);
            led.setData(ledBuffer);
            ledTimer.reset();
            ledTimer.start();
        }
    }
}
