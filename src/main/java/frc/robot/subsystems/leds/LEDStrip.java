package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDStrip {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBuffer contLedBuffer;
    private final Timer ledTimer;

    public LEDStrip(int ledPort, int ledLength) {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledLength);
        contLedBuffer = new AddressableLEDBuffer(ledLength);

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

    public void updateContinuousColor(Color color){
        if(ledTimer.hasElapsed(0.05)){
            for (int i = contLedBuffer.getLength() - 1; i > 0; i--){
                contLedBuffer.setLED(i, contLedBuffer.getLED(i - 1));
            }
            contLedBuffer.setLED(0, color);
            ledTimer.reset();
            ledTimer.start();
        }
    }

    public void setContinuousColor(){
        led.setData(contLedBuffer);
    }

    public void fillContinuousColor(Color color){
        for(int i = 0; i < contLedBuffer.getLength(); i++){
            contLedBuffer.setLED(i, color);
        }
    }
}
