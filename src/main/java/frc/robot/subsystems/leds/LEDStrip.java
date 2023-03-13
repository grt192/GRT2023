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
    private final int LEDS_PER_SEC = 60;
    private final int LED_GROUP_LENGTH = 5;

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

    //increment the colors in the cont buffer, does not set the leds
    public void updateContinuousColor(Color color){
        int inc = (int) Math.ceil(ledTimer.get() * LEDS_PER_SEC);
        inc = Math.max(1, inc);
        for (int i = contLedBuffer.getLength() - inc; i > 0; i--){
            contLedBuffer.setLED(i, contLedBuffer.getLED(i - inc));
        }
        for(int i = 0; i < inc; i++){
            contLedBuffer.setLED(i, color);
        }
        ledTimer.reset();
        ledTimer.start();
        
    }

    //set cont buffer on to the leds
    public void setContinuousColor(){
        led.setData(contLedBuffer);
    }

    public void setTwoColors(Color color1, Color color2){
        for(int i = 0; i < ledBuffer.getLength(); i++){
            if(i % (LED_GROUP_LENGTH * 2) < LED_GROUP_LENGTH){
                ledBuffer.setLED(i, color1);
            } else {
                ledBuffer.setLED(i, color2);
            }
        }
        led.setData(ledBuffer);
    }

    //set entire cont buffer to a single color
    //does the same thing as set solid color except the continuous buffer is reset
    public void fillContinuousColor(Color color){
        for(int i = 0; i < contLedBuffer.getLength(); i++){
            contLedBuffer.setLED(i, color);
        }
    }
}
