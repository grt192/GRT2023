package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDStrip {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final Timer ledTimer;
    private final int LEDS_PER_SEC = 150;
    private final int LED_GROUP_LENGTH = 5;

    public LEDStrip(int ledPort, int ledLength) {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledLength);

        led.setLength(ledBuffer.getLength());
        led.start();
        ledTimer = new Timer();
        ledTimer.start();
    }


    //increment the colors in the cont buffer, does not set the leds
    public void updateContinuousColor(Color color){
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

    //set cont buffer on to the leds
    public void setContinuousColor(){
        led.setData(ledBuffer);
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
        for(int i = 0; i < ledBuffer.getLength(); i++){
            ledBuffer.setLED(i, color);
        }
    }

    public void fillContinuousColorIgnoringOneColor(Color color, Color ignoreColor){
        for(int i = 0; i < ledBuffer.getLength(); i++){
            if(!ignoreColor.equals(ledBuffer.getLED(i))){
                ledBuffer.setLED(i, color);
            }
        }
    }
}
