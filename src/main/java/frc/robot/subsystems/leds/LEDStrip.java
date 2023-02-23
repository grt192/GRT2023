package frc.robot.subsystems.leds;

import java.awt.Color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDStrip {

    private final int ledPort;
    private final int ledLength;

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public LEDStrip(int ledPort, int ledLength) {
        this.ledPort = ledPort;
        this.ledLength = ledLength;

        this.led = new AddressableLED(ledPort);
        this.ledBuffer = new AddressableLEDBuffer(ledLength);
        this.led.setLength(ledBuffer.getLength());
    }

    public void setSolidColor(Color color) {
        for (int i = 0; i < ledLength; i++) {
            ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
        }
        led.setData(ledBuffer);
        led.start();
    }
}
