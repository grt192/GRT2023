package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

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
            ledBuffer.setRGB(i, (int) color.red, (int) color.green, (int) color.blue);
        }
        led.setData(ledBuffer);
        led.start();
    }
}
