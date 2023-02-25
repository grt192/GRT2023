package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LEDConstants.*;

public class LEDSubsystem extends SubsystemBase {
    private final LEDStrip ledStrip;

    private final Timer blinkTimer;
    private static final double BLINK_DURATION_SECONDS = 1.0;
    private static final Color BLINK_COLOR = new Color(0, 255, 0);

    private Color color = Color.kFirstRed;
    private boolean blinking = false;

    public boolean pieceGrabbed = false;

    public LEDSubsystem() {
        ledStrip = SIGNAL_LED_STRIP;
        blinkTimer = new Timer();
    }

    @Override
    public void periodic() {
        // Start blink timer loop if we are holding a piece
        if (pieceGrabbed) {
            blinkTimer.start();
        } else {
            blinking = false;
            blinkTimer.stop();
            blinkTimer.reset();
        }

        // Toggle the blink boolean every duration to swap the LEDs between the driver piece color
        // and the blink color.
        if (blinkTimer.advanceIfElapsed(BLINK_DURATION_SECONDS)) blinking = !blinking;
        ledStrip.setSolidColor(blinking ? BLINK_COLOR : color);
    }

    public void setColor(Color color) {
        this.color = color;
    }
}
