package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LEDConstants.*;

public class LEDSubsystem extends SubsystemBase {
    private final LEDStrip ledStrip;

    private final Timer timer;
    private static final double BLINK_DURATION_SECONDS = 1.0;
    private static final Color BLINK_COLOR = new Color(0, 255, 0);

    private Color color = Color.kFirstRed;

    public LEDSubsystem() {
        ledStrip = SIGNAL_LED_STRIP;
        timer = new Timer();
    }

    @Override
    public void periodic() {
        ledStrip.setSolidColor(color);

        // if (timer.hasElapsed(durationSeconds)) {
            // ledStrip.setSolidColor(defaultColor);
            // timer.stop();
            // timer.reset();
        // }
    }

    public void setColor(Color color) {
        this.color = color;
    }
}
