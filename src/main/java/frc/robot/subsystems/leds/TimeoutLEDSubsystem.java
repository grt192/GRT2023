package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimeoutLEDSubsystem extends SubsystemBase {
    private final LEDStrip ledStrip;

    private final Timer timer;
    private final double durationSeconds;

    private final Color defaultColor;

    public TimeoutLEDSubsystem(LEDStrip ledStrip, double durationSeconds, Color defaultColor) {
        this.ledStrip = ledStrip;
        this.timer = new Timer();
        this.durationSeconds = durationSeconds;
        this.defaultColor = defaultColor;

        ledStrip.setSolidColor(defaultColor);
    }

    public void periodic() {
        // ledStrip.setSolidColor(defaultColor);

        // if (timer.hasElapsed(durationSeconds)) {
            // ledStrip.setSolidColor(defaultColor);
            // timer.stop();
            // timer.reset();
        // }
    }

    public void setNewColor(Color color) {
        System.out.println("seting new color");
        ledStrip.setSolidColor(color);
        // timer.start();
    }
}
