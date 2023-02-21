package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimeoutLEDSubsystem extends SubsystemBase {

    private LEDStrip ledStrip;

    private Timer timer;
    private double durationSeconds;

    private Color defaultColor;

    public TimeoutLEDSubsystem(LEDStrip ledStrip, double durationSeconds, Color defaultColor) {
        this.ledStrip = ledStrip;
        this.timer = new Timer();
        this.durationSeconds = durationSeconds;
        this.defaultColor = defaultColor;

        ledStrip.setSolidColor(defaultColor);

    }

    public void periodic() {
        if (timer.hasElapsed(durationSeconds)) {
            ledStrip.setSolidColor(defaultColor);
            timer.stop();
            timer.reset();
        }
    }

    public void setNewColor(Color color) {
        ledStrip.setSolidColor(color);
        timer.start();
    }
    
}
