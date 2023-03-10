package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LEDConstants.*;

public class LEDSubsystem extends SubsystemBase {
    private final LEDStrip ledStrip;

    private final Timer blinkTimer;
    private static final double BLINK_DURATION_SECONDS = 0.5;
    private static final Color BLINK_COLOR = new Color(0, 0, 0);
    private boolean blinking = false;

    private static final double BRIGHTNESS_SCALE_FACTOR = 0.25;

    private Color color = new Color(192, 8, 254);
    public boolean pieceGrabbed = false;

    public LEDSubsystem() {
        ledStrip = new LEDStrip(LED_PWM_PORT, LED_LENGTH);
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

    /**
     * Sets the driver-provided color of the LEDs.
     * @param red The red component of the color.
     * @param green The green component of the color.
     * @param blue The blue component of the color.
     */
    public void setColor(double red, double green, double blue) {
        this.color = new Color(
            (int) (red * BRIGHTNESS_SCALE_FACTOR),
            (int) (green * BRIGHTNESS_SCALE_FACTOR),
            (int) (blue * BRIGHTNESS_SCALE_FACTOR)
        );
        
    }

    public void setColorHSV(double h, double s, double v){
        this.color = Color.fromHSV((int)h, (int)s, (int)v);
    }

    public static double getJoystickAngleAsPercent(double x, double y){
        double angle = Math.atan(y/x);
        if(x < 0){
            angle += Math.PI;
        }
        return angle / (2 * Math.PI);
    }
    
}
