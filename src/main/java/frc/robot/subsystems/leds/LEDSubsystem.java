package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
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
    private static final double INPUT_DEADZONE = 0.1;

    private Color color = CUBE_COLOR;
    private Color lastPieceColor = color;

    private boolean manual = false; //when driver is directly controlling leds
    public boolean pieceGrabbed = false;

    private final Timer aprilBlinkTimer = new Timer();
    private final Timer aprilFlashTimer = new Timer();
    private static final double APRIL_BLINK_DURATION_SECONDS = 0.05;
    private boolean aprilStarted = false;

    private static final Color APRIL_COLOR = new Color(255, 255, 255);
    private static final Color CUBE_COLOR = new Color(192, 8, 254);
    private static final Color CONE_COLOR = new Color(255, 100, 0);

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
        if (manual) {
            //if manual and continuous set the bottom to color
            ledStrip.updateContinuousColor(color);
            ledStrip.setBuffer();
        } else {
            setColorPulse();
            //setTwoColor()
        }
    }

    /**
     * Toggles whether drivers are manually controlling the color of the LEDs.
     */
    public void toggleManual() {
        this.manual = !manual;
    }

    /**
     * Displays that an AprilTag has been detected by sending a pulse down the LEDs.
     */
    public void displayTagDetected() {
        if (!aprilStarted) {
            aprilStarted = true;
            aprilBlinkTimer.start();
        }
        if (aprilBlinkTimer.hasElapsed(APRIL_BLINK_DURATION_SECONDS * 5)) {
            aprilBlinkTimer.reset();
            aprilBlinkTimer.start();
        }
        aprilFlashTimer.reset();
        aprilFlashTimer.start();
    }

    private void setColorPulse() {
        //current color is the color that will be added at the bottom of the strip buffer
        if (!aprilBlinkTimer.hasElapsed(APRIL_BLINK_DURATION_SECONDS) && aprilBlinkTimer.hasElapsed(.001)){
            ledStrip.updateContinuousColor(APRIL_COLOR);
        } else {
            ledStrip.fillSolidColorIgnoringColor(color, APRIL_COLOR);
            ledStrip.updateContinuousColor(color);
        }

        if (blinking) ledStrip.fillSolidColorIgnoringColor(BLINK_COLOR, APRIL_COLOR);
        ledStrip.setBuffer();
    }

    private void setTwoColor() {
        Color color2 = !aprilFlashTimer.hasElapsed(0.05)
            ? APRIL_COLOR
            : color;

        ledStrip.fillGroupedColors(blinking ? BLINK_COLOR : color, color2);
        ledStrip.setBuffer();
    }

    /**
     * Sets the color of the LEDs commanded by driver input. If manual input is enabled, this sets the color
     * to the HSV color created by the angle of the joystick. Otherwise, set the color to CUBE if the joystick
     * is pushed right and CONE if it is pushed left.
     * 
     * @param x The x input of the joystick.
     * @param y The y input of the joystick.
     */
    public void setDriverColor(double x, double y){
        if (manual) {
            double angleRads = MathUtil.inputModulus(Math.atan2(y, x), 0, 2 * Math.PI);
            color = Color.fromHSV(
                (int) (Math.toDegrees(angleRads) / 2.0),
                (int) 255,
                (int) (255 * BRIGHTNESS_SCALE_FACTOR)
            );
        } else if (x > INPUT_DEADZONE) {
            color = CUBE_COLOR;
            lastPieceColor = color;
        } else if (x < -INPUT_DEADZONE) {
            color = CONE_COLOR;
            lastPieceColor = color;
        } else {
            color = lastPieceColor;
        }
    }
}
