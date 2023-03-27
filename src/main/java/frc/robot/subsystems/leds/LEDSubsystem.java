package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.TrackingTimer;
import static frc.robot.Constants.LEDConstants.*;

public class LEDSubsystem extends SubsystemBase {
    private final LEDStrip ledStrip;

    private final Timer blinkTimer;
    private static final double BLINK_DURATION_SECONDS = 0.5;
    private static final double BLINK_OFF_TO_ON_RATIO = 4;
    private static final Color BLINK_COLOR = new Color(0, 0, 0);
    private boolean blinking = false;

    private static final double BRIGHTNESS_SCALE_FACTOR = 0.25;
    private static final double INPUT_DEADZONE = 0.1;

    private Color currentColor = CUBE_COLOR;
    private Color lastPieceColor = currentColor;

    private boolean manual = false; //when driver is directly controlling leds
    public boolean pieceGrabbed = false;

    private final TrackingTimer aprilBlinkTimer = new TrackingTimer();
    private final TrackingTimer aprilFlashTimer = new TrackingTimer();
    private static final double APRIL_BLINK_DURATION_SECONDS = 0.05;

    private static final Color APRIL_COLOR = new Color(252, 255, 236);
    private static final Color CUBE_COLOR = scaleDownColorBrightness(new Color(192, 8, 254));
    private static final Color CONE_COLOR = scaleDownColorBrightness(new Color(255, 100, 0));
    private static final Color COLOR_SENSOR_OFF_COLOR = scaleDownColorBrightness(new Color(255, 0, 0));

    private boolean colorSensorOff = false;

    public LEDSubsystem() {
        ledStrip = new LEDStrip(LED_PWM_PORT, LED_LENGTH);
        blinkTimer = new Timer();
    }

    private static Color scaleDownColorBrightness(Color color) {
        return new Color(
            color.red * BRIGHTNESS_SCALE_FACTOR,
            color.green * BRIGHTNESS_SCALE_FACTOR,
            color.blue * BRIGHTNESS_SCALE_FACTOR
        );
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
            ledStrip.updateContinuousColor(currentColor);
        } else {
            pushColorsToBufferAsPulses();
            //setTwoColor()
        }

        //if the color sensor is off, add red in groups on top of the current buffer, otherwise push the current buffer
        if(colorSensorOff){
            ledStrip.fillGroupedWithBlanks(COLOR_SENSOR_OFF_COLOR, 10, 20);
            ledStrip.setOverlay();
        } else {
            ledStrip.setBuffer();
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
        aprilBlinkTimer.start();
        aprilBlinkTimer.advanceIfElapsed(APRIL_BLINK_DURATION_SECONDS * BLINK_OFF_TO_ON_RATIO);

        aprilFlashTimer.reset();
        aprilFlashTimer.start();
    }

    private void pushColorsToBufferAsPulses() {
        if (!aprilBlinkTimer.hasElapsed(APRIL_BLINK_DURATION_SECONDS) && aprilBlinkTimer.hasStarted()){
            ledStrip.updateContinuousColor(APRIL_COLOR);
        } else {
            ledStrip.fillSolidColorIgnoringColor(currentColor, APRIL_COLOR);
            ledStrip.updateContinuousColor(currentColor);
        }

        if (blinking) ledStrip.fillSolidColorIgnoringColor(BLINK_COLOR, APRIL_COLOR);
    }

    private void setTwoColor() {
        Color color2 = !aprilFlashTimer.hasElapsed(0.05)
            ? APRIL_COLOR
            : currentColor;

        ledStrip.fillGroupedColors(blinking ? BLINK_COLOR : currentColor, color2);
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
            currentColor = Color.fromHSV(
                (int) (Math.toDegrees(angleRads) / 2.0),
                (int) 255,
                (int) (255 * BRIGHTNESS_SCALE_FACTOR)
            );
        } else if (x > INPUT_DEADZONE) {
            currentColor = CUBE_COLOR;
            lastPieceColor = currentColor;
        } else if (x < -INPUT_DEADZONE) {
            currentColor = CONE_COLOR;
            lastPieceColor = currentColor;
        } else {
            currentColor = lastPieceColor;
        }
    }

    public void setColorSensorOff(boolean dead){
        colorSensorOff = dead;
    }
}
