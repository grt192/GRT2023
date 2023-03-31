package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.RollerSubsystem.HeldPiece;
import frc.robot.util.TrackingTimer;
import static frc.robot.Constants.LEDConstants.*;

public class LEDSubsystem extends SubsystemBase {
    private final LEDStrip ledStrip;
    private final LEDLayer baseLayer;
    private final LEDLayer manualColorLayer;
    private final LEDLayer aprilDetectedLayer;
    private final LEDLayer colorSensorLayer;
    private final LEDLayer heldPieceLayer;

    private final Timer blinkTimer;
    private static final double BLINK_DURATION_SECONDS = 0.5;
    private static final double BLINK_OFF_TO_ON_RATIO = 4;
    private static final Color BLINK_COLOR = new Color(0, 0, 0);
    private boolean blinking = false;

    private final TrackingTimer aprilBlinkTimer = new TrackingTimer();
    private static final double APRIL_BLINK_DURATION_SECONDS = 0.05;

    private final Timer fadeTimer = new Timer();
    private static final double COLOR_SENSOR_FADE_PERIOD_SECONDS = .5;
    private boolean colorSensorOff = false;

    private static final double BRIGHTNESS_SCALE_FACTOR = 0.25;
    private static final double INPUT_DEADZONE = 0.35;
    private static final int LEDS_PER_SEC = 150;

    private Color pieceColor = CUBE_COLOR;
    private Color manualColor = new Color(0, 0, 0);

    private HeldPiece heldPiece = HeldPiece.EMPTY;
    private boolean risingEdge = false;
    private boolean fallingEdge = false;
    private Color RISING_EDGE_COLOR = new Color(0, 0, 255);
    private Color FALLING_EDGE_COLOR = new Color(255, 0, 0);
    private Timer heldPieceTimer = new Timer();

    private boolean manual = false; // If the driver is directly controlling leds
    public boolean pieceGrabbed = false;

    private static final Color APRIL_COLOR = scaleDownColorBrightness(new Color(252, 255, 236));
    private static final Color CUBE_COLOR = scaleDownColorBrightness(new Color(192, 8, 254));
    private static final Color CONE_COLOR = scaleDownColorBrightness(new Color(255, 100, 0));
    private static final Color COLOR_SENSOR_OFF_COLOR = scaleDownColorBrightness(new Color(255, 0, 0));

    private final Timer ledTimer; // TODO: better naming

    public LEDSubsystem() {
        ledStrip = new LEDStrip(LED_PWM_PORT, LED_LENGTH);

        baseLayer = new LEDLayer(LED_LENGTH);
        manualColorLayer = new LEDLayer(LED_LENGTH);
        aprilDetectedLayer = new LEDLayer(LED_LENGTH);
        colorSensorLayer = new LEDLayer(LED_LENGTH);
        heldPieceLayer = new LEDLayer(LED_LENGTH);

        blinkTimer = new Timer();
        ledTimer = new Timer();
        ledTimer.start();
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
        
        // Number of leds to increment each continuous led layer by
        int inc = Math.min((int) Math.ceil(ledTimer.get() * LEDS_PER_SEC), LED_LENGTH);
        ledTimer.reset();
        ledTimer.start();

        // Update baseLayer - the piece color indicated by the mech driver, or the blink color if a piece
        // is held and we are blinking.
        Color baseColor = blinking ? BLINK_COLOR : pieceColor;
        baseLayer.fillColor(baseColor);

        // Update manualColorLayer - the manual color set by the mech driver in manual mode.
        if (manual) {
            manualColorLayer.incrementColors(inc, manualColor);
        } else {
            manualColorLayer.reset();
        }

        if(fallingEdge){
            heldPieceLayer.fillColor(FALLING_EDGE_COLOR);
            heldPieceTimer.reset();
            heldPieceTimer.start();
        } else if (risingEdge){
            heldPieceLayer.fillColor(RISING_EDGE_COLOR);
            heldPieceTimer.reset();
            heldPieceTimer.start();
        } else if (heldPieceTimer.hasElapsed(1)){
            heldPieceLayer.fillColor(null);
        }

        // Update colorSensorLayer - pulsing red grouped indicators to indicate a color sensor failure.
        if (colorSensorOff) {
            fadeTimer.start();
            colorSensorLayer.fillGrouped(
                5, 10,
                crossFadeWithTime(COLOR_SENSOR_OFF_COLOR, baseColor, fadeTimer.get(), COLOR_SENSOR_FADE_PERIOD_SECONDS)
            );
        } else {
            fadeTimer.stop();
            fadeTimer.reset();
            colorSensorLayer.reset();
        }

        // Update aprilDetectedLayer - white pulses to indicate an april tag detection.
        if (!aprilBlinkTimer.hasElapsed(APRIL_BLINK_DURATION_SECONDS) && aprilBlinkTimer.hasStarted()) {
            aprilDetectedLayer.incrementColors(inc, APRIL_COLOR);
        } else {
            aprilDetectedLayer.incrementColors(inc, null);
        }

        // Add layers to buffer, set leds
        ledStrip.addLayer(baseLayer);
        ledStrip.addLayer(manualColorLayer);
        ledStrip.addLayer(heldPieceLayer);
        ledStrip.addLayer(colorSensorLayer);
        ledStrip.addLayer(aprilDetectedLayer);
        ledStrip.setBuffer();
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
    }

    /**
     * Cross-fades between two colors using a sinusoidal scaling function.
     * @param color The color to set.
     * @param fadeColor The color to fade into.
     * @param currentTimeSeconds The current elapsed time of fading.
     * @param periodSeconds The period of the fade function, in seconds.
     * @return The scaled and faded color.
     */
    private static Color crossFadeWithTime(Color color, Color fadeColor, double currentTimeSeconds, double periodSeconds) {
        // The [0.0, 1.0] brightness scale to scale the color by. Scale = 1/2 * cos(t) + 1/2 where
        // t is scaled to produce the desired period.
        double scale = 0.5 * Math.cos(currentTimeSeconds * 2 * Math.PI / periodSeconds) + 0.5;

        return new Color(
            color.red * scale + fadeColor.red * (1 - scale),
            color.green * scale + fadeColor.green * (1 - scale),
            color.blue * scale + fadeColor.blue * (1 - scale)
        );
    }

    /**
     * Scales a color's brightness by the BRIGHTNESS_SCALE_FACTOR.
     * @param color The color to scale down.
     * @return The scaled down color.
     */
    private static Color scaleDownColorBrightness(Color color) {
        return new Color(
            color.red * BRIGHTNESS_SCALE_FACTOR,
            color.green * BRIGHTNESS_SCALE_FACTOR,
            color.blue * BRIGHTNESS_SCALE_FACTOR
        );
    }

    /**
     * Sets the color of the LEDs commanded by driver input. If manual input is enabled, this sets the color
     * to the HSV color created by the angle of the joystick. Otherwise, set the color to CUBE if the joystick
     * is pushed right and CONE if it is pushed left.
     * 
     * @param x The x input of the joystick.
     * @param y The y input of the joystick.
     */
    public void setDriverColors(double x, double y){
        double angleRads = MathUtil.inputModulus(Math.atan2(y, x), 0, 2 * Math.PI);
        manualColor = Color.fromHSV(
            (int) (Math.toDegrees(angleRads) / 2.0),
            (int) 255,
            (int) (255 * BRIGHTNESS_SCALE_FACTOR)
        );

        if (x > INPUT_DEADZONE) {
            pieceColor = CUBE_COLOR;
        } else if (x < -INPUT_DEADZONE) {
            pieceColor = CONE_COLOR;
        }
    }

    /**
     * Sets the color sensor status
     * @param dead whether the color sensor is dead or not
     */
    public void setColorSensorOff(boolean dead){
        colorSensorOff = dead;
    }

    public void setHeldPiece(HeldPiece newHeldPiece){
        if(heldPiece == HeldPiece.EMPTY && newHeldPiece != HeldPiece.EMPTY){
            risingEdge = true;
        } else {
            risingEdge = false;
        }

        if(heldPiece != HeldPiece.EMPTY && newHeldPiece == HeldPiece.EMPTY){
            fallingEdge = true;
        } else {
            fallingEdge = false;
        }
        heldPiece = newHeldPiece;
    }
}
