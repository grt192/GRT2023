package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LEDConstants.*;

public class LEDSubsystem extends SubsystemBase {
    private final LEDStrip ledStrip;

    private final Timer blinkTimer;
    private static final double BLINK_DURATION_SECONDS = 0.5;
    private final double APRIL_BLINK_DURATION = .2;
    private static final Color BLINK_COLOR = new Color(0, 0, 0);
    private boolean blinking = false;
    private boolean continuous = false;
    public boolean manual = false;

    private static final double BRIGHTNESS_SCALE_FACTOR = 0.25;

    private Color color = new Color(192, 8, 254);
    private final Color aprilColor = new Color(255, 0, 0);
    private final Timer aprilTimer = new Timer();
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
        if(manual && continuous){
            //if manual and continuous set the bottom to color
            ledStrip.updateContinuousColor(color);
            ledStrip.setContinuousColor();
        } else if(manual){
            //if manual and not continuous fill entire strip with color
            ledStrip.fillContinuousColor(color);
            ledStrip.setContinuousColor();
        } else {
            Color currentColor;
            //current color is the color that will be added at the bottom of the strip buffer
            if(!aprilTimer.hasElapsed(APRIL_BLINK_DURATION)){
                currentColor = aprilColor;
            } else {
                currentColor = color;
            }
            if(blinking){
                ledStrip.setSolidColor(BLINK_COLOR);
                //we update the continuous color so that the pulses continue even when the leds are off
                ledStrip.updateContinuousColor(currentColor);
            }
            //if the leds are on update the continuous color and then set the leds to that continuous buffer
            ledStrip.updateContinuousColor(currentColor);
            ledStrip.setContinuousColor();
        }
    }

    /**
     * Sets the driver-provided color of the LEDs from provided RGB values.
     * @param r The red component of the color, from [0, 255].
     * @param g The green component of the color, from [0, 255].
     * @param b The blue component of the color, from [0, 255].
     */
    public void setRGB(double r, double g, double b) {
        //if the color doesnt change, dont do anything
        if(this.color.red == r && this.color.green == g && this.color.blue == b){ 
            return;
        }
        this.color = new Color(
            (int) (r * BRIGHTNESS_SCALE_FACTOR),
            (int) (g * BRIGHTNESS_SCALE_FACTOR),
            (int) (b * BRIGHTNESS_SCALE_FACTOR)
        );
        //sets the whole strip to the color
        ledStrip.fillContinuousColor(this.color);
        ledStrip.setContinuousColor();
    }

    /**
     * Sets the driver-provided color of the LEDs from provided HSV values.
     * @param h The hue component of the color, from [0, 180).
     * @param s The saturation component of the color, from [0, 255].
     * @param v The value component of the color, from [0, 255].
     */
    //sets the color to the 
    public void setHSV(double h, double s, double v) {
        this.color = Color.fromHSV(
            (int) h,
            (int) s,
            (int) (v * BRIGHTNESS_SCALE_FACTOR)
        );
    }

    public void toggleLEDControlMode(){
        continuous = !continuous;
    }

    public void setManual(boolean manual){
        this.manual = manual;
    }

    public void tagDetected(){
        if(aprilTimer.hasElapsed(APRIL_BLINK_DURATION * 2)){
            aprilTimer.reset();
            aprilTimer.start();
        }
    }
}
