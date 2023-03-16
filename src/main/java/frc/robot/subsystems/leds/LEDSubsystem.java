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
    private final double APRIL_BLINK_DURATION = .1;
    private static final Color BLINK_COLOR = new Color(0, 0, 0);
    private boolean blinking = false;
    public boolean manual = false; //when driver is directly controlling leds

    private static final double BRIGHTNESS_SCALE_FACTOR = 0.25;
    private static final double INPUT_DEADZONE = .1;

    private Color color = new Color(192, 8, 254);
    private final Color APRIL_COLOR = new Color(255, 0, 0);
    private final Color CUBE_COLOR = new Color(192, 8, 254);
    private final Color CONE_COLOR = new Color(255, 100, 0);
    private final Timer aprilTimer = new Timer();
    private final Timer aprilTimer2 = new Timer();
    private Color lastColor = color;
    public boolean pieceGrabbed = false;
    private boolean aprilStarted = false;

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
        if(manual){
            //if manual and continuous set the bottom to color
            ledStrip.updateContinuousColor(color);
            ledStrip.setContinuousColor();
        } else {
            setColorPulse();
            //setTwoColor()
        }
    }

    public void setManual(boolean manual){
        this.manual = manual;
    }

    public void tagDetected(){
        if(!aprilStarted){
            aprilStarted = true;
            aprilTimer.start();
        }
        if(aprilTimer.hasElapsed(APRIL_BLINK_DURATION * 3)){
            aprilTimer.reset();
            aprilTimer.start();
        }
        aprilTimer2.reset();
        aprilTimer2.start();
    }

    public void setColorPulse(){
        //current color is the color that will be added at the bottom of the strip buffer
        if(!aprilTimer.hasElapsed(APRIL_BLINK_DURATION) && aprilTimer.hasElapsed(.001)){
            ledStrip.updateContinuousColor(APRIL_COLOR);
        } else {
            ledStrip.fillContinuousColorIgnoringOneColor(color, APRIL_COLOR);
            ledStrip.updateContinuousColor(color);
        }
        if(blinking){
            ledStrip.fillContinuousColorIgnoringOneColor(BLINK_COLOR, APRIL_COLOR);
        }
        ledStrip.setContinuousColor();
        //if the leds are on update the continuous color and then set the leds to that continuous buffer
        
    }

    public void setTwoColor(){
        Color color2;
        if(blinking){
            color = BLINK_COLOR;
        }
        if(!aprilTimer2.hasElapsed(.05)){
            color2 = APRIL_COLOR;
        } else {
            color2 =  color;
        }
        ledStrip.setTwoColors(color, color2);
    }

    public void setColorFromInput(double x, double y){
        if(manual){
            double angleRads = MathUtil.inputModulus(Math.atan2(y, x), 0, 2 * Math.PI);
            this.color = Color.fromHSV(
            (int) (angleRads / (2 * Math.PI) * 180),
            (int) 255,
            (int) (255 * BRIGHTNESS_SCALE_FACTOR)
        );
        } else {
            if(x > INPUT_DEADZONE){
                color = CUBE_COLOR;
                lastColor = color;
            } else if(x < -INPUT_DEADZONE){
                color = CONE_COLOR;
                lastColor = color;
            } else {
                color = lastColor;
            }
        }
    }
}
