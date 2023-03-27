package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

public class LEDLayer {
    
    Color[] ledArray;
    int length;

    public LEDLayer(int length){
        this.length = length;
        ledArray = new Color[length];
        for(int i = 0; i < length; i++){
            ledArray[i] = null;
        }
    }

    /**
     * Set an led color
     * @param i the index of led to set
     * @param color the color to set it to (can be null)
     */
    public void setLED(int i, Color color){
        ledArray[i] = color;
    }
    
    /**
     * Gets LED color
     * @param i the led index to retrieve
     * @return the color of led at index i
     */
    public Color getLED(int i){
        return ledArray[i];
    }

    /**
     * Moves the leds up by an increment
     * @param inc the number of leds to move up by
     * @param color the color to set at the bottom
     */
    public void incrementColors(int inc, Color color){
        for (int i = 0; i < length - inc; i++){
            setLED(i, getLED(i + inc));
        }
        for(int i = length - inc; i < length; i++){
            setLED(i, color);
        }
    }

    /**
     * Fills entire layer with a color
     * @param color color to fill the layer by
     */
    public void fillColor(Color color){
        for(int i = 0; i < length; i++){
            setLED(i, color);
        }
    }

    /**
     * Sets entire layer to empty leds
     */
    public void reset(){
        fillColor(null);
    }

    /**
     * Fills leds in groups NOTE: "off" leds are set to null
     * @param onGroupLength the length of leds to be on in a row
     * @param offGroupLength the length of leds to be off in a row
     * @param color the color to set the on leds
     */
    public void fillGrouped(int onGroupLength, int offGroupLength, Color color){
        for (int i = 0; i < length; i++) {
            if (i % (onGroupLength + offGroupLength) < onGroupLength){
                setLED(i, color);
            } else {
                setLED(i, null);
            }
        }
    }


}
