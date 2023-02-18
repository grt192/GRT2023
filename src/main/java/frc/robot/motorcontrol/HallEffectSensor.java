package frc.robot.motorcontrol;

import java.util.ArrayList;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class HallEffectSensor {
    private DigitalInput sensor;
    private HallEffectMagnet[] magnets; // array of magnet locations from smallest extension to largest extension
    private ArrayList<GenericEntry> shuffleboardEntries = null;

    // State vars tracking current sensor location using index of magnet position[]
    // Ie. [-1, 0] means sensor is located in the area before the 1st magnet, [1, 1] means sensor is at 2nd magnet 
    private int lowerPos; // -1 min, (array length - 1) max
    private int upperPos; // 0 min, array length max

    private boolean prevDetected;
    private double prevMechPos;

    public HallEffectSensor(int id, HallEffectMagnet[] magnets, double initialMechPos) {
        this(id, magnets, initialMechPos, -1, 0);
    }
    
    public HallEffectSensor(int id, HallEffectMagnet[] magnets, double initialMechPos, int lowerPos, int upperPos) {
        this.sensor = new DigitalInput(id);
        this.magnets = magnets;

        this.prevDetected = false;
        this.prevMechPos = initialMechPos;

        this.lowerPos = lowerPos;
        this.upperPos = upperPos;
    }

    /**
     * Add magnets to the given Shuffleboard tab at the given position. 
     * @param shuffleboardTab Shuffleboard tab
     * @param columnIndex initial Shuffleboard column in which the first magnet entry is located
     * @param rowIndex Shuffleboard row in which the entries are located 
     */
    public void addToShuffleboard(ShuffleboardTab shuffleboardTab, int columnIndex, int rowIndex) {
        shuffleboardEntries = new ArrayList<GenericEntry>();
        for (HallEffectMagnet magnet : magnets) {
            GenericEntry entry = shuffleboardTab.add("Magnet " + magnet.getExtendDistanceInches() + "in", false).withPosition(columnIndex++, rowIndex).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
            shuffleboardEntries.add(entry);
        }
    }

    /**
     * Sets magnet Shuffleboard entry. If Shuffleboard is not being used, it does nothing.
     * @param index index of magnet to set value of
     * @param newValue new value
     */
    private void setShuffleboardValue(int index, boolean newValue) {
        if (shuffleboardEntries == null) return; 
        shuffleboardEntries.get(index).setValue(newValue);
    }

    /**
     * Returns a HallEffectMagnet representing sensor location. Must call once every periodic loop to update sensor state/mechanism encoder value.
     *  
     * @param mechPos Mechanism position, double
     * @return A HallEffectMagnet object or null if sensor is between magnets.
     */
    public HallEffectMagnet getHallEffectState(double mechPos) {
        boolean detected = !sensor.get();
        boolean movingUp = (mechPos - prevMechPos) > 0;

        // If magnet is newly detected
        if (detected && prevDetected == false) { // 0V signal (ie. false) indicates magnet is detected
            if (movingUp) {
                lowerPos = upperPos;
            } else {
                upperPos = lowerPos;
            }

            setShuffleboardValue(lowerPos, true);
        }
        
        // If magnet is no longer detected
        if (!detected && prevDetected == true) {
            setShuffleboardValue(lowerPos, false);

            if (movingUp) {
                upperPos = lowerPos + 1;
            } else {
                lowerPos = upperPos - 1;
            }
        }

        // Set previous state
        prevDetected = detected;
        prevMechPos = mechPos;

        return (lowerPos == upperPos) ? magnets[lowerPos] : null;
    }
}
