package frc.robot.motorcontrol;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.tiltedelevator.ElevatorState;

public class HallEffectMagnet {

    private double extendDistanceMeters;
    private ElevatorState elevatorState;

    public HallEffectMagnet(double extendDistanceMeters) {
        this(extendDistanceMeters, null);
    }

    public HallEffectMagnet(double extendDistanceMeters, ElevatorState elevatorState) {
        this.extendDistanceMeters = extendDistanceMeters;
        this.elevatorState = elevatorState;
    }
    
    public double getExtendDistanceMeters() { return extendDistanceMeters; }
    
    public double getExtendDistanceInches() { return Units.metersToInches(extendDistanceMeters); }

    public boolean hasElevatorState() { return elevatorState != null; }

    public ElevatorState getElevatorState() { return elevatorState; }
}
