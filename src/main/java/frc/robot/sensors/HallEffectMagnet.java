package frc.robot.sensors;

public class HallEffectMagnet {
    private final double extendDistanceMeters;

    public HallEffectMagnet(double extendDistanceMeters) {
        this.extendDistanceMeters = extendDistanceMeters;
    }
    
    public double getExtendDistanceMeters() {
        return extendDistanceMeters;
    }
}
