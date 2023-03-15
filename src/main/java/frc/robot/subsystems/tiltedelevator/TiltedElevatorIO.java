package frc.robot.subsystems.tiltedelevator;

import org.littletonrobotics.junction.AutoLog;

public interface TiltedElevatorIO {
    @AutoLog
    public static class TiltedElevatorIOInputs {
        public boolean zeroLimitSwitchPressed;
        public double currentExtensionMeters;
        public double currentVelocityMetersPerSecond;
    }

    public default void updateInputs(TiltedElevatorIOInputs inputs) {};

    public default void zeroExtensionEncoder() {}

    public default void setExtensionPower(double power) {}

    public default void setExtensionPosition(double targetExtensionMeters) {}

    public default void enableReverseLimit(boolean enable) {}
}
