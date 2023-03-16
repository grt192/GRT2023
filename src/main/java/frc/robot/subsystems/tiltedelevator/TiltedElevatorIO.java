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

    /**
     * Zeros the position of the extension encoder.
     */
    public default void zeroExtensionEncoder() {}

    /**
     * Sets the power of the extension motor.
     * @param power The [-1.0, 1.0] power to set.
     */
    public default void setPower(double power) {}

    /**
     * Sets the target position of the extension motor.
     * @param targetExtensionMeters The target extension, in meters.
     */
    public default void setPosition(double targetExtensionMeters) {}

    /**
     * Sets whether to enable the extension reverse soft limit.
     * @param enable Whether to enable the reverse soft limit.
     */
    public default void enableReverseLimit(boolean enable) {}
}
