package frc.robot.subsystems.drivetrain;

public interface TankIO {
    /**
     * Sets the power of the left motors.
     * @param power The [-1.0, 1.0] power to set.
     */
    default void setLeftPower(double power) {}

    /**
     * Sets the power of the right motors.
     * @param power The [-1.0, 1.0] power to set.
     */
    default void setRightPower(double power) {}
}
