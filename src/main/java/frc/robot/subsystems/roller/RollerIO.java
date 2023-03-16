package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public static class RollerIOInputs {
        public boolean limitSwitchPressed;
        public double proximitySensorReading;

        public double colorSensorRed;
        public double colorSensorGreen;
        public double colorSensorBlue;
    }

    default void updateInputs(RollerIOInputs inputs) {};

    /**
     * Sets the power of the rollers.
     * @param power The [-1.0, 1.0] power to set.
     */
    default void setRollPower(double power) {};

    /**
     * Sets the power of the open motor.
     * @param power The [-1.0, 1.0] power to set.
     */
    default void setOpenPower(double power) {};
}
