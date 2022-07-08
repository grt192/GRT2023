package frc.robot.motorcontrol;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class GRTTalonFX extends WPI_TalonFX {
    private double positionConversionFactor = 1.0;
    private double velocityConversionFactor = 1.0;

    /**
     * Constructs a `GRTTalonSRX` from a CAN ID.
     * @param deviceId The CAN ID of the TalonSRX to control.
     */
    public GRTTalonFX(int deviceId) {
        super(deviceId);

        // Set 30.0 amp current limit
        configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30.0, 0, 0));
    }

    /**
     * Sets the position conversion factor of the `TalonSRX`. All calls to `getSelectedSensorPosition()`
     * will be scaled by this value.
     * 
     * @param factor The position conversion factor.
     */
    public void setPositionConversionFactor(double factor) {
        this.positionConversionFactor = factor;
    }

    /**
     * Sets the velocity conversion factor of the `TalonSRX`. All calls to `getSelectedSensorVelocity()`
     * will be scaled by this value.
     * 
     * @param factor The velocity conversion factor.
     */
    public void setVelocityConversionFactor(double factor) {
        this.velocityConversionFactor = factor;
    }

    /**
     * Gets the selected sensor position in sensor ticks * position conversion factor.
     */
    @Override
    public double getSelectedSensorPosition() {
        return super.getSelectedSensorPosition() * positionConversionFactor;
    }

    /**
     * Gets the selected sensor velocity in sensor ticks / 100ms * velocity conversion factor.
     */
    @Override
    public double getSelectedSensorVelocity() {
        return super.getSelectedSensorVelocity() * velocityConversionFactor;
    }

    /**
     * Sets the Talon's output based on the control mode and value specified. See
     * https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_w_p_i___talon_s_r_x.html#a1cec2a01a9f12011b209ecc79eb62367
     * for more detailed documentation.
     * 
     * @param mode The output mode to apply.
     * @param value The setpoint value. Note that for `Position` and `MotionMagic`, 
     * this should be in units of your position conversion factor. For `Velocity`, 
     * this should be in units of your velocity conversion factor.
     */
    @Override
    public void set(ControlMode mode, double value) {
        switch (mode) {
            case Position:
            case MotionMagic:
                super.set(mode, value / positionConversionFactor);
                break;
            case Velocity:
                super.set(mode, value / velocityConversionFactor);
                break;
            default:
                super.set(mode, value);
        }        
    }
}
