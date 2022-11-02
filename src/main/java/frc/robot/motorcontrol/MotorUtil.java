package frc.robot.motorcontrol;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class MotorUtil {
    /**
     * Creates a WPI_TalonSRX on a given device ID, configuring it with global defaults.
     * @param deviceId The CAN ID of the Talon.
     * @return The configured Talon.
     */
    public static WPI_TalonSRX createTalonSRX(int deviceId) {
        WPI_TalonSRX talon = new WPI_TalonSRX(deviceId);

        // Set 60.0 amp current limit to kick in after 0.2 seconds
        talon.configFactoryDefault();
        talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60.0, 60.0, 0.2));

        return talon;
    }

    /**
     * Creates a WPI_TalonSRX on a given device ID, configuring it with global defaults.
     * @param deviceId The CAN ID of the Talon.
     * @return The configured Talon.
     */
    public static WPI_TalonFX createTalonFX(int deviceId) {
        WPI_TalonFX talon = new WPI_TalonFX(deviceId);

        // Set 60.0 amp current limit to kick in after 0.2 seconds
        talon.configFactoryDefault();
        talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60.0, 60.0, 0.2));

        return talon;
    }

    /**
     * Creates a CANSparkMax on a given device ID and motor type, configuring it with global defaults.
     * @param deviceId The CAN ID of the SparkMax.
     * @param motorType The SparkMax's motor type (kBrushed or kBrushless).
     * @return The configured SparkMax.
     */
    public static CANSparkMax createSparkMax(int deviceId, MotorType motorType) {
        CANSparkMax spark = new CANSparkMax(deviceId, motorType);

        // Set 60.0 amp current limit
        spark.restoreFactoryDefaults();
        spark.setSmartCurrentLimit(60);

        return spark;
    }

    /**
     * Creates a CANSparkMax on a given device ID, configuring it with global defaults.
     * @param deviceId The CAN ID of the SparkMax.
     * @return The configured SparkMax.
     */
    public static CANSparkMax createSparkMax(int deviceId) {
        return createSparkMax(deviceId, MotorType.kBrushless);
    }
}
