package frc.robot.motorcontrol;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
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
     * @param configureMotor A Consumer<CANSparkMax> to configure the motor further before settings are burned to flash.
     * @return The configured SparkMax.
     */
    public static CANSparkMax createSparkMax(int deviceId, MotorType motorType, Consumer<CANSparkMax> configureMotor) {
        CANSparkMax spark = new CANSparkMax(deviceId, motorType);

        // Set 60.0 amp current limit
        spark.restoreFactoryDefaults();
        spark.setSmartCurrentLimit(60);

        // Apply manually configured settings
        configureMotor.accept(spark);
        spark.burnFlash();

        return spark;
    }

    /**
     * Creates a brushless CANSparkMax on a given device ID, configuring it with global defaults.
     * @param deviceId The CAN ID of the SparkMax.
     * @param configureMotor A Consumer<CANSparkMax> to configure the motor further before settings are burned to flash.
     * @return The configured SparkMax.
     */
    public static CANSparkMax createSparkMax(int deviceId, Consumer<CANSparkMax> configureMotor) {
        return createSparkMax(deviceId, MotorType.kBrushless, configureMotor);
    }

    /**
     * Creates a brushless CANSparkMax on a given device ID, configuring it with global defaults.
     * @param deviceId The CAN ID of the SparkMax.
     * @return The configured SparkMax.
     */
    public static CANSparkMax createSparkMax(int deviceId) {
        return createSparkMax(deviceId, MotorType.kBrushless, (sparkMax) -> {});
    }
}
