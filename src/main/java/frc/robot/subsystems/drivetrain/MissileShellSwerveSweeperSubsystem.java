package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.Constants;
import frc.robot.util.MotorUtil;

public class MissileShellSwerveSweeperSubsystem extends BaseDrivetrain {
    private final CANSparkMax steerMotor;
    private SparkMaxAnalogSensor steerAbsoluteEncoder;

    private static final int STEER_PORT = Constants.SwerveConstants.brSteer;
    private static final double SWEEP_SPEED = 0.05;

    private double maxVolts, minVolts = 1.0; // TODO: better way of setting "unset" min volts?

    private final ShuffleboardTab shuffleboardTab;
    private final GenericEntry currentVoltsEntry, maxVoltsEntry, minVoltsEntry;

    public MissileShellSwerveSweeperSubsystem() {
        steerMotor = MotorUtil.createSparkMax(STEER_PORT, (sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake);

            steerAbsoluteEncoder = sparkMax.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
        });

        shuffleboardTab = Shuffleboard.getTab("Swerve sweeper " + STEER_PORT);
        currentVoltsEntry = shuffleboardTab.add("Current voltage", 0.0)
            .withPosition(0, 1)
            .withSize(5, 3)
            .withWidget(BuiltInWidgets.kGraph)
            .getEntry();
        maxVoltsEntry = shuffleboardTab.add("Max voltage", 0.0)
            .withPosition(0, 0)
            .getEntry();
        minVoltsEntry = shuffleboardTab.add("Min voltage", 0.0)
            .withPosition(1, 0)
            .getEntry();
    }

    @Override
    public void periodic() {
        double volts = steerAbsoluteEncoder.getPosition();

        maxVolts = Math.max(volts, maxVolts);
        minVolts = Math.min(volts, minVolts);

        currentVoltsEntry.setDouble(volts);
        maxVoltsEntry.setDouble(maxVolts);
        minVoltsEntry.setDouble(minVolts);

        steerMotor.set(SWEEP_SPEED);
    }

    @Override
    public void setDrivePowers(double forwardPower) {
        // ...
    }
}
