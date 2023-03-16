package frc.robot.subsystems.tiltedelevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;
import frc.robot.sensors.HallEffectSensor;
import frc.robot.util.MotorUtil;

import static frc.robot.Constants.TiltedElevatorConstants.*;

public class TiltedElevatorIOReal implements TiltedElevatorIO {
    private final CANSparkMax extensionMotor;
    private RelativeEncoder extensionEncoder;
    private SparkMaxPIDController extensionPidController;

    private final CANSparkMax extensionFollow;
    private final CANSparkMax extensionFollowB;

    private final DigitalInput zeroLimitSwitch;
    private final HallEffectSensor leftHallSensor;

    // Constants
    private static final double EXTENSION_GEAR_RATIO = 14.0 / 64.0;
    private static final double EXTENSION_CIRCUMFERENCE = Units.inchesToMeters(Math.PI * 0.500); // approx circumference of winch
    private static final double EXTENSION_ROTATIONS_TO_METERS = EXTENSION_GEAR_RATIO * EXTENSION_CIRCUMFERENCE * 2.0 * (15.0 / 13.4);

    private static final double extensionP = 2.4;
    private static final double extensionI = 0;
    private static final double extensionD = 0;
    private static final double extensionTolerance = 0.003;
    private static final double extensionRampRate = 0.4;
    private static final double arbFeedforward = Constants.IS_R1 ? 0.03 : 0;

    public TiltedElevatorIOReal() {
        extensionMotor = MotorUtil.createSparkMax(EXTENSION_ID, (sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake); 
            sparkMax.setInverted(true);
            sparkMax.setClosedLoopRampRate(extensionRampRate);

            extensionEncoder = sparkMax.getEncoder();
            extensionEncoder.setPositionConversionFactor(EXTENSION_ROTATIONS_TO_METERS);
            extensionEncoder.setVelocityConversionFactor(EXTENSION_ROTATIONS_TO_METERS / 60.0);
            extensionEncoder.setPosition(0);

            sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) (EXTENSION_LIMIT_METERS + Units.inchesToMeters(.5)));
            sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
            sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.inchesToMeters(-2));
            sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);

            extensionPidController = MotorUtil.createSparkMaxPIDController(sparkMax, extensionEncoder);
            extensionPidController.setP(extensionP);
            extensionPidController.setI(extensionI);
            extensionPidController.setD(extensionD);
            extensionPidController.setSmartMotionAllowedClosedLoopError(extensionTolerance, 0);
        });

        extensionFollow = MotorUtil.createSparkMax(EXTENSION_FOLLOW_ID, (sparkMax) -> {
            sparkMax.follow(extensionMotor);
            sparkMax.setIdleMode(IdleMode.kBrake);
        });

        if (Constants.IS_R1) extensionFollowB = null;
        else extensionFollowB = MotorUtil.createSparkMax(EXTENSION_FOLLOW_B_ID, (sparkMax) -> {
            sparkMax.follow(extensionMotor);
            sparkMax.setIdleMode(IdleMode.kBrake);
        });

        zeroLimitSwitch = new DigitalInput(ZERO_LIMIT_ID);

        if (Constants.IS_R1) leftHallSensor = null;
        else leftHallSensor = new HallEffectSensor(LEFT_HALL_ID, LEFT_MAGNETS, extensionEncoder.getPosition());
    }

    @Override
    public void updateInputs(TiltedElevatorIOInputs inputs) {
        inputs.zeroLimitSwitchPressed = !zeroLimitSwitch.get();
        inputs.currentExtensionMeters = extensionEncoder.getPosition();
        inputs.currentVelocityMetersPerSecond = extensionEncoder.getVelocity();
    }

    @Override
    public void zeroExtensionEncoder() {
        extensionEncoder.setPosition(0);
    }

    @Override
    public void setPower(double power) {
        extensionMotor.set(power);
    }

    @Override
    public void setPosition(double targetExtensionMeters) {
        extensionPidController.setReference(
            MathUtil.clamp(targetExtensionMeters, 0, EXTENSION_LIMIT_METERS + Units.inchesToMeters(0.75)),
            ControlType.kPosition, 0,
            arbFeedforward, ArbFFUnits.kPercentOut
        );
    }

    @Override
    public void enableReverseLimit(boolean enable) {
        extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, enable);
    }
}
