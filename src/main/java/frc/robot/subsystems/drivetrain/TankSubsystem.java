package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

public class TankSubsystem extends BaseDrivetrain {
    private final TankIO tankIO;

    private double leftDrive;
    private double rightDrive;

    public TankSubsystem(TankIO tankIO) {
        this.tankIO = tankIO;
    }

    /**
     * Drives the tank subsystem with the given forward and turn powers.
     * @param forwardPower The power [-1.0, 1.0] in the forward direction.
     * @param turnPower The power [-1.0, 1.0] in the angular direction.
     */
    public void setDrivePowers(double forwardPower, double turnPower) {
        // POSSIBLE/CONCEPT -- double theta = Math.atan(yPower/xPower); // if the robot needs to move some amount laterally, it can rotate to this angle and move in that direction

        leftDrive = forwardPower - turnPower;
        rightDrive = forwardPower + turnPower;
    }

    /**
     * Drives the tank subsystem with the given forward power.
     * @param forwardPower The power [-1.0, 1.0] in the forward direction.
     */
    @Override
    public void setDrivePowers(double fowardPower) {
        setDrivePowers(fowardPower, 0.0);
    }

    @Override
    public void periodic() {
        if (Math.abs(leftDrive) >= 1.0) {
            leftDrive = leftDrive / Math.abs(leftDrive);
            rightDrive = rightDrive / Math.abs(leftDrive);
        }
        if (Math.abs(rightDrive) >= 1.0) {
            rightDrive = rightDrive / Math.abs(rightDrive);
            leftDrive = leftDrive / Math.abs(rightDrive);
        }

        tankIO.setLeftPower(leftDrive);
        tankIO.setRightPower(rightDrive);

        Logger.getInstance().recordOutput("Tank/LeftPower", leftDrive);
        Logger.getInstance().recordOutput("Tank/RightPower", rightDrive);
    }
}
