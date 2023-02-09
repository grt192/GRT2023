package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * The top and bottom auton sequences, where the robot deposits its preloaded game piece and picks up
 * and places another.
 */
public class BlueBottomNonBalanceAuton extends BaseAutonSequence {
    private static Pose2d INITIAL_POSE = new Pose2d(
        Units.inchesToMeters(BLUE_INITX),
        Units.inchesToMeters(12.873),
        Rotation2d.fromDegrees(180)
    );
    private static Pose2d MIDPOS1 = new Pose2d(
        Units.inchesToMeters(94.373),
        Units.inchesToMeters(28.007),
        Rotation2d.fromDegrees(180)
    );
    private static Pose2d MIDPOS2 = new Pose2d(
        Units.inchesToMeters(179.577),
        Units.inchesToMeters(28.007), 
        Rotation2d.fromDegrees(180)
    );

    /**
     * Constructs an auton sequence from the given parameters.
     * @param robotContainer The robot container.
     * @param placePose1 The robot's pose when placing the first game piece.
     * @param pickPose The robot's pose when intaking the second game piece.
     * @param placePose2 The robot's pose when placing the second game piece.
     * @param height The height of the mover subsystem when placing the first game piece.
     * @param height2 The height of the mover subsystem when placing the second game piece.
     */
    public BlueBottomNonBalanceAuton(
        BaseSwerveSubsystem swerveSubsystem,
        RollerSubsystem rollerSubsystem,
        TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d placePose1, 
        Pose2d pickPose, 
        Pose2d placePose2, 
        ElevatorState height, 
        ElevatorState height2
    ) {
        super(
            swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
            INITIAL_POSE, MIDPOS1, MIDPOS2,
            placePose1, pickPose, placePose2,
            height, height2
        );
    }
}