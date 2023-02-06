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
public class RedTopNonBalanceAuton extends BaseAutonSequence {
    private static Pose2d INITIAL_POSE = new Pose2d(
        Units.inchesToMeters(RED_INITX),
        Units.inchesToMeters(195.55),
        Rotation2d.fromDegrees(0)
    );
    private static Pose2d MIDPOS1 = new Pose2d(
        Units.inchesToMeters(556.376),
        Units.inchesToMeters(187.638),
        Rotation2d.fromDegrees(0)
    );
    private static Pose2d MIDPOS2 = new Pose2d(
        Units.inchesToMeters(471.173),
        Units.inchesToMeters(187.638),
        Rotation2d.fromDegrees(0)
    );
    private static Pose2d MIDPOS3 = new Pose2d(
        Units.inchesToMeters(429.104),
        Units.inchesToMeters(205.895),
        Rotation2d.fromDegrees(-90)
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
    public RedTopNonBalanceAuton(
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
            INITIAL_POSE, MIDPOS1, MIDPOS2, MIDPOS3,
            placePose1, pickPose, placePose2,
            height, height2
        );
    }
}