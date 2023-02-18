package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.positions.PiecePosition;
import frc.robot.positions.PlacePosition;
import frc.robot.positions.PlacePosition.PlaceState;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * The red bottom auton sequences, where the robot deposits its preloaded game piece and picks up
 * and places another.
 */
public class RedBottomAuton extends BaseBottomAutonSequence {
    private static final Pose2d INITIAL_POSE = new Pose2d(
        Units.inchesToMeters(PlacePosition.RED_INIT_X_IN),
        Units.inchesToMeters(17.74),
        new Rotation2d()
    );
    private static final Pose2d MIDPOS1 = new Pose2d(
        Units.inchesToMeters(556.376),
        Units.inchesToMeters(28.007),
        new Rotation2d()
    );
    private static final Pose2d MIDPOS2 = new Pose2d(
        Units.inchesToMeters(456.3),
        Units.inchesToMeters(28.007),
        new Rotation2d()
    );

    private static final PlaceState PLACE_STATE = PlacePosition.A1HIGH.RED;
    private static final PlaceState PLACE_STATE_2 = PlacePosition.A1MID.RED;

    private static final Pose2d GRAB_POSE = PiecePosition.PIECE4.RED;

    public RedBottomAuton(
        BaseSwerveSubsystem swerveSubsystem,
        RollerSubsystem rollerSubsystem,
        TiltedElevatorSubsystem tiltedElevatorSubsystem
    ) {
        super(
            swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
            INITIAL_POSE, MIDPOS1, MIDPOS2,
            PLACE_STATE, GRAB_POSE, PLACE_STATE_2
        );
    }
}
