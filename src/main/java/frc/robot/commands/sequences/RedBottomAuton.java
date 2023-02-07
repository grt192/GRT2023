package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * The top and bottom auton sequences, where the robot deposits its preloaded game piece and picks up
 * and places another.
 */
public class RedBottomAuton extends BaseBottomAutonSequence {
    private static final Pose2d INITIAL_POSE = (new Pose2d(
        Units.inchesToMeters(PlacePosition.RED_INIT_X_IN),
        Units.inchesToMeters(12.873),
        Rotation2d.fromDegrees(0)
    ));
    private static final Pose2d MIDPOS1 = (new Pose2d(
        Units.inchesToMeters(556.376),
        Units.inchesToMeters(28.007),
        Rotation2d.fromDegrees(0)
    ));
    private static final Pose2d MIDPOS2 = (new Pose2d(
        Units.inchesToMeters(471.173),
        Units.inchesToMeters(28.007),
        Rotation2d.fromDegrees(0)
    ));

    private static final PlacePosition PLACE_STATE = PlacePosition.Red.A3HIGH;
    private static final PlacePosition PLACE_STATE_2 = PlacePosition.Red.A3MID;

    private static final PiecePosition GRAB_POSE = PiecePosition.Blue.PIECE1;

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
