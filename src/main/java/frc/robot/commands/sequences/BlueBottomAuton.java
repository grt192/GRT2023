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
public class BlueBottomAuton extends BaseBottomAutonSequence {
    private static final Pose2d INITIAL_POSE = new Pose2d(
        Units.inchesToMeters(PlacePosition.BLUE_INIT_X_IN),
        Units.inchesToMeters(12.873),
        Rotation2d.fromDegrees(180)
    );
    private static final Pose2d MIDPOS1 = new Pose2d(
        Units.inchesToMeters(94.373),
        Units.inchesToMeters(28.007),
        Rotation2d.fromDegrees(180)
    );
    private static final Pose2d MIDPOS2 = new Pose2d(
        Units.inchesToMeters(179.577),
        Units.inchesToMeters(28.007), 
        Rotation2d.fromDegrees(180)
    );

    private static final PlacePosition PLACE_STATE = PlacePosition.Blue.A1MID;
    private static final PlacePosition PLACE_STATE_2 = PlacePosition.Blue.A2MID;

    private static final PiecePosition GRAB_POSE = PiecePosition.Blue.PIECE1;

    public BlueBottomAuton(
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
