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
 * The blue bottom auton sequence, where the robot deposits its preloaded game piece and picks up
 * and places another.
 */
public class BlueBottomAuton extends BaseBottomAutonSequence {
    private static final Pose2d INITIAL_POSE = new Pose2d(
        Units.inchesToMeters(PlacePosition.BLUE_INIT_X_IN),
        Units.inchesToMeters(17.714),
        Rotation2d.fromDegrees(180)
    );
    private static final Pose2d MIDPOS1 = new Pose2d(
        Units.inchesToMeters(94.373),
        Units.inchesToMeters(28.007),
        Rotation2d.fromDegrees(180)
    );
    private static final Pose2d MIDPOS2 = new Pose2d(
        Units.inchesToMeters(195.000),
        Units.inchesToMeters(28.007), 
        Rotation2d.fromDegrees(180)
    );

    private static final PlaceState PLACE_STATE = PlacePosition.A2HIGH.BLUE;
    private static final PlaceState PLACE_STATE_2 = PlacePosition.A2MID.BLUE;

    private static final Pose2d GRAB_POSE = PiecePosition.PIECE4.BLUE;

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
