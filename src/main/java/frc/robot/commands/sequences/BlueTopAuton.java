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
 * The blue top auton sequences, where the robot deposits its preloaded game piece and picks up
 * and places another.
 */
public class BlueTopAuton extends BaseTopAutonSequence {
    private static final Pose2d INITIAL_POSE = new Pose2d(
        Units.inchesToMeters(PlacePosition.BLUE_INIT_X_IN),
        Units.inchesToMeters(190.366),
        Rotation2d.fromDegrees(180)
    );
    private static final Pose2d MIDPOS1 = new Pose2d(
        Units.inchesToMeters(94.373),
        Units.inchesToMeters(187.638),
        Rotation2d.fromDegrees(180)
    );
    private static final Pose2d MIDPOS2 = new Pose2d(
        Units.inchesToMeters(179.577),
        Units.inchesToMeters(187.638),
        Rotation2d.fromDegrees(180)
    );
    private static final Pose2d MIDPOS3 = new Pose2d(
        Units.inchesToMeters(221.451),
        Units.inchesToMeters(205.895),
        Rotation2d.fromDegrees(-90)
    );
    private static final Pose2d MIDPOS4 = new Pose2d(
        Units.inchesToMeters(278.978),
        Units.inchesToMeters(205.895),
        Rotation2d.fromDegrees(-90)
    );

    private static final PlaceState PLACE_STATE = PlacePosition.C2MID.BLUE;
    private static final PlaceState PLACE_STATE_2 = PlacePosition.C3MID.BLUE;

    private static final Pose2d GRAB_POSE = PiecePosition.PIECE1.BLUE;

    public BlueTopAuton(
        BaseSwerveSubsystem swerveSubsystem,
        RollerSubsystem rollerSubsystem,
        TiltedElevatorSubsystem tiltedElevatorSubsystem
    ) {
        super(
            swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
            INITIAL_POSE, MIDPOS1, MIDPOS2, MIDPOS3, MIDPOS4,
            PLACE_STATE, GRAB_POSE, PLACE_STATE_2
        );
    }
}
