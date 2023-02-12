package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.positions.PiecePosition;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * The red top auton sequences, where the robot deposits its preloaded game piece and picks up
 * and places another.
 */
public class RedTopAuton extends BaseTopAutonSequence {
    private static final Pose2d INITIAL_POSE = new Pose2d(
        Units.inchesToMeters(PlacePosition.RED_INIT_X_IN),
        Units.inchesToMeters(190.366),
        new Rotation2d()
    );
    private static final Pose2d MIDPOS1 = new Pose2d(
        Units.inchesToMeters(556.376),
        Units.inchesToMeters(187.638),
        new Rotation2d()
    );
    private static final Pose2d MIDPOS2 = new Pose2d(
        Units.inchesToMeters(471.173),
        Units.inchesToMeters(187.638),
        new Rotation2d()
    );
    private static final Pose2d MIDPOS3 = new Pose2d(
        Units.inchesToMeters(429.104),
        Units.inchesToMeters(205.895),
        Rotation2d.fromDegrees(-90)
    );

    private static final PlacePosition PLACE_STATE = PlacePosition.Red.C3HIGH;
    private static final PlacePosition PLACE_STATE_2 = PlacePosition.Red.C3MID;

    private static final PiecePosition GRAB_POSE = PiecePosition.Red.PIECE1;

    public RedTopAuton(
        BaseSwerveSubsystem swerveSubsystem,
        RollerSubsystem rollerSubsystem,
        TiltedElevatorSubsystem tiltedElevatorSubsystem
    ) {
        super(
            swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
            INITIAL_POSE, MIDPOS1, MIDPOS2, MIDPOS3,
            PLACE_STATE, GRAB_POSE, PLACE_STATE_2
        );
    }
}
