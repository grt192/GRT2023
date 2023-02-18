package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.positions.PlacePosition;
import frc.robot.positions.PlacePosition.PlaceState;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * The red middle auton sequence, where the robot deposits its preloaded game piece and balances
 * on the charging station.
 */
public class RedBalanceAuton extends BaseBalanceAutonSequence {
    private static final Pose2d INITIAL_POSE = new Pose2d(
        Units.inchesToMeters(PlacePosition.RED_INIT_X_IN),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(0)
    );
    private static final Pose2d MIDOUTSIDE_POSE = new Pose2d(
        Units.inchesToMeters(401.150),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(180)
    );
    private static final Pose2d OUTSIDEOUTSIDE_POSE = new Pose2d(
        Units.inchesToMeters(436.300),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(90)
    );
    private static final Pose2d BACKOUTSIDE_POSE = new Pose2d(
        Units.inchesToMeters(401.150),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(0)
    );

    private static final PlaceState PLACE_STATE = PlacePosition.C3HIGH.RED;

    public RedBalanceAuton(
        BaseSwerveSubsystem swerveSubsystem,
        RollerSubsystem rollerSubsystem,
        TiltedElevatorSubsystem tiltedElevatorSubsystem
    ) {
        super(
            swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
            INITIAL_POSE, PLACE_STATE, MIDOUTSIDE_POSE, OUTSIDEOUTSIDE_POSE, BACKOUTSIDE_POSE
        );
    }
}
