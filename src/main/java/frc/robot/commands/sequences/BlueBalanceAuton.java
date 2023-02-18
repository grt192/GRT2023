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
 * The blue middle auton sequence, where the robot deposits its preloaded game piece and balances
 * on the charging station.
 */
public class BlueBalanceAuton extends BaseBalanceAutonSequence {
    private static final Pose2d INITIAL_POSE = new Pose2d(
        Units.inchesToMeters(PlacePosition.BLUE_INIT_X_IN),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(180)
    );
    private static final Pose2d MIDOUTSIDE_POSE = new Pose2d(
        Units.inchesToMeters(215.000),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(180)
    );
    private static final Pose2d OUTSIDEOUTSIDE_POSE = new Pose2d(
        Units.inchesToMeters(250.150),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(90)
    );
    private static final Pose2d BACKOUTSIDE_POSE = new Pose2d(
        Units.inchesToMeters(215.000),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(0)
    );

    private static final PlaceState PLACE_STATE = PlacePosition.B1MID.BLUE;

    public BlueBalanceAuton(
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
