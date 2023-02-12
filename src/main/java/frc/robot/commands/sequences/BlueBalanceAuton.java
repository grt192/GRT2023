package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.positions.PlacePosition;
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
    private static final Pose2d OUTSIDE_POSE = new Pose2d(
        Units.inchesToMeters(224.701),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(0)
    );

    private static final PlacePosition PLACE_STATE = PlacePosition.Blue.B2MID;

    public BlueBalanceAuton(
        BaseSwerveSubsystem swerveSubsystem,
        RollerSubsystem rollerSubsystem,
        TiltedElevatorSubsystem tiltedElevatorSubsystem
    ) {
        super(
            swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
            INITIAL_POSE, PLACE_STATE, OUTSIDE_POSE
        );
    }
}
