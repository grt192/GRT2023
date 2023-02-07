package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * The middle auton sequence, where the robot deposits its preloaded game piece and balances
 * on the charging station.
 */
public class RedBalanceAuton extends BalanceAutonSequence {
    private static final Pose2d INITIAL_POSE = new Pose2d(
        Units.inchesToMeters(PlacePosition.RED_INIT_X_IN),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(0)
    );
    private static final Pose2d OUTSIDE_POSE = new Pose2d(
        Units.inchesToMeters(426.049),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(0)
    );

    public RedBalanceAuton(
        BaseSwerveSubsystem swerveSubsystem,
        RollerSubsystem rollerSubsystem,
        TiltedElevatorSubsystem tiltedElevatorSubsystem
    ) {
        super(
            swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
            INITIAL_POSE, PlacePosition.Red.C3HIGH, OUTSIDE_POSE
        );
    }
}
