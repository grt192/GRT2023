package frc.robot.commands.sequences;

import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * The red middle auton sequence, where the robot deposits its preloaded game piece and balances
 * on the charging station.
 */
public class RedBalanceAuton extends BaseBalanceAutonSequence {
    public RedBalanceAuton(
        BaseSwerveSubsystem swerveSubsystem,
        RollerSubsystem rollerSubsystem,
        TiltedElevatorSubsystem tiltedElevatorSubsystem
    ) {
        super(
            swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
            true
        );
    }
}
