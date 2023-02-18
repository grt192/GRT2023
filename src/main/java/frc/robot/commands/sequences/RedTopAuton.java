package frc.robot.commands.sequences;

import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * The red top auton sequences, where the robot deposits its preloaded game piece and picks up
 * and places another.
 */
public class RedTopAuton extends BaseTopAutonSequence {
    public RedTopAuton(
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
