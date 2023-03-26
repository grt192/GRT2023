package frc.robot.commands.auton;

import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class PreloadedOnlyAutonSequence extends BaseAutonSequence {
    /**
     * Preloaded-only auton sequence. This only places the preloaded game piece and
     * does not taxi.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param initialPosition The initial place position of the sequence.
     * @param isRed Whether this is a red auton path.
     */
    public PreloadedOnlyAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        PlacePosition initialPosition, boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, initialPosition, isRed);
    }
}
