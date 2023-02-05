package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: think about naming; ths is more of a state manager than a subsystem, and it shouldn't be specific to
// the roller and tilted elevator either
public class RollerToTiltedSubsystem extends SubsystemBase {
    private final RollerSubsystem rollerSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    public RollerToTiltedSubsystem(RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem) {
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
    }

    @Override
    public void periodic() {
        tiltedElevatorSubsystem.pieceGrabbed = rollerSubsystem.hasPiece();
    }
}
