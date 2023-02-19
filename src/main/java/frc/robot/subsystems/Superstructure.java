package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.RollerSubsystem.HeldPiece;

public class Superstructure extends SubsystemBase {
    private final RollerSubsystem rollerSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    public Superstructure(RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem) {
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
    }

    @Override
    public void periodic() {
        tiltedElevatorSubsystem.pieceGrabbed = rollerSubsystem.getPiece() != HeldPiece.EMPTY;
        rollerSubsystem.allowOpen = tiltedElevatorSubsystem.getExtension() >= Units.inchesToMeters(20);
    }
}
