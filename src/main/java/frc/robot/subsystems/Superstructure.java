package frc.robot.subsystems;

import static frc.robot.Constants.RollerConstants.ALLOW_OPEN_HEIGHT;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.RollerSubsystem.HeldPiece;
import frc.robot.vision.SwitchableCamera;

public class Superstructure extends SubsystemBase {
    private final RollerSubsystem rollerSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;
    private final SwitchableCamera switchableCamera;

    private boolean lastHasPiece = false;

    public Superstructure(RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem, SwitchableCamera switchableCamera) {
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
        this.switchableCamera = switchableCamera;
    }

    @Override
    public void periodic() {
        boolean hasPiece = rollerSubsystem.getPiece() != HeldPiece.EMPTY;

        tiltedElevatorSubsystem.pieceGrabbed = hasPiece;
        if (hasPiece != lastHasPiece) {
            switchableCamera.setCamera(!hasPiece);
        }

        lastHasPiece = hasPiece;

        rollerSubsystem.allowOpen = tiltedElevatorSubsystem.getExtension() >= ALLOW_OPEN_HEIGHT;
    }
}
