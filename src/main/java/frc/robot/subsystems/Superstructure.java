package frc.robot.subsystems;

import static frc.robot.Constants.RollerConstants.ALLOW_OPEN_HEIGHT;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.RollerSubsystem.HeldPiece;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.vision.SwitchableCamera;

public class Superstructure extends SubsystemBase {
    private final RollerSubsystem rollerSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;
    private final SwitchableCamera switchableCamera;

    private ElevatorState lastState;

    public Superstructure(RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem, SwitchableCamera switchableCamera) {
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
        this.switchableCamera = switchableCamera;
    }

    @Override
    public void periodic() {
        boolean hasPiece = rollerSubsystem.getPiece() != HeldPiece.EMPTY;
        tiltedElevatorSubsystem.pieceGrabbed = hasPiece;

        ElevatorState currentState = tiltedElevatorSubsystem.getState();
        if (currentState != lastState) {
            switch (currentState) {
                case CONE_HIGH:
                case CONE_MID:
                case CUBE_MID:
                case CUBE_HIGH:
                    switchableCamera.setCamera(false);
                    break;
                case GROUND:
                    switchableCamera.setCamera(true);
                    break;
                default:
                    break;
            }
        }
        lastState = currentState;

        rollerSubsystem.allowOpen = tiltedElevatorSubsystem.getExtension() >= ALLOW_OPEN_HEIGHT;
    }
}
