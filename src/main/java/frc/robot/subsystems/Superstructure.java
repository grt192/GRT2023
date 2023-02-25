package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.RollerSubsystem.HeldPiece;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.vision.SwitchableCamera;

import static frc.robot.Constants.RollerConstants.*;

public class Superstructure extends SubsystemBase {
    private final RollerSubsystem rollerSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    private final SwitchableCamera switchableCamera;

    private ElevatorState lastElevatorState;

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
        if (currentState != lastElevatorState) {
            switch (currentState) {
                case CONE_HIGH, CONE_MID, CUBE_MID, CUBE_HIGH ->
                    switchableCamera.setCamera(false);
                case GROUND ->
                    switchableCamera.setCamera(true);
                default -> {}
            }
        }
        lastElevatorState = currentState;

        rollerSubsystem.allowOpen = tiltedElevatorSubsystem.getExtension() >= ALLOW_OPEN_HEIGHT;
    }
}
