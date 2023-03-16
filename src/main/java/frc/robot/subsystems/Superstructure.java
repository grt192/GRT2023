package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem.HeldPiece;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;
import frc.robot.vision.SwitchableCamera;

import static frc.robot.Constants.RollerConstants.*;

public class Superstructure extends SubsystemBase {
    private final RollerSubsystem rollerSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;
    private final LEDSubsystem ledSubsystem;

    private final SwitchableCamera switchableCamera;

    private ElevatorState lastElevatorState;

    public Superstructure(
        RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem, LEDSubsystem ledSubsystem,
        SwitchableCamera switchableCamera
    ) {
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
        this.switchableCamera = switchableCamera;
        this.ledSubsystem = ledSubsystem;
    }

    @Override
    public void periodic() {
        boolean hasPiece = rollerSubsystem.getPiece() != HeldPiece.EMPTY;

        tiltedElevatorSubsystem.pieceGrabbed = hasPiece;
        ledSubsystem.pieceGrabbed = hasPiece;

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

        rollerSubsystem.allowOpen = tiltedElevatorSubsystem.getExtensionMeters() >= ALLOW_OPEN_EXTENSION_METERS;
    }
}
