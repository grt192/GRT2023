package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.dropping.AutoAlignCommand;
import frc.robot.controllers.BaseDriveController;
import frc.robot.subsystems.RollerSubsystem.HeldPiece;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;
import frc.robot.vision.SwitchableCamera;

import static frc.robot.Constants.RollerConstants.*;

public class Superstructure extends SubsystemBase {
    private final RollerSubsystem rollerSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;
    private final LEDSubsystem ledSubsystem;

    private final AutoAlignCommand autoAlignCommand;
    private final SwitchableCamera switchableCamera;
    private final BaseDriveController driveController;

    private ElevatorState lastElevatorState;

    public Superstructure(
        RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem, LEDSubsystem ledSubsystem,
        AutoAlignCommand autoAlignCommand, SwitchableCamera switchableCamera, BaseDriveController driveController
    ) {
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
        this.ledSubsystem = ledSubsystem;

        this.autoAlignCommand = autoAlignCommand;
        this.switchableCamera = switchableCamera;
        this.driveController = driveController;
    }

    @Override
    public void periodic() {
        // Update subsystem "piece grabbed" states
        boolean hasPiece = rollerSubsystem.getPiece() != HeldPiece.EMPTY;

        tiltedElevatorSubsystem.pieceGrabbed = hasPiece;
        ledSubsystem.pieceGrabbed = hasPiece;

        // Toggle driver camera when the elevator state switches to / from GROUND.
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

        // Close the roller when the elevator extends below ALLOW_OPEN_EXTENSION_METERS.
        rollerSubsystem.allowOpen = tiltedElevatorSubsystem.getExtensionMeters() >= ALLOW_OPEN_EXTENSION_METERS;

        // Cancel auto-align command if magnitude of drive inputs is greater than 0.15.
        double inputMagnitude = Math.hypot(
            driveController.getForwardPower(),
            driveController.getLeftPower()
        );
        if (inputMagnitude > 0.15) autoAlignCommand.cancel();
    }
}
