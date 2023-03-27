package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

    private static final double CANCEL_AUTO_ALIGN_POWER = 0.1;

    private ElevatorState lastElevatorState;

    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Driver");
    private final GenericEntry cancellingEntry = shuffleboardTab.add("Align cancel", false)
        .withPosition(9, 3)
        .getEntry();

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
        ledSubsystem.setColorSensorOff(!rollerSubsystem.colorSensorConnected());

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
        double translateMagnitude = Math.hypot(
            driveController.getForwardPower(),
            driveController.getLeftPower()
        );
        double turnMagnitude = Math.abs(driveController.getRotatePower());
        boolean cancelling = translateMagnitude > CANCEL_AUTO_ALIGN_POWER || turnMagnitude > CANCEL_AUTO_ALIGN_POWER;
        
        if (cancelling) autoAlignCommand.cancel();
        cancellingEntry.setBoolean(cancelling);
    }
}
