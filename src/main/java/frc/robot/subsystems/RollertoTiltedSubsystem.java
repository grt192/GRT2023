package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class RollertoTiltedSubsystem extends SubsystemBase{
    private final RollerSubsystem rollerSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    public RollertoTiltedSubsystem(RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem){
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
    }

    @Override
    public void periodic() {
        if (!rollerSubsystem.isLimit()){
            tiltedElevatorSubsystem.pieceGrabbed = true;
        }
        else{
            tiltedElevatorSubsystem.pieceGrabbed = false;
        }
    }
}
