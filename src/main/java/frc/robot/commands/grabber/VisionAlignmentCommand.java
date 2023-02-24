package frc.robot.commands.grabber;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.RollerSubsystem.HeldPiece;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.vision.PhotonWrapper;

/**
 * Intakes 1 game piece with the roller mech. This command runs the rollers at a set power
 * until the limit switch is triggered.
 */
public class VisionAlignmentCommand extends CommandBase {
    private final BaseSwerveSubsystem driveSubsystem;

    private double angularPower;
    private final PIDController turnPID;
    PhotonCamera recognitionCamera;

    PhotonPipelineResult result;
    boolean hasTargets;
    PhotonTrackedTarget target;

    public VisionAlignmentCommand(BaseSwerveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;

        recognitionCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
        turnPID = new PIDController(0.2/35, 0.0, 0.0);
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    } 

    @Override
    public void execute() {
        result = recognitionCamera.getLatestResult();
        hasTargets = result.hasTargets();
        target = result.getBestTarget();

        if(hasTargets){
            angularPower = turnPID.calculate(target.getYaw(), 0.0);
        }
        else{
            angularPower = 0.0;
        }
        
        driveSubsystem.setDrivePowers(0.1, 0.0, angularPower, true);    
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Intake done");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(target.getYaw()) <= 1.0;
    }
}
