package frc.robot.commands.Balancing;

import com.fasterxml.jackson.databind.JsonSerializable.Base;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class PIDSwitchBalancer extends CommandBase {
    private final BaseDrivetrain driveSubsystem;
    private final AHRS ahrs; 
    
    private double initialHeading;
    private double oldPitch;
    private double currentPitch;
    private double deltaAngle;

    PIDController roughPID;
    PIDController finePID;
    PIDController turnPID;

    private double returnDrivePower; // drive power to be returned to DT
    private double returnAngularPower; // angular power to return to DT (for heading correction)

    public boolean reachedStation;
    public boolean passedCenter;
    public boolean waiting;

    public boolean fine; 
    public boolean balanced;

    private final Timer timer;

    public PIDSwitchBalancer(BaseDrivetrain driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.ahrs = driveSubsystem.getAhrs();

        roughPID = new PIDController(0.5/35, 0.0, 0.0); 
        finePID = new PIDController(0.25/22, 0.0, 0.0); 

        turnPID = new PIDController(0.1/5,0.0, 0.0); // kP = max pwr / max err
        timer = new Timer();
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("------------------- Balancer initialized -------------------");
        initialHeading = ahrs.getCompassHeading();
        reachedStation = false;
        passedCenter = false;
        balanced = false;
        waiting = false;

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        returnAngularPower = turnPID.calculate(ahrs.getCompassHeading(), initialHeading); // correct angle of approach

        if(!reachedStation) {
            returnDrivePower = - 0.80;
            if(ahrs.getPitch() <= - 15.0) reachedStation = true;
        }
        else{
            
            currentPitch = ahrs.getPitch();
            deltaAngle = Math.abs(currentPitch - oldPitch);
            fine = ahrs.getPitch() >= -11.0 && deltaAngle <= 0.25;

            if(fine) returnDrivePower = -1 * finePID.calculate(ahrs.getPitch(), 0);
            else returnDrivePower = -1 * roughPID.calculate(ahrs.getPitch(), 0);
           
            System.out.println("DeltaAngle" + deltaAngle);
            
            if(Math.abs(ahrs.getPitch()) <= 2.0 && deltaAngle <= 0.05){
                balanced = true;
                if(driveSubsystem instanceof BaseSwerveSubsystem) ((BaseSwerveSubsystem) driveSubsystem).lockNow();
            }
        }

        if(driveSubsystem instanceof BaseSwerveSubsystem){
            ((BaseSwerveSubsystem) driveSubsystem).setDrivePowers(returnDrivePower, 0.0);
        }
        else driveSubsystem.setDrivePowers(returnDrivePower);

        oldPitch = currentPitch; // set the current angle to old angle so it is accessible for next cycle     
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("------------------- Balancing process finished -------------------");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return reachedStation && balanced;
        // return reachedStation && Math.abs(ahrs.getPitch()) <= 2.0 && stoptimer.hasElapsed(0.20);
    }
}
