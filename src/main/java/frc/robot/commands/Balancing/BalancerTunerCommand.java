package frc.robot.commands.Balancing;

import com.fasterxml.jackson.databind.JsonSerializable.Base;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class BalancerTunerCommand extends CommandBase {
    private final BaseDrivetrain driveSubsystem;
    private final AHRS ahrs; 
    
    private double initialHeading;
    private double oldPitch;
    private double currentPitch;
    private double angularAcceleration;

    PIDController drivePID;
    PIDController turnPID;

    private double returnDrivePower; // drive power to be returned to DT
    private double returnAngularPower; // angular power to return to DT (for heading correction)

    public boolean reachedStation;
    public boolean passedCenter;
    public boolean waiting;
    public boolean balanced;

    private final Timer timer;

    public BalancerTunerCommand(BaseDrivetrain driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.ahrs = driveSubsystem.getAhrs();

        drivePID = new PIDController(0.3/35, 0.0, 0.0); // no deriv successful
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
        
        // get initial angle

        // move set amount forward


        driveSubsystem.setDrivePowers(0.0);   
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
