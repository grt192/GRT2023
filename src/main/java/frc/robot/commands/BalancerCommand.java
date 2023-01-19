package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;

public class BalancerCommand extends CommandBase {
    private final BaseDrivetrain driveSubsystem;
    private final AHRS ahrs; 

    private double returnPower; //power to be returned to DT
    private boolean reachedStation;

    public BalancerCommand(BaseDrivetrain driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.ahrs = driveSubsystem.getAhrs();

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("---------------------- Balancer initialized ----------------------");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!reachedStation) {
            returnPower = 0.25;
            if(ahrs.getPitch() >= 4.0) reachedStation = true;
        }
        else{
            returnPower  = 0.01142857143 * ahrs.getPitch(); // MAX_ERR * Kp = MAX_PWR   35 * Kp = 0.4    
        }

        driveSubsystem.setDrivePowers(returnPower);
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("------------------ Balancing process finished ---------------------");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(ahrs.getPitch()) <= 2.0;
    }
}
