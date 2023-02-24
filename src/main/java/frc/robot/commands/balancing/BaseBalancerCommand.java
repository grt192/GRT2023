package frc.robot.commands.balancing;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public abstract class BaseBalancerCommand extends CommandBase {
    protected final BaseDrivetrain driveSubsystem;
    protected final AHRS ahrs;

    public BaseBalancerCommand(BaseDrivetrain driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.ahrs = driveSubsystem.getAhrs();

        addRequirements(driveSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("------------------- Balancing process finished -------------------");
        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            BaseSwerveSubsystem swerveSubsystem = (BaseSwerveSubsystem) driveSubsystem;
            swerveSubsystem.setChargingStationLocked(true);
            swerveSubsystem.lockNow();
        }
    }
}
