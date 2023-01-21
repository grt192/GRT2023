package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerSubsystem;

public class AidenIntakeCommand extends CommandBase {
    private final RollerSubsystem rollerSubsystem;
    private boolean limitswitchUp;
    private double power;

    public AidenIntakeCommand(RollerSubsystem rollerSubsystem){
        power = 0.0;
        this.rollerSubsystem = rollerSubsystem;
        limitswitchUp = rollerSubsystem.isLimit();
        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Roller to intake");
    } 
    
    @Override
    public void execute() {
        if (limitswitchUp){
            power = 0.3;
        }
        else {
            power = 0;
        }
        rollerSubsystem.setRollPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Intake done");
    }
    
    @Override
    public boolean isFinished() {
        return (power == 0);
    }
}
