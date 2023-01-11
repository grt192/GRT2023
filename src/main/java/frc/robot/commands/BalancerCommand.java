package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Balancer;

/** An example command that uses an example subsystem. */
public class BalancerCommand extends CommandBase {

  private final Balancer m_autobal;
  public double returnPower;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BalancerCommand(Balancer subsystem) {
    m_autobal = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_autobal.phase = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    returnPower = m_autobal.calcPower();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}