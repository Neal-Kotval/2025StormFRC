package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class MoveClimb extends Command {
  private final Climb climb;
  private final double power;

  public MoveClimb(Climb climb, double power) {
    this.climb = climb;
    this.power = power;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    climb.setServoSpeed(power);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    climb.setServoSpeed(0);
  }
}