package frc.robot.commands.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class MoveElevator extends Command {
  private final Elevator elevator;
  private final double power;

  public MoveArm(Elevator elevator, double power) {
    this.elevator = elevator;
    this.power = power;
    addRequirements(elevator);
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    elevator.setElevatorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setElevatorSpeed(0);
  }
}