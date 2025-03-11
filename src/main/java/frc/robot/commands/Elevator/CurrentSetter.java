package frc.robot.commands.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;


public class CurrentSetter extends Command {
  private final Elevator elevator;

  public CurrentSetter(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    elevator.setElevatorPositionDefault(elevator.getTicks());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    
  }
}