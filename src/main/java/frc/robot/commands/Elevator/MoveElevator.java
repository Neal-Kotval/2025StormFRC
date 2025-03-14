package frc.robot.commands.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;


public class MoveElevator extends Command {
  private final Elevator elevator;
  private final double power;
  private final Arm arm;

  public MoveElevator(Elevator elevator, Arm arm, double power) {
    this.elevator = elevator;
    this.arm = arm;
    this.power = power;
   
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    if (arm.getTicks() < Constants.TickValues.armSafetyTicks && power>0) {
      arm.setArmPositionTicks(Constants.TickValues.armSafetyTicks);
    }
    elevator.setElevatorSpeed(power);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    double currentTicks = elevator.getTicks();
    elevator.setElevatorPositionTicks(currentTicks);
    //arm.setArmSpeed(0);
    // elevator.setElevatorSpeed(0);
  }
}