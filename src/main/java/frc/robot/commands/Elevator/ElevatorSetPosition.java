package frc.robot.commands.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;


public class ElevatorSetPosition extends Command {
  private final Elevator elevator;
  private final double targetTicks;
  private final Arm arm;

  public ElevatorSetPosition(Elevator elevator, Arm arm, double targetTicks) {
    this.elevator = elevator;
    this.arm = arm;
    this.targetTicks = targetTicks;
    addRequirements(elevator);
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    if (arm.getTicks() < Constants.TickValues.armSafetyTicks) {
      arm.setArmPositionTicks(Constants.TickValues.armSafetyTicks);
    }
    elevator.setElevatorPositionTicks(targetTicks);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(elevator.getTicks()-targetTicks)<0.5;
  }

  @Override
  public void end(boolean interrupted) {
    double currentTicks = elevator.getTicks();
    elevator.setElevatorPositionTicks(currentTicks);
  }
}