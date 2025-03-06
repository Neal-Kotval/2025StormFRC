package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;


public class setArmPositionNeutral extends Command {
  private final Arm arm;
  private final Elevator elevator;

  public setArmPositionNeutral(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    if (elevator.getTicks() < Constants.TickValues.L1ElevatorTicks/2) {
      arm.setArmPositionTicks(0);
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(arm.getTicks())<0.5;
  }

  @Override
  public void end(boolean interrupted) {
    arm.setArmSpeed(0);
  }
}