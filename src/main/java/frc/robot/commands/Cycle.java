package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;


public class Cycle extends Command {
  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;

  public Cycle(Elevator elevator, Arm arm, Intake intake) {
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    addRequirements(elevator, arm, intake);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    if (arm.getTicks() < Constants.TickValues.armSafetyTicks) {
      arm.setArmPositionTicks(Constants.TickValues.armSafetyTicks);
    }
    elevator.setElevatorPosition(Constants.TickValues.L3Ticks);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(Constants.TickValues.L3Ticks-elevator.getTicks()) < 1);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setElevatorSpeed(0);
    arm.setArmSpeed(0);
    intake.setIntakeSpeed(0);
  }
}