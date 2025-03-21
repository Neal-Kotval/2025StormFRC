package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;


public class ArmSetPosition extends Command {
  private final double targetTicks;
  private final Arm arm;

  public ArmSetPosition(Elevator elevator, Arm arm, double targetTicks) {
    this.arm = arm;
    this.targetTicks = targetTicks;
    addRequirements(elevator);
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    arm.setArmPositionTicks(targetTicks);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(arm.getTicks()-targetTicks)<0.5;
  }

  @Override
  public void end(boolean interrupted) {
    arm.setArmSpeed(0);
  }
}