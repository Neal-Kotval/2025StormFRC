package frc.robot.commands.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class MoveArm extends Command {
  private final Arm arm;
  private final double power;

  public MoveArm(Arm arm, double power) {
    this.arm = arm;
    this.power = power;
    addRequirements(arm);
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    arm.setArmSpeed(power);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
  }
}