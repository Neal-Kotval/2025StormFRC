package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class MoveIntake extends Command {
  private final Intake intake;
  private final double power;

  public MoveIntake(Intake intake, double power) {
    this.intake = intake;
    this.power = power;
    addRequirements(intake);
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    intake.setIntakeSpeed(power);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0);
  }
}