package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class Load extends Command {
  private final Intake intake;
  private final double power;
  private double initialTime;
  
    public Load(Intake intake, double power,double initalTime) {
      this.intake = intake;
      this.power = power;
      this.initialTime = initalTime;
      addRequirements(intake);
    }
  
  
    @Override
    public void initialize() {
      initialTime = Timer.getFPGATimestamp();
    
  }

  @Override
  public void execute() {
    if(Timer.getFPGATimestamp()-initialTime>=0.1){
        intake.setIntakeSpeed(-power);
    }
    intake.setIntakeSpeed(power);
  }

  @Override
  public boolean isFinished() {
    return intake.hasObject();
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0);
  }
}