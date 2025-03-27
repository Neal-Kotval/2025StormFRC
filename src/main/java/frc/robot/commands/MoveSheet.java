package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class MoveSheet extends Command {
  private final Sheet sheet;
  private final double power;

  public MoveSheet(Sheet sheet, double power) {
    this.sheet = sheet;
    this.power = power;
    //addRequirements(sheet);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    sheet.setSheetSpeed(power);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    sheet.setSheetSpeed(0);
  }
}