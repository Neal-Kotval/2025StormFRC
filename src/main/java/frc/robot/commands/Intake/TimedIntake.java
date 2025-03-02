package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class TimedIntake extends Command {

    private Intake intake;
    private double initialTime;
    private double duration;

    public TimedIntake(Intake intake, double duration) {
        this.intake = intake;
        this.duration = duration;
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.setIntakeSpeed(0.2);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0);;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp()-initialTime) >= duration;
    }

}