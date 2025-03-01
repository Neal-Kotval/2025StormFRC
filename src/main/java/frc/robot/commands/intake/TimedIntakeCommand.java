package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class TimedIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double durationSeconds;
    private final Timer timer = new Timer();

    public TimedIntakeCommand(IntakeSubsystem subsystem, double durationSeconds) {
        this.intakeSubsystem = subsystem;
        this.durationSeconds = durationSeconds;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        intakeSubsystem.resetEncoder();
    }

    @Override
    public void execute() {
        intakeSubsystem.startIntake();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        timer.stop();
    }
}
