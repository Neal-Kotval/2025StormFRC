package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class EncoderIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double targetDistance;

    public EncoderIntakeCommand(IntakeSubsystem subsystem, double targetDistance) {
        this.intakeSubsystem = subsystem;
        this.targetDistance = targetDistance;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.resetEncoder();
    }

    @Override
    public void execute() {
        intakeSubsystem.startIntake();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getDistance() >= targetDistance;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
    }
}
