package frc.robot.commands.Intake;
import frc.robot.subsystems.Intake;

public class IntakeUntilDetected {

    private final Intake intake; 
    private final double power;

    public IntakeUntilDetected(Intake intake, double power) {
        this.intake = intake;
        this.power = power;
    }

    public void initialize() {

    }

    public void execute() {
        intake.setIntakeSpeed(power);
    }

    public boolean isFinished() {
        return intake.hasObject();
    }

    public void end() {
        intake.setIntakeSpeed(0);
    }
}
