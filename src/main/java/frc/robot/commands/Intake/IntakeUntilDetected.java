package frc.robot.commands.Intake;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeUntilDetected extends Command{

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
