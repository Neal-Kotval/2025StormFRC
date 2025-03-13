package frc.robot.commands.Intake;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeUntilDetected extends Command {

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
        double initialTime =  Timer.getFPGATimestamp();
        while (intake.hasObject()) {
            continue;
        }
        
        return intake.hasObject() && Timer.getFPGATimestamp()-initialTime >= 0.1;
    }

    public void end(boolean interrupted) {
        // double startTime = Timer.getFPGATimestamp();
        // while(Math.abs(Timer.getFPGATimestamp()-startTime) < 0.15) {
        //     intake.setIntakeSpeed(-0.2);
        // }
        intake.setIntakeSpeed(0);
    }
}
