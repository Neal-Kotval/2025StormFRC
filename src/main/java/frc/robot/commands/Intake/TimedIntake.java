package frc.robot.commands.Intake;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class TimedIntake extends Command {

    private final Intake intake;
    private final double duration;
    private double initialTime;
    private final double power;

    public TimedIntake(Intake intake, double duration,double power) {
        this.intake = intake;
        this.duration = duration;
        this.power =power;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
      
    }

    @Override
    public void execute() {
       intake.setIntakeSpeed(power);
    }

    @Override
    public void end(boolean interrupted) {
       intake.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - initialTime) >= duration;
    }
}
