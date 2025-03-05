package frc.robot.commands.Swerve;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class TimedSwerve extends Command {

    private final Swerve swerve;
    private final double duration;
    private double initialTime;
    private final double xVelocity;
    private final double yVelocity;
    private final SwerveRequest.RobotCentric alignRequest;

    public TimedSwerve(Swerve swerve, double duration, double xVelocity, double yVelocity) {
        this.swerve = swerve;
        this.duration = duration;
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;

        alignRequest = new SwerveRequest.RobotCentric()
            .withDeadband(0.1)
            .withRotationalDeadband(0.1)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
        System.out.println("TimedSwerve initialized");
    }

    @Override
    public void execute() {
        System.out.println("Executing TimedSwerve: X=" + xVelocity + " Y=" + yVelocity);

        SwerveRequest.RobotCentric moveRequest = alignRequest
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withRotationalRate(0);

        swerve.setControl(moveRequest);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("TimedSwerve ended, stopping motion");

        SwerveRequest.RobotCentric stopRequest = alignRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

        swerve.setControl(stopRequest);
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - initialTime) >= duration;
    }
}
