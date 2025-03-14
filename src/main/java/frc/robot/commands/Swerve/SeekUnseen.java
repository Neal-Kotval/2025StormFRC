package frc.robot.commands.Swerve;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.LimelightHelpers;
public class SeekUnseen extends Command {

    private final Swerve swerve;
    private final SwerveRequest.RobotCentric alignRequest;

    public SeekUnseen(Swerve swerve) {
        this.swerve = swerve;

        alignRequest = new SwerveRequest.RobotCentric()
            .withDeadband(0.1)
            .withRotationalDeadband(0.1)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
     
    }

    @Override
    public void execute() {

        SwerveRequest.RobotCentric moveRequest = alignRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(1.5);
        swerve.setControl(moveRequest);

        
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended, stopping motion");

        SwerveRequest.RobotCentric stopRequest = alignRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);
        
        swerve.setControl(stopRequest);
    }

    @Override
    public boolean isFinished() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0)==1;
    }
}
