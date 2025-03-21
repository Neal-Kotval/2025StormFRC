package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private LinearServo servo;

    public Climb() {
        servo = new LinearServo(Constants.PWMids.climb, 50, 1);
    }

    public void periodic() {
        servo.updateCurPos();
    }

    public void setPosition(double rotations) {
        while (!servo.isFinished()) {
            servo.setPosition(rotations);
        }
    }

    public void setServoSpeed(double speed) {
        servo.setLinearSpeed(speed);
    }
}
