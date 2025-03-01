package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    private static final double INTAKE_SPEED = 0.8;
    private static final double TICKS_PER_REV = 2048.0;
    private static final double DISTANCE_PER_REV = 1.0;
    private static final double DISTANCE_PER_TICK = DISTANCE_PER_REV / TICKS_PER_REV;

    public IntakeSubsystem(int canID) {
        intakeMotor = new TalonFX(canID);
    }

    public void startIntake() {
        intakeMotor.set(INTAKE_SPEED);
    }

    public void stopIntake() {
        intakeMotor.set(INTAKE_SPEED);
    }

    public void resetEncoder() {
        intakeMotor.setPosition(0);
    }

    public double getDistance() {
        double ticks = intakeMotor.getPosition().getValueAsDouble();
        return ticks * DISTANCE_PER_TICK;
    }
}
