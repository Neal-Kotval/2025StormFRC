package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.CANrange;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private static final double TICKS_PER_REV = 2048.0;
    private static final double DISTANCE_PER_REV = 1.0;
    private static final double DISTANCE_PER_TICK = DISTANCE_PER_REV / TICKS_PER_REV;
    private final CANrange coralSensor;

    public Intake() {
        intakeMotor = new TalonFX(Constants.CANids.intakeMotor);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        coralSensor = new CANrange(Constants.CANids.coralSensor);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntake(double speed) {
        intakeMotor.set(speed);
    }

    public void resetEncoder() {
        intakeMotor.setPosition(0);
    }

    public double getDistance() {
        double ticks = intakeMotor.getPosition().getValueAsDouble();
        return ticks * DISTANCE_PER_TICK;
    }

    public boolean hasObject() {
        return coralSensor.getIsDetected().getValue();
    }
}