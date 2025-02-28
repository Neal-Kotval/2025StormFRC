package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This subsystem controls an elevator mechanism using two Falcon 500 motors.
 * The master motor runs a closed-loop PID based on its integrated relative encoder,
 * and the follower motor mirrors the master.
 * 
 * Because Phoenix 6 does not include a built-in gravity compensation parameter (kG),
 * we add an arbitrary feedforward (GRAVITY_FF) to the control request to help hold the elevator.
 */
public class Arm extends SubsystemBase {
    private final TalonFX armMotor;
    // Create a PositionVoltage control request (using slot 0 for PID gains).
    private final PositionVoltage positionControl = new PositionVoltage(0);

    // PID gains for closed-loop position control (tune these values for your mechanism)
    private static final double kP = 2.0;
    private static final double kI = 0.0;
    private static final double kD = 0.1;
    private static final double kG = 0.05;

    // Conversion constants:
    // The Falcon 500's integrated encoder produces 2048 ticks per revolution.
    private static final double TICKS_PER_REV = 2048.0;
    // The gear ratio between the motor and the elevator (adjust if you have gearing)
    private static final double GEAR_RATIO = 1.0;
    // Tolerance (in ticks) to decide if the elevator is “at” the target.
    private static final double TOLERANCE_TICKS = 10.0;

    public Arm() {
        armMotor = new TalonFX(Constants.CANids.armMotor);

        // Configure PID gains on the master using slot 0.
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kG = kG;
        armMotor.getConfigurator().apply(config);
    }

    /**
     * Reads the current elevator position in rotations.
     * @return The elevator position (in rotations).
     */
    public double getArmPosition() {
        // Get the sensor position (in ticks) and convert to rotations.
        double ticks = armMotor.getPosition().getValueAsDouble();
        return (ticks / TICKS_PER_REV) * GEAR_RATIO;
    }

    /**
     * Sets the target position for the elevator (in rotations) using closed-loop control.
     * This method converts the target position to sensor ticks and adds a feedforward for gravity.
     * @param targetRotations The desired elevator position (in rotations).
     */
    public void setArmPosition(double targetRotations) {
        // Convert the desired position from rotations to sensor ticks.
        double targetTicks = (targetRotations / GEAR_RATIO) * TICKS_PER_REV;
        // Send the closed-loop control request with the target and add our manually tuned gravity feedforward.
        armMotor.setControl(positionControl.withPosition(targetTicks)
                                               .withFeedForward(kG));
    }

    /**
     * Determines if the elevator has reached the target position.
     * @param targetRotations The desired target position (in rotations).
     * @return true if the current position is within tolerance; false otherwise.
     */
    public boolean atTargetPosition(double targetRotations) {
        double targetTicks = (targetRotations / GEAR_RATIO) * TICKS_PER_REV;
        double currentTicks = armMotor.getPosition().getValueAsDouble();
        return Math.abs(currentTicks - targetTicks) < TOLERANCE_TICKS;
    }

    public void setEncoder(double pos) {
        armMotor.setPosition(pos);
    }

    /**
     * Stops the elevator by setting the motor output to zero.
     */
    public void stopArm() {
        armMotor.set(0);
    }

    public void setArmSpeed(double speed) {
        armMotor.set(speed);
    }
}
