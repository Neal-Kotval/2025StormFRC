package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage; 
/**
 * This subsystem controls an elevator mechanism using two Falcon 500 motors.
 * The master motor runs a closed-loop PID based on its integrated relative encoder,
 * and the follower motor mirrors the master.
 * 
 * Because Phoenix 6 does not include a built-in gravity compensation parameter (kG),
 * we add an arbitrary feedforward (GRAVITY_FF) to the control request to help hold the elevator.
 */
public class Elevator extends SubsystemBase {
    private final TalonFX masterMotor;
    private final TalonFX followerMotor;
    // Create a PositionVoltage control request (using slot 0 for PID gains).
    private final PositionVoltage positionControl = new PositionVoltage(0);

    // PID gains for closed-loop position control (tune these values for your mechanism)
    private static final double kP = 2;
    private static final double kI = 0.0;
    private static final double kD = 0.1;
    private static final double kG = 0;

    // Conversion constants:
    // The Falcon 500's integrated encoder produces 2048 ticks per revolution.
    private static final double TICKS_PER_REV = 2048.0;
    // The gear ratio between the motor and the elevator (adjust if you have gearing)
    private static final double GEAR_RATIO = 1.0;


    public Elevator() {
        masterMotor = new TalonFX(Constants.CANids.elevatorLeftMotor);
        followerMotor = new TalonFX(Constants.CANids.elevatorRightMotor);



        followerMotor.setControl(new Follower(Constants.CANids.elevatorLeftMotor, false));
        
        // TalonFXConfiguration followerConfiguration = new TalonFXConfiguration();
        // followerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // followerMotor.getConfigurator().apply(followerConfiguration);
        MotionMagicConfigs motionMagicConfigs;

        // Configure PID gains on the master using slot 0.
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kG = kG;

        // motionMagicConfigs = config.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = 3; // Target cruise velocity of 80 rps
        // motionMagicConfigs.MotionMagicAcceleration = 3; // Target acceleration of 160 rps/s (0.5 seconds)
        // motionMagicConfigs.MotionMagicJerk = 0.2; // Target jerk of 1600 rps/s/s (0.1 seconds)
        // // create a Motion Magic request, voltage output


        masterMotor.getConfigurator().apply(config);
        masterMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);
        masterMotor.setPosition(0);
        followerMotor.setPosition(0);

    }

    public double getTicks() {
        return masterMotor.getPosition().getValueAsDouble();
    }


    public void setElevatorPositionTicks(double ticks) {

        // final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        // masterMotor.setControl(m_request.withPosition(ticks));

        // positionControl.Velocity = 0.2;

        masterMotor.setControl(positionControl.withPosition(ticks));
    }

    public void setElevatorDown(double ticks) {

    }

    /**
     * Determines if the elevator has reached the target position.
     * @param targetRotations The desired target position (in rotations).
     * @return true if the current position is within tolerance; false otherwise.
     */

    public void setEncoder(double pos) {
        masterMotor.setPosition(pos);
    }

    /**
     * Stops the elevator by setting the motor output to zero.
     */
    public void stopElevator() {
        masterMotor.set(0);
    }

    public void setElevatorSpeed(double speed) {
        masterMotor.set(speed);
    }
}
