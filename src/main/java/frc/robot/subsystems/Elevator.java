package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.CANids;

import com.ctre.phoenix6.controls.ControlRequest;
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

    // PID gains for closed-loop position control (tune these values for your mechanism)
    private static final double kP = ElevatorConstants.kP;
    private static final double kI = ElevatorConstants.kI;
    private static final double kD = ElevatorConstants.kD;
    private static final double kG = ElevatorConstants.kG; //tune later
    private static final GravityTypeValue GravityType = ElevatorConstants.GravityType;


    public Elevator() {
        masterMotor = new TalonFX(CANids.elevatorLeftMotor);
        followerMotor = new TalonFX(CANids.elevatorRightMotor);
        followerMotor.setControl(new Follower(CANids.elevatorLeftMotor, false));
        
        // TalonFXConfiguration followerConfiguration = new TalonFXConfiguration();
        // followerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // followerMotor.getConfigurator().apply(followerConfiguration);

        // Configure PID gains on the master using slot 0.
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kG = kG;
        config.Slot0.GravityType = GravityType;

        masterMotor.getConfigurator().apply(config);
        masterMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);
        masterMotor.setPosition(0);
        followerMotor.setPosition(0);

        // set Motion Magic settings
        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0.1; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 0.1; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 0.1; // Target jerk of 1600 rps/s/s (0.1 seconds)

    }

    public double getRotations() {
        return masterMotor.getPosition().getValueAsDouble();
    }

    public void periodic() {
        if (this.getRotations() >= ElevatorConstants.kMaxRotations) {
            this.setElevatorSpeed(0);
        }
    }

    public void moveTo(double rotations) {
        // create a Motion Magic request, voltage output
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

        // set target position to 100 rotations
        masterMotor.setControl(m_request.withPosition(rotations));
    }

    public void setEncoder(double pos) {
        masterMotor.setPosition(pos);
    }

    public void stopElevator() {
        masterMotor.set(0);
    }

    public void setElevatorSpeed(double speed) {
        masterMotor.set(speed);
    }
}
