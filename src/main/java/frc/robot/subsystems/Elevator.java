package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

import frc.robot.Constants.ElevatorConstants;

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
    MotionMagicConfigs motionMagicConfigs;

    public Elevator() {
        masterMotor = new TalonFX(Constants.CANids.elevatorLeftMotor);
        followerMotor = new TalonFX(Constants.CANids.elevatorRightMotor);
        followerMotor.setControl(new Follower(Constants.CANids.elevatorLeftMotor, false));

        // Configure PID gains on the master using slot 0.
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = ElevatorConstants.kP;
        config.Slot0.kI = ElevatorConstants.kI;
        config.Slot0.kD = ElevatorConstants.kD;
        config.Slot0.kG = ElevatorConstants.kG;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 30; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 300; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 3000; // Target jerk of 1600 rps/s/s (0.1 seconds)
        // create a Motion Magic request, voltage output

        masterMotor.getConfigurator().apply(config);
        masterMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);
        masterMotor.setPosition(0);
        followerMotor.setPosition(0);

    }

    public double getTicks() {
        return masterMotor.getPosition().getValueAsDouble();
    }

    public Command getCurrentSetter() {
        return new InstantCommand(() -> {
            double currentTicks = this.getTicks();
            final PositionVoltage m_request = new PositionVoltage(0);
            masterMotor.setControl(m_request.withPosition(currentTicks));
        });
    }

    public void setElevatorPosition(double rotations) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        masterMotor.setControl(m_request.withPosition(rotations));
    }

    public void setElevatorPositionDefault(double rotations) {
        final PositionVoltage m_request = new PositionVoltage(0);
        masterMotor.setControl(m_request.withPosition(rotations));
    }

    /**
     * Determines if the elevator has reached the target position.
     * @param targetRotations The desired target position (in rotations).
     * @return true if the current position is within tolerance; false otherwise.
     */

    public void setEncoder(double pos) {
        masterMotor.setPosition(pos);
    }

    public void setElevatorSpeed(double speed) {
        masterMotor.set(speed);
    }

    // public void periodic() {
    //     if (this.getTicks() >= ElevatorConstants.kMaxRotations) {
    //         this.setElevatorSpeed(0);
    //     }
    // }
}
