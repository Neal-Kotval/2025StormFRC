package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
 */
public class Elevator extends SubsystemBase {
    private final TalonFX masterMotor;
    private final TalonFX followerMotor;
    // Create a PositionVoltage control request (using slot 0 for PID gains).
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    public Elevator() {
        masterMotor = new TalonFX(Constants.CANids.elevatorLeftMotor);
        followerMotor = new TalonFX(Constants.CANids.elevatorRightMotor);
        
        followerMotor.setControl(new Follower(Constants.CANids.elevatorLeftMotor, false));
        
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        masterMotor.getConfigurator().apply(talonFXConfigs);
        
        masterMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);
        
        masterMotor.setPosition(0);
        followerMotor.setPosition(0);
        

    }

    public double getTicks() {
        return masterMotor.getPosition().getValueAsDouble();
    }



    public void setElevatorPositionTicks(double ticks) {


        masterMotor.setControl(m_request.withPosition(ticks));
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
