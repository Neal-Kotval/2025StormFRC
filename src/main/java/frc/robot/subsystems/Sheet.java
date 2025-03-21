package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.CANrange;

public class Sheet extends SubsystemBase {
    private final Servo servo;

    public Sheet() {
        servo = new Servo(Constants.PWMids.sheet);
    }

    public void setSheetSpeed(double speed) {
        servo.set(speed);
    }
    
}