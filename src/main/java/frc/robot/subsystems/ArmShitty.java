package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;;

public class ArmShitty {
    private SparkMax motor = new SparkMax(0, MotorType.kBrushless);
    private AbsoluteEncoder encoder = motor.getAbsoluteEncoder();

    
}