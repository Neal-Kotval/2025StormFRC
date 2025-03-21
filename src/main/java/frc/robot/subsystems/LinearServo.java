package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class LinearServo extends Servo{

    double m_speed;
    double m_length;
    double setPos;
    double curPos;
    PWM pwm;

    public LinearServo(int id, int length, int speed) {
        super(id);
        m_length = length;
        m_speed = speed;
    }

    public void setPosition(double setpoint){
        setPos = clamp(setpoint, 0, m_length);
        setSpeed((setPos/m_length *2)-1);
    }

    public void setLinearSpeed(double speed ){
        setSpeed(speed);
    }

    double lastTime = 0;
    /**
     * Run this method in any periodic function to update the position estimation of your
    servo
    */
    public void updateCurPos(){
        double dt = Timer.getFPGATimestamp() - lastTime;
        if (curPos > setPos + m_speed *dt){
            curPos -= m_speed *dt;
        } else if(curPos < setPos - m_speed *dt){
            curPos += m_speed * dt;
        } else {
            curPos = setPos;
        }
    }

    public boolean isFinished(){
        return curPos == setPos;
    }

    public static int clamp(int value, int low, int high) {
        return Math.max(low, Math.min(value, high));
    }

    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }
}
