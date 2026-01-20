package frc.robot.Motors.MotorClasses;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Motors.MotorInterface;

public class TalonMotor implements MotorInterface {
    boolean isInverted;
    int motorID;
    Talon m_Motor;

    public TalonMotor(int id, boolean inverted) {
        this.isInverted = inverted;
        this.motorID = id;

        this.m_Motor = new Talon(id);
    }

    public void rotate(double speed) {
        if (this.isInverted) {
            speed=-speed;
        }

        this.m_Motor.set(speed);
     }
}
