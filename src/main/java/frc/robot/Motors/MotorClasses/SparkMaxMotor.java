package frc.robot.Motors.MotorClasses;
import com.revrobotics.spark.SparkMax;
import frc.robot.Motors.MotorInterface;

public class SparkMaxMotor implements MotorInterface {
    boolean isInverted;
    int motorID;
    SparkMax m_Motor;

    public SparkMaxMotor(int id, boolean inverted) {
        this.isInverted = inverted;
        this.motorID = id;

        this.m_Motor = new SparkMax(id, null);
    }

    public void rotate(double speed) {
        if (this.isInverted) {
            speed=-speed;
        }

        this.m_Motor.set(speed);
     }
}
