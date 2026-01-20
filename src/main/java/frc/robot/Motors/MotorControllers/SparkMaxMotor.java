package frc.robot.Motors.MotorControllers;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Motors.MotorInterface;

public class SparkMaxMotor implements MotorInterface {
    boolean isInverted;
    int motorID;
    Spark m_Motor;

    public SparkMaxMotor(int id, boolean inverted) {
        this.isInverted = inverted;
        this.motorID = id;

        this.m_Motor = new Spark(motorID);
    }

    public void rotate(double speed) {
        if (this.isInverted) {
            speed=-speed;
        }

        this.m_Motor.set(speed);
     }
}
