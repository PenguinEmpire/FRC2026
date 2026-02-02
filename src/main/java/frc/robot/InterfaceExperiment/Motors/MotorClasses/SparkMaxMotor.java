package frc.robot.InterfaceExperiment.Motors.MotorClasses;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.InterfaceExperiment.Motors.MotorInterface;

public class SparkMaxMotor implements MotorInterface {
    boolean isInverted;
    int motorID;
    SparkMax m_Motor;

    public SparkMaxMotor(int id, boolean inverted) {
        this.isInverted = inverted;
        this.motorID = id;

        this.m_Motor = new SparkMax(this.motorID, MotorType.kBrushless);
    }

    public void rotate(double speed) {
        if (this.isInverted) {
            speed=-speed;
        }

        this.m_Motor.set(speed);
     }
}
