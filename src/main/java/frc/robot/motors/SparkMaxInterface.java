package frc.robot.motors;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;

public interface SparkMaxInterface {
    public void set(double speed);
    public void configure(SparkMaxConfig config, ResetMode resetMode, PersistMode persistMode);
    public int getDeviceId();
    public MotorType getMotorType();
    public SparkClosedLoopController getClosedLoopController();
}
