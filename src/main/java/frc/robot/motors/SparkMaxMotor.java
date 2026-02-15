package frc.robot.motors;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SparkMaxMotor implements SparkMaxInterface {
    SparkMax motor;
    
    public SparkMaxMotor(Integer id, MotorType type) {
        motor = new  SparkMax(id, type);
    }

    public void set(double speed) {
        motor.set(speed);
    }

    public void configure(SparkMaxConfig config, ResetMode resetMode, PersistMode persistMode) {
        motor.configure(config, resetMode, persistMode);
    }

    public int getDeviceId() {
        return motor.getDeviceId();
    }

    public MotorType getMotorType() {
        return motor.getMotorType();
    }

    public SparkClosedLoopController getClosedLoopController() {
        return motor.getClosedLoopController();
    }
}
