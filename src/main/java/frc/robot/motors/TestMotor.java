package frc.robot.motors;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class TestMotor implements SparkMaxInterface{
    int id;
    MotorType type;
    SparkClosedLoopController closedLoopController;
    public TestMotor(Integer id, MotorType type) {
        this.id = id;
        this.type = type;
    }

    public void set(double speed) {}
    public void configure(SparkMaxConfig config, ResetMode resetMode, PersistMode persistMode) {
        //closedLoopController = config. 
        //we need a way to make the closedLoopController have an actual loop on it (Make a pid interface?)
    }
    public int getDeviceId() {
        return this.id;
    }
    public MotorType getMotorType() {
        return this.type;
    }
    public SparkClosedLoopController getClosedLoopController() {
        return closedLoopController;
    }
}
