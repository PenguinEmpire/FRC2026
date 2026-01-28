package frc.robot.Modules;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class JointModule {
    private final String name;
    private final SparkMax motor;
    private final SparkClosedLoopController pidController;

    private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(20, 7.5));
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    
    private double targetPosition;

    private double kDt = 0.2;

    public JointModule(String motorName, int id) {
        this.name = motorName;
        this.motor = new SparkMax(id, MotorType.kBrushless);

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(0, 0, 0)
                .positionWrappingEnabled(true)
                .positionWrappingMinInput(0)
                .positionWrappingMaxInput(2 * Math.PI)
                .outputRange(-1, 1);

        motorConfig.encoder.positionConversionFactor(2 * Math.PI);
        // Apply the initial configuration
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController = motor.getClosedLoopController();
    }

    public void setPosition(double newTargetPosition) {
        this.targetPosition = newTargetPosition;
        m_goal = new TrapezoidProfile.State(targetPosition, 0);
    }

    public double getPosition() {
        AbsoluteEncoder absEncoder = motor.getAbsoluteEncoder();

       
        return absEncoder.getPosition();
    }

    public void periodic() {
        //double currentPos = getPosition();
        //double outputFF = feedforward.calculate(m_setpoint.position, m_setpoint.velocity);
        m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
        pidController.setSetpoint(m_setpoint.position, SparkBase.ControlType.kPosition);
    }
}