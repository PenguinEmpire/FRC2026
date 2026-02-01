// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.LogManager;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_jointMotor = new SparkMax(Constants.IntakeConstants.kIntakeJointMotorID, MotorType.kBrushless);;
  private final SparkClosedLoopController pidController;
  
  private PS5Controller joystick = new PS5Controller(0);

  private final TrapezoidProfile m_profile =
    new TrapezoidProfile(new TrapezoidProfile.Constraints(20, 7.5));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    
  private double targetPosition;

  private double kDt = 0.2;

  private final SparkMax m_RollerMotor = new SparkMax(Constants.IntakeConstants.kIntakeRollerMotorID, MotorType.kBrushless);
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
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
        m_jointMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController = m_jointMotor.getClosedLoopController();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setPosition(double newTargetPosition) {
      this.targetPosition = newTargetPosition;
      m_goal = new TrapezoidProfile.State(targetPosition, 0);
  }

  public double getPosition() {
      RelativeEncoder relativeEncoder = m_jointMotor.getEncoder();

       
      return relativeEncoder.getPosition();
  }

  @Override
  public void periodic() {
    //double currentPos = getPosition();
    //double outputFF = feedforward.calculate(m_setpoint.position, m_setpoint.velocity);
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
    if (!joystick.getCircleButton() && !joystick.getCrossButton()) {
       pidController.setSetpoint(m_setpoint.position, SparkBase.ControlType.kPosition);
    }

    SmartDashboard.putNumber("IntakeTargetPosition", m_goal.position);
    SmartDashboard.putNumber("IntakeCurrentPosition", m_setpoint.position);
    //SmartDashboard.putNumber("IntakeTargetPosition", getPosition());
  }

  public void ManualMove(double speed) {
      m_jointMotor.set(speed);
  }
  public void spinRollers(boolean active) {
    if (active) {
      m_RollerMotor.set(Constants.IntakeConstants.kRollerMotorPower);
    }
  }
}
