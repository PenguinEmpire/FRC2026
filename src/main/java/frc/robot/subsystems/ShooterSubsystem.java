// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;  

public class ShooterSubsystem extends SubsystemBase {
  private SparkMax m_leftShooterMotor = new SparkMax(Constants.ShooterConstants.kLeftShooterMotorID, MotorType.kBrushless);
  private SparkMax m_middleShooterMotor = new SparkMax(Constants.ShooterConstants.kMiddleShooterMotorID, MotorType.kBrushless);
  private SparkMax m_rightShooterMotor = new SparkMax(Constants.ShooterConstants.kRightShooterMotorID, MotorType.kBrushless);

  private SparkMax m_jointMotor = new SparkMax(Constants.ShooterConstants.kJointMotorID, MotorType.kBrushless);
  private PS5Controller joystick = new PS5Controller(0);

  private final SparkClosedLoopController pidController;

  private VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

    private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(SmartDashboard.getNumber("ShooterJointMaxVelocity", 20), SmartDashboard.getNumber("ShooterJointMaxAcceleration", 7.5)));
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private double kDt = SmartDashboard.getNumber("ShooterkDT", 0.2);

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    SmartDashboard.putNumber("ShooterkDT", 0.2);
    SmartDashboard.putNumber("ShooterJointMaxVelocity", 20);
    SmartDashboard.putNumber("ShooterJointMaxAcceleration", 7.5);
    SmartDashboard.putNumber("ShooterJointP", 0);
    SmartDashboard.putNumber("ShooterJointI", 0);
    SmartDashboard.putNumber("ShooterJointD", 0);
    SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(SmartDashboard.getNumber("ShooterJointP", 0), SmartDashboard.getNumber("ShooterJointI", 0), SmartDashboard.getNumber("ShooterJointD", 0))
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

  private double getTargetAngle() {
    if (m_VisionSubsystem.HasTarget()) {
      double ty = m_VisionSubsystem.GetYAngle();
      return ty*ty+ty+1; //Whatever the line of best fit is
    } else {
      return 0; //Whatever the default position is
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_VisionSubsystem.HasTarget() && joystick.getR1Button()) {
      m_goal = new TrapezoidProfile.State(getTargetAngle(), 0);
      m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
      pidController.setSetpoint(m_setpoint.position, SparkBase.ControlType.kPosition);
    }

    SmartDashboard.putNumber("HoodTargetPosition", m_goal.position);
    SmartDashboard.putNumber("HoodCurrentPosition", m_setpoint.position);
    //SmartDashboard.putNumber("HoodCurrentPosition", m_jointMotor.getEncoder().getPosition()); //Alternative way to get position
  }

  public void MoveHood(double speed) {
    m_jointMotor.set(speed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void enableShooter(boolean active) {
    if (active) {
      m_leftShooterMotor.set(Constants.ShooterConstants.kShooterVelocity);
      m_middleShooterMotor.set(Constants.ShooterConstants.kShooterVelocity);
      m_rightShooterMotor.set(Constants.ShooterConstants.kShooterVelocity);
    }
  }
}
