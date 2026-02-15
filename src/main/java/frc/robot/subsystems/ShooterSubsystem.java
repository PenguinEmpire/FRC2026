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
import frc.robot.motors.SparkMaxInterface;
import frc.robot.motors.SparkMaxMotor;
import frc.robot.motors.TestMotor;
import frc.robot.subsystems.vision.LimelightVision;  

public class ShooterSubsystem extends SubsystemBase {
  private SparkMaxInterface m_leftShooterMotor = new SparkMaxMotor(Constants.ShooterConstants.kLeftShooterMotorID, MotorType.kBrushless);
  private SparkMaxInterface m_middleShooterMotor = new SparkMaxMotor(Constants.ShooterConstants.kMiddleShooterMotorID, MotorType.kBrushless);
  private SparkMaxInterface m_rightShooterMotor = new SparkMaxMotor(Constants.ShooterConstants.kRightShooterMotorID, MotorType.kBrushless);

  private SparkMaxInterface m_jointMotor = new SparkMaxMotor(Constants.ShooterConstants.kJointMotorID, MotorType.kBrushless);

  private SparkMaxInterface[] motorList = {m_leftShooterMotor, m_middleShooterMotor, m_rightShooterMotor, m_jointMotor};
  private PS5Controller joystick = new PS5Controller(0);

  private final SparkClosedLoopController pidController;

  private VisionSubsystem m_VisionSubsystem = new VisionSubsystem(new LimelightVision());

    private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(SmartDashboard.getNumber("ShooterJointMaxVelocity", 20), SmartDashboard.getNumber("ShooterJointMaxAcceleration", 7.5)));
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private double kDt = SmartDashboard.getNumber("ShooterkDT", 0.2);

    double kP = SmartDashboard.getNumber("ShooterJointP", 0);
    double kI = SmartDashboard.getNumber("ShooterJointI", 0);
    double kD = SmartDashboard.getNumber("ShooterJointD", 0);

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem(boolean testing) {
    if (testing) {
      //make the motors the test motors
      for (int i = 0; i<motorList.length; i++) {
        motorList[i] = new TestMotor(motorList[i].getDeviceId(), motorList[i].getMotorType());
      }
    }
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
                .pid(kP, kI, kD)
                .positionWrappingEnabled(true)
                .positionWrappingMinInput(0)
                .positionWrappingMaxInput(2 * Math.PI)
                .outputRange(-1, 1);

        motorConfig.encoder.positionConversionFactor(2 * Math.PI);
        // Apply the initial configuration
        m_jointMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController = m_jointMotor.getClosedLoopController();
  }

  public ShooterSubsystem() {
    this(false);
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
    if (m_VisionSubsystem.HasTarget()) {
      m_goal = new TrapezoidProfile.State(getTargetAngle(), 0);
      m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
      pidController.setSetpoint(m_setpoint.position, SparkBase.ControlType.kPosition);
    }

    SmartDashboard.putNumber("HoodTargetPosition", m_goal.position);
    SmartDashboard.putNumber("HoodCurrentPosition", m_setpoint.position);
    //SmartDashboard.putNumber("HoodCurrentPosition", m_jointMotor.getEncoder().getPosition()); //Alternative way to get position

    kP = SmartDashboard.getNumber("ShooterJointP", 0);
    kI = SmartDashboard.getNumber("ShooterJointI", 0);
    kD = SmartDashboard.getNumber("ShooterJointD", 0);
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
