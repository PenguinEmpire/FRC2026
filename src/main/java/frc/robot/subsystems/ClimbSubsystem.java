// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax m_ClimbMotor1 = new SparkMax(Constants.ClimbConstants.kClimbMotor1ID, MotorType.kBrushless);
  //private Servo m_ServoLeftMotor = new Servo(Constants.ClimbConstants.kServoLeftID);
  //private Servo m_ServoRightMotor = new Servo(Constants.ClimbConstants.kServoRightID);
  
  
  //private Servo m_ServoMiddleMotor = new Servo(Constants.ClimbConstants.kServoMiddleID);
  //private Servo m_ServoGuideMotor = new Servo(Constants.ClimbConstants.kServoGuideID);

  private PS5Controller joystick = new PS5Controller(0);

  private final SparkClosedLoopController pidController;

    private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(SmartDashboard.getNumber("ShooterJointMaxVelocity", 20), SmartDashboard.getNumber("ShooterJointMaxAcceleration", 7.5)));
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private double kDt = SmartDashboard.getNumber("ShooterkDT", 0.2);

  public ClimbSubsystem() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
            .inverted(false)
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
        m_ClimbMotor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.follow(Constants.ClimbConstants.kClimbMotor1ID, true);

        m_ClimbMotor2.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);*/

        pidController = m_ClimbMotor1.getClosedLoopController();
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
  public void periodic() {
    // This method will be called once per scheduler run
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
    if (!joystick.getTriangleButton() && !joystick.getSquareButton()) {
       pidController.setSetpoint(m_setpoint.position, SparkBase.ControlType.kPosition);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  /*public void setServos(double newTargetPosition) {
      m_ServoLeftMotor.set(newTargetPosition);
      m_ServoRightMotor.set(newTargetPosition);
  }*/

  /*public void setRightServo(double newTargetPosition) {
      m_ServoMiddleMotor.set(newTargetPosition);
  }*/

  /*public void setGuideServoPosition(double newTargetPosition) {
      m_ServoGuideMotor.set(newTargetPosition);
  }*/

  
  public void setPosition(double newTargetPosition) {
      m_goal = new TrapezoidProfile.State(newTargetPosition, 0);
  }

  public double getPosition() {
      RelativeEncoder relativeEncoder = m_ClimbMotor1.getEncoder();

       
      return relativeEncoder.getPosition();
  }

  public void manualMove(double speed) {
      m_ClimbMotor1.set(speed);
  }
}
