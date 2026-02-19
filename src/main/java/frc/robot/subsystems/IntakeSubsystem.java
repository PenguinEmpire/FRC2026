// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
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
    
  private double targetPosition;

  double kP = SmartDashboard.getNumber("IntakeJointP", 0);
  double kI = SmartDashboard.getNumber("IntakeJointI", 0);
  double kD = SmartDashboard.getNumber("IntakeJointD", 0);

  //DigitalInput sensor = new DigitalInput(0);

  private final SparkMax m_RollerMotor = new SparkMax(Constants.IntakeConstants.kIntakeRollerMotorID, MotorType.kBrushless);

  PIDController manuPID  = new PIDController(0.02, kI, kD);

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    SmartDashboard.putNumber("IntakekDT", 0.2);
    SmartDashboard.putNumber("IntakeJointMaxVelocity", 20);
    SmartDashboard.putNumber("IntakeJointMaxAcceleration", 7.5);
    SmartDashboard.putNumber("IntakeJointP", 0);
    SmartDashboard.putNumber("IntakeJointI", 0);
    SmartDashboard.putNumber("IntakeJointD", 0);

    SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingMinInput(0)
                .positionWrappingMaxInput(2 * Math.PI)
                .outputRange(-0.01,0.01);

        motorConfig.closedLoop.feedForward
                  .kS(0)
                  .kV(0)
                  .kA(0)
                  .kG(0)
                  .kCos(0)
                  .kCosRatio(0);
        
        motorConfig.closedLoop.maxMotion
                  .cruiseVelocity(0)
                  .maxAcceleration(0)
                  .allowedProfileError(0);

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
  }

  public double getPosition() {
      RelativeEncoder relativeEncoder = m_jointMotor.getEncoder();

       
      return relativeEncoder.getPosition();
  }

  @Override
  public void periodic() {

    
    //pidController.setSetpoint(targetPosition, SparkBase.ControlType.kMAXMotionPositionControl);
    
    
    
    System.out.println(-manuPID.calculate(targetPosition, getPosition()));

    m_jointMotor.set(-manuPID.calculate(targetPosition, getPosition()));

    SmartDashboard.putNumber("IntakeTargetPosition", targetPosition);
    SmartDashboard.putNumber("IntakeCurrentPosition", getPosition());
    //SmartDashboard.putNumber("IntakeTargetPosition", getPosition());

  kP = SmartDashboard.getNumber("IntakeJointP", 0);
  kI = SmartDashboard.getNumber("IntakeJointI", 0);
  kD = SmartDashboard.getNumber("IntakeJointD", 0);

  /*if (sensor.get()) {
    spinRollers(false);
  }*/


  }

  public void ManualMove(double speed) {
      m_jointMotor.set(speed);
  }
  public void spinRollers(boolean active) {
    if (active) {
      m_RollerMotor.set(Constants.IntakeConstants.kRollerMotorPower);
    } else {
      m_RollerMotor.set(0);
    }
  }
}
