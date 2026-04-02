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

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.vision.LimelightVision;  

public class ShooterSubsystem extends SubsystemBase {
  private SparkMax m_leftShooterMotor = new SparkMax(Constants.ShooterConstants.kLeftShooterMotorID, MotorType.kBrushless);
  private SparkMax m_middleShooterMotor = new SparkMax(Constants.ShooterConstants.kMiddleShooterMotorID, MotorType.kBrushless);
  private SparkMax m_rightShooterMotor = new SparkMax(Constants.ShooterConstants.kRightShooterMotorID, MotorType.kBrushless);

  private SparkMax m_jointMotor = new SparkMax(Constants.ShooterConstants.kJointMotorID, MotorType.kBrushless);
  private PS5Controller joystick = new PS5Controller(0);

  private SimpleMotorFeedforward flywheelFeedForward = new SimpleMotorFeedforward(0.0147, 0.000182507);

  private final SparkClosedLoopController pidController;

  private PIDController manuPID = new PIDController(0.007, 0, 0);

  private PIDController flywheelPID = new PIDController(0.00055, 0, 0.0001); //0.00035 needs to be tuned

  private VisionSubsystem m_VisionSubsystem = new VisionSubsystem(new LimelightVision());

  private boolean limiting = true;

    double kP = SmartDashboard.getNumber("ShooterJointP", 0);
    double kI = SmartDashboard.getNumber("ShooterJointI", 0);
    double kD = SmartDashboard.getNumber("ShooterJointD", 0);

    SlewRateLimiter shoooterLimiter = new SlewRateLimiter(0.1);

    double targetShooterVelocity = 0;

    double currentHoodVoltage = 0;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    SmartDashboard.putNumber("ShooterkDT", 0.2);
    SmartDashboard.putNumber("ShooterJointMaxVelocity", 20);
    SmartDashboard.putNumber("ShooterJointMaxAcceleration", 7.5);
    SmartDashboard.putNumber("ShooterJointP", 0);
    SmartDashboard.putNumber("ShooterJointI", 0);
    SmartDashboard.putNumber("ShooterJointD", 0);
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    SparkMaxConfig flywheelConfig = new SparkMaxConfig();
    flywheelConfig.smartCurrentLimit(30);

    m_leftShooterMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_middleShooterMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightShooterMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



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

        /*motorConfig.closedLoop.feedForward
                  .kS(0)
                  .kV(0)
                  .kA(0)
                  .kG(0)
                  .kCos(0)
                  .kCosRatio(0);*/
        
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

  private double getTargetAngle() {
      double ty = m_VisionSubsystem.GetYAngle();
      double target = 0.0254911*ty*ty+2.90326*ty+19.15338;
      if (target > 0) {
        return 0; //Whatever the line of best fit is
      }
      if (target < -270) {
        return -270;
      }

      return target;
  }

  public double getShooterSpeed() {
    return m_leftShooterMotor.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*if (m_VisionSubsystem.HasTarget()) {
      pidController.setSetpoint(getTargetAngle(), SparkBase.ControlType.kPosition);
    }*/

    //System.out.println(getShooterSpeed());

    double error = getShooterSpeed() - targetShooterVelocity;

    double pidSpeed = flywheelFeedForward.calculate(targetShooterVelocity) + flywheelPID.calculate(getShooterSpeed(), targetShooterVelocity); //double pidSpeed = flywheelPID.calculate(getShooterSpeed(), targetShooterVelocity);
    double output;
    if (limiting) {
      output = shoooterLimiter.calculate(pidSpeed); //With limiter
    } else {
      output = pidSpeed;
    }

    m_leftShooterMotor.set(output);
    m_rightShooterMotor.set(output);

    SmartDashboard.putNumber("FlywheelCurrent", m_leftShooterMotor.getOutputCurrent());

    SmartDashboard.putNumber("KickerCurrent", m_middleShooterMotor.getOutputCurrent());

    SmartDashboard.putNumber("FlywheelVelocity", m_leftShooterMotor.getEncoder().getVelocity());

    SmartDashboard.putNumber("KickerVelocity", m_middleShooterMotor.getEncoder().getVelocity());

    //System.out.println(getShooterSpeed());
    if (m_VisionSubsystem.HasTarget() && (m_VisionSubsystem.getCurrentID() == 10 || m_VisionSubsystem.getCurrentID() == 26 || m_VisionSubsystem.getCurrentID() == 5 || m_VisionSubsystem.getCurrentID() == 2) && currentHoodVoltage ==0) {
      m_jointMotor.set(manuPID.calculate(getPosition(), getTargetAngle()));
    } else {
      m_jointMotor.set(currentHoodVoltage);
    }

    //SmartDashboard.putNumber("HoodTargetPosition", getTargetAngle());
    SmartDashboard.putNumber("HoodCurrentPosition", m_jointMotor.getEncoder().getPosition());
    //SmartDashboard.putNumber("HoodCurrentPosition", m_jointMotor.getEncoder().getPosition()); //Alternative way to get position

    kP = SmartDashboard.getNumber("ShooterJointP", 0);
    kI = SmartDashboard.getNumber("ShooterJointI", 0);
    kD = SmartDashboard.getNumber("ShooterJointD", 0);
  }

  public void moveHood(double speed) {
    //m_jointMotor.set(speed);

    currentHoodVoltage = speed;
  }

  public double getPosition() {
    return m_jointMotor.getEncoder().getPosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void enableShooter(boolean active) {
    double calc;
    if (active) {
      calc = shoooterLimiter.calculate(Constants.ShooterConstants.kShooterVelocity);
      targetShooterVelocity = Constants.ShooterConstants.kTargetShooterRPM;
      //limiting = false;
    } else {
     calc = shoooterLimiter.calculate(0);
    //limiting = true;
     targetShooterVelocity = 0;
    }
    //m_leftShooterMotor.set(calc);
    //m_rightShooterMotor.set(calc);
  }

  public void enableKicker(double speed) {
    m_middleShooterMotor.set(speed);
    System.out.println("Kicker set to speed " + speed);
  }

  public void runFeedForwardTest() {
    double v = 0;
    while (v<=0.25) {
      m_leftShooterMotor.set(v);
      m_rightShooterMotor.set(v);
      try {Thread.sleep(5000);
      } catch (InterruptedException e) {
        System.out.println(e);
      }
      System.out.println("("+getShooterSpeed()+", " + v);
      v+=0.0001;
    }
  }
}