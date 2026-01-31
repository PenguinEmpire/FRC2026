// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.LogManager;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Modules.JointModule;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
  private final JointModule m_JointRotation = new JointModule("IntakeJoint", Constants.IntakeConstants.kIntakeJointMotorID);

  private final SparkMax m_RollerMotor = new SparkMax(Constants.IntakeConstants.kIntakeRollerMotorID, MotorType.kBrushless);
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {}

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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setRotationPosition(double position) {
        m_JointRotation.setPosition(position); //  Move intake to a target position
    }



  public void spinRollers(boolean active) {
    if (active) {
      m_RollerMotor.set(Constants.IntakeConstants.kRollerMotorPower);
    }
  }

  public void manualMove(double speed) {
    m_JointRotation.ManualMove(speed);
  }
}
