// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignToAngle extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final SwerveSubsystem m_SwerveSubsystem;
  double currentAngle = 0;

  double targetAngle = 0;

  Pigeon2 pigion = new Pigeon2(0);

  PIDController pidRot = new PIDController(0.1, 0, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToAngle(SwerveSubsystem subsystem, double target) {
    m_SwerveSubsystem = subsystem;
    targetAngle = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAngle = pigion.getYaw().getValueAsDouble();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SwerveSubsystem.setRotationalRate(pidRot.calculate(currentAngle,targetAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentAngle-targetAngle<0.5) {
      return true;
    }
    return false;
  }
}
