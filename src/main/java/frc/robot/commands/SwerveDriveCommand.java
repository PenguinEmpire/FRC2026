// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SwerveDriveCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final VisionSubsystem m_visionSubsystem;

  double rotationalSpeed;

  private PIDController rotationalPID;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveDriveCommand(VisionSubsystem visionSubsystem) {
    m_visionSubsystem = visionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationalPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_visionSubsystem.HasTarget()) {
      double tx = m_visionSubsystem.GetXAngle();
      //rotationalSpeed = rotationalPID.calculate(tx, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
