// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.SpacialPosition;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.vision.LimelightVision;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignToPosition extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final SwerveSubsystem m_SwerveSubsystem;

  double targetAngleX = -Math.PI/2.0;
  double targetAngleY = -Math.PI/2.0;

  VisionSubsystem m_VisionSubsystem = new VisionSubsystem(new LimelightVision());

  PIDController pidx = new PIDController(0.002, 0, 0); //0.0002725
  PIDController pidy = new PIDController(0.002, 0, 0); //0.0002725

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToPosition(SwerveSubsystem subsystem, double targetTx, double targetTy) {
    m_SwerveSubsystem = subsystem;
    targetAngleX = targetTx;
    targetAngleY = targetTy;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SpacialPosition offset = new SpacialPosition(0,0,0);
    m_VisionSubsystem.ChangeFiducial3DOffset(offset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedx = pidx.calculate(Math.toRadians(m_VisionSubsystem.GetXAngle()) ,Math.toRadians(targetAngleX));
    double speedy = pidy.calculate(Math.toRadians(m_VisionSubsystem.GetYAngle()) ,Math.toRadians(targetAngleY));
    
    m_SwerveSubsystem.setVelocityX(speedx);
    m_SwerveSubsystem.setVelocityY(speedy);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.resetRotationalRate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
