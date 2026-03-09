// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignToAngle extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final SwerveSubsystem m_SwerveSubsystem;

  double targetAngle = -Math.PI/2.0;

  Pigeon2 pigeon = new Pigeon2(0);

  PIDController pidRot = new PIDController(0.00001, 0, 0); //0.0002725

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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidRot.calculate(Math.toRadians(pigeon.getYaw().getValueAsDouble()%360.0) ,targetAngle);
    double x= 0;
    x+=Math.copySign(0.4712389, speed);
    
    m_SwerveSubsystem.setRotationalRate(x);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.resetRotationalRate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs((pigeon.getYaw().getValueAsDouble()%360.0)-targetAngle)<0.1) {
      return true;
    }
    return false;
  }
}
