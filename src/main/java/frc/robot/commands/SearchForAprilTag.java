// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

/** An example command that uses an example subsystem. */
public class SearchForAprilTag extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final SwerveSubsystem m_subsystem;

  private CommandSwerveDrivetrain drivetrain;

  private final VisionSubsystem m_VisionSubsystem;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

  double turningSpeed;

  private PIDController rotPID = new PIDController(0.026,0.001,0.0001);

  private double kS = Constants.SwerveConstants.rotationalkS;
  boolean runCommand = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SearchForAprilTag(SwerveSubsystem subsystem, VisionSubsystem vision, CommandSwerveDrivetrain drive, double speed) {
    this.m_subsystem = subsystem;
    this.m_VisionSubsystem = vision;
    this.drivetrain=drive;
    this.turningSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, vision, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      drivetrain.setControl(
                drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(turningSpeed)
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
                drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
        );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_VisionSubsystem.HasTarget();
  }
}
