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
public class AlignToAprilTag extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final SwerveSubsystem m_subsystem;

  private CommandSwerveDrivetrain drivetrain;

  private final VisionSubsystem m_VisionSubsystem;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

  private PIDController rotPID = new PIDController(0.026,0.001,0.0001);

  private double kS = Constants.SwerveConstants.rotationalkS;
  boolean runCommand = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToAprilTag(SwerveSubsystem subsystem, VisionSubsystem vision, CommandSwerveDrivetrain drive) {
    this.m_subsystem = subsystem;
    this.m_VisionSubsystem = vision;
    this.drivetrain=drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, vision, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runCommand = m_VisionSubsystem.HasTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (runCommand) {
      double RadTx = m_VisionSubsystem.GetXAngle()*(Math.PI/180.0);
      double pidSpeed = rotPID.calculate(RadTx, 5*(Math.PI/180.0));

      pidSpeed+=Math.copySign(kS, pidSpeed); //replace kS with angularVelocity if wanting to take physics approach
      final double output = pidSpeed;
      System.out.println(output);
      drivetrain.setControl(
                drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(output)
        );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
                drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
        );
    runCommand = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double RadTx = m_VisionSubsystem.GetXAngle()*Math.PI/180.0;
    return Math.abs((5*(Math.PI/180))-RadTx)<0.026;
  }
}
