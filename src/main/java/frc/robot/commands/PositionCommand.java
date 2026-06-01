// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PositionCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final IntakeSubsystem m_intakeSubsystem;

  public enum Position {
        // Intake Positions
        IntakeUp(0.0),  // Fold intake inside frame
        IntakeDown(41.2), //figure this value out, originally 0.985
        IntakeBounce(21.991132736206055),

        ClimbUp(0.0),

        ClimbDown(0.0);
  
        
        private final double encoderPosition;

        Position(double encoderPosition) {
            this.encoderPosition = encoderPosition;
        }

        public double getEncoderPosition() {
            return encoderPosition;
        }
    }

    //Declare subsystems
    private final Position pos;

    double targetPosition;



  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PositionCommand(IntakeSubsystem subsystem, Position pos) {
    m_intakeSubsystem = subsystem;
    //m_climbSubsystem = climb;
    //Initialize the declared subsystems above
    // Use addRequirements() here to declare subsystem dependencies.
    this.pos = pos;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (pos) {
            case IntakeUp:
                //exampleSubsystem.setPosition(Position.Example1.getEncoderPosition());
                targetPosition = Position.IntakeUp.getEncoderPosition();
                m_intakeSubsystem.setPosition(Position.IntakeUp.getEncoderPosition());
                break;
            case IntakeDown:
                targetPosition = Position.IntakeDown.getEncoderPosition();
                m_intakeSubsystem.setPosition(Position.IntakeDown.getEncoderPosition());
                break;
            case IntakeBounce:
                targetPosition = Position.IntakeBounce.getEncoderPosition();
                m_intakeSubsystem.setPosition(Position.IntakeBounce.getEncoderPosition());
                break;
            /*case ClimbUp:
                m_climbSubsystem.setPosition(Position.ClimbUp.getEncoderPosition());
                break;
            case ClimbDown:
                m_climbSubsystem.setPosition(Position.ClimbDown.getEncoderPosition());
                break;*/
              default:
                break;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Intake reached position: " + targetPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Return if the subsystems have reached target within tolerance (use a method inside the subsystem)
    return m_intakeSubsystem.reachedPosition();
  }
}
