// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PositionCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ExampleSubsystem m_subsystem;

  public enum Position {
        // Intake Positions
        Example1(0.0),  // Fold intake inside frame
        Example2(0.0); //figure this value out, originally 0.985
  
        
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



  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PositionCommand(ExampleSubsystem subsystem, Position pos) {
    m_subsystem = subsystem;
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
            case Example1:
                //exampleSubsystem.setPosition(Position.Example1.getEncoderPosition());
                break;
              default:
                break;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Return if the subsystems have reached target within tolerance (use a method inside the subsystem)
    return false;
  }
}
