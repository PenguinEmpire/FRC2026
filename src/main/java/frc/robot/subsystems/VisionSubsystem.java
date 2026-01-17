// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  private int[] ValidIds = {1, 2, 3, 4};

  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem() {
    LimelightHelpers.setFiducial3DOffset("", 0.0, 0.0, 0.0);
    LimelightHelpers.SetFiducialIDFiltersOverride("", ValidIds);
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

  public boolean HasTarget() {
    return LimelightHelpers.getTV("");
  }

  public double TargetID() {
    return LimelightHelpers.getFiducialID("");
  }

  public double GetX() {
    return LimelightHelpers.getTX("");
  }

  public double GetY() {
    return LimelightHelpers.getTY("");
  }

  public void ChangeFiducial3DOffset(double x, double y, double z) {
    LimelightHelpers.setFiducial3DOffset("", x, y, z);
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
}
