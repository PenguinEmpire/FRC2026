// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  private int[] ValidIds = {1, 2, 3, 4};
  private double currentID = -1;

  public double Fiducial3DOffsetX = 0;
  public double Fiducial3DOffsetY = 0;
  public double Fiducial3DOffsetZ = 0;

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

  public double GetXAngle() {
    return LimelightHelpers.getTX("");
  }

  public double GetYAngle() {
    return LimelightHelpers.getTY("");
  }

  public void ChangeFiducial3DOffset(double[] offset) {
    LimelightHelpers.setFiducial3DOffset("", offset[0], offset[1], offset[2]);
    Fiducial3DOffsetX = offset[0];
    Fiducial3DOffsetY = offset[1];
    Fiducial3DOffsetZ = offset[2];
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
    if (TargetID()!=currentID) {
      if (HasTarget()) {
        currentID = TargetID();
        int Id = (int) Math.round(currentID);
        ChangeFiducial3DOffset(Constants.AprilTagConstants.kAprilTagOffsets[Id]);
      } else {
        currentID = -1;
        ChangeFiducial3DOffset(Constants.AprilTagConstants.kUndefinedOffset);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
