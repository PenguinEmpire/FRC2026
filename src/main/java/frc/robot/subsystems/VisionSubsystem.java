// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.SpacialPosition;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionInterface;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  private int[] ValidIds = {5, 10, 26};
  private double currentID = -1;

  public double Fiducial3DOffsetX = 0;
  public double Fiducial3DOffsetY = 0;
  public double Fiducial3DOffsetZ = 0;

  private VisionInterface limelightVision;

  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem(VisionInterface limeInterface) {
    limelightVision = limeInterface;
    limelightVision.setFiducial3DOffset(0.0, 0.0, 0.0);
    limelightVision.SetFiducialIDFiltersOverride(ValidIds);
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
    return limelightVision.getTV();
  }

  public double TargetID() {
    return limelightVision.getFiducialID();
  }

  public double GetXAngle() {
    return limelightVision.getTX();
  }

  public double GetYAngle() {
    return limelightVision.getTY();
  }

  public void ChangeFiducial3DOffset(SpacialPosition offset) {
    limelightVision.setFiducial3DOffset(offset.getX(), offset.getY(), offset.getZ());
    Fiducial3DOffsetX = offset.getX();
    Fiducial3DOffsetY = offset.getY();
    Fiducial3DOffsetZ = offset.getZ();
  }

  public double getDistanceEstimate() {
    double LimelightMountAngleDegrees = Constants.VisionConstants.kLimelightMountAngleDegrees;
    double LimelightLensHeightInches = Constants.VisionConstants.kLimelightLensHeightInches;

    double targetHeightFromGroundInches = Constants.AprilTagConstants.kAprilTagHeights.get(currentID);

    double d = (targetHeightFromGroundInches-LimelightLensHeightInches) / Math.tan(LimelightMountAngleDegrees+GetYAngle());
    return d;
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

  public void updateOffsets() {
    if (TargetID()!=currentID) {
      if (HasTarget()) {
        currentID = Double.valueOf(TargetID());
      } else {
        currentID = -1;
      }
      ChangeFiducial3DOffset(Constants.AprilTagConstants.kAprilTagOffsets.get(currentID));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("VisionID", currentID);
    SmartDashboard.putNumber("tx", GetXAngle());
    SmartDashboard.putNumber("ty", GetYAngle());
    SmartDashboard.putBoolean("Within Shooter Range", GetXAngle()>=-10);

    updateOffsets();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

   public double getCurrentID() { //Only for testing, if not testing use targetID
    return currentID;
  }
}
