package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;

public class LimelightVision implements VisionInterface {
    String limelightName;

    public LimelightVision(String name) {
        limelightName = name;
    }

    public LimelightVision() {
        this("");
    }

    public void setFiducial3DOffset(double offsetX, double offsetY, double offsetZ) {
        LimelightHelpers.setFiducial3DOffset(limelightName, offsetX, offsetY, offsetZ);
   
    }
    public void SetFiducialIDFiltersOverride(int[] validIDs) {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIDs);
    }
    public double getFiducialID() {
        return LimelightHelpers.getFiducialID(limelightName);
    }
    public boolean getTV() {
        return LimelightHelpers.getTV(limelightName);
    }
    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }
    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }
}
