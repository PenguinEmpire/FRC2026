package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.vision.VisionInterface;

public class MyVisionTester implements VisionInterface{
    double offsetX;
    double offsetY;
    double offsetZ;

    int[] validIDs;

    double FiducialID;
    boolean hasID;
    double tx;
    double ty;

    public MyVisionTester() {}

    public void setFiducial3DOffset(double offsetX, double offsetY, double offsetZ) {
        this.offsetX = offsetX;
        this.offsetY = offsetY;
        this.offsetZ = offsetZ;
    }
    public void SetFiducialIDFiltersOverride(int[] validIDs) {
        this.validIDs = validIDs;
    }
    public double getFiducialID() {
        return FiducialID;
    }
    public boolean getTV() {
        return hasID;
    }
    public double getTX() {
        return tx;
    }
    public double getTY() {
        return ty;
    }
}
