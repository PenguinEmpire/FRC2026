package frc.robot.subsystems.vision;

public interface VisionInterface {
    public void setFiducial3DOffset(double offsetX, double offsetY, double offsetZ);
    public void SetFiducialIDFiltersOverride(int[] validIDs);
    public double getFiducialID();
    public boolean getTV();
    public double getTX();
    public double getTY();
}