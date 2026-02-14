package frc.robot;

public class SpacialPosition {
    double xPos;
    double yPos;
    double zPos;
    public SpacialPosition(double x, double y, double z) {
        xPos = x;
        yPos = y;
        zPos = z;
    }

    public double getX() {
        return xPos;
    }

    public double getY() {
        return yPos;
    }

    public double getZ() {
        return zPos;
    }
}
