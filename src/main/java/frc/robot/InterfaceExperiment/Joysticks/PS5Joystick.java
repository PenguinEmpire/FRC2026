package frc.robot.InterfaceExperiment.Joysticks;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class PS5Joystick implements JoystickInterface{
    private double MaxSpeed = Constants.SwerveConstants.MaxSpeed;
    private final PS5Controller joystick;

    SlewRateLimiter limiterX = new SlewRateLimiter(SmartDashboard.getNumber("DriveSlewRateLimit", 3));
    SlewRateLimiter limiterY = new SlewRateLimiter(SmartDashboard.getNumber("DriveSlewRateLimit", 3));
    SlewRateLimiter limiterR = new SlewRateLimiter(SmartDashboard.getNumber("RotateSlewRateLimit", 3));

    public PS5Joystick(PS5Controller newJoystick) {
        SmartDashboard.putNumber("DriveSlewRateLimit", 3);
        SmartDashboard.putNumber("RotateSlewRateLimit", 2);
        this.joystick = newJoystick;
    }

    public double getVelocityX() {
        return limiterX.calculate(-joystick.getLeftY()) * MaxSpeed;
    }

    public double getVelocityY() {
        return limiterY.calculate(-joystick.getLeftX()) * MaxSpeed;
    }

    public double getRotationalRate() {
        return limiterR.calculate(-joystick.getRightX()) * Constants.SwerveConstants.kMaxAngularSpeed;
    }

    public boolean getR1() {
        return joystick.getR1Button();
    }

    public boolean getL2() {
        return joystick.getL2Button();
    }
}
