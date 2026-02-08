package frc.robot.InterfaceExperiment.Joysticks;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.Constants;

public class PS5Joystick implements JoystickInterface{
    private double MaxSpeed = Constants.SwerveConstants.MaxSpeed;
    private final PS5Controller joystick;

    SlewRateLimiter limiterX = new SlewRateLimiter(3);
    SlewRateLimiter limiterY = new SlewRateLimiter(3);
    SlewRateLimiter limiterR = new SlewRateLimiter(2);

    public PS5Joystick(PS5Controller newJoystick) {
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
}
