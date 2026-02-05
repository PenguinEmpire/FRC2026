package frc.robot.InterfaceExperiment.Joysticks;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class PS5Joystick implements JoystickInterface{
    private double MaxSpeed = Constants.SwerveConstants.MaxSpeed;
    private final PS5Controller joystick;

    public PS5Joystick(PS5Controller newJoystick) {
        this.joystick = newJoystick;
    }

    public double getVelocityX() {
        return -joystick.getLeftY() * MaxSpeed;
    }

    public double getVelocityY() {
        return -joystick.getLeftX() * MaxSpeed;
    }

    public double getRotationalRate() {
        return -joystick.getRightX() * Constants.SwerveConstants.kMaxAngularSpeed;
    }

    public boolean getR1() {
        return joystick.getR1Button();
    }
}
