package frc.robot.InterfaceExperiment.Joysticks;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.VisionSubsystem;

public class AimingJoystick implements JoystickInterface{
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final JoystickInterface joystick;
    private final VisionSubsystem m_VisionSubsystem;

    private PIDController rotPID = new PIDController(0.026,0,0);
        
    private double kS = Constants.SwerveConstants.rotationalkS;
    

    public AimingJoystick(JoystickInterface newJoystick, VisionSubsystem vision) {
        this.joystick = newJoystick;
        this.m_VisionSubsystem = vision;
    }

    private double getDampeningPercent() {
        if (m_VisionSubsystem.HasTarget() && joystick.getR1()) {
            return Constants.SwerveConstants.AlignmentSpeedConstant; //Put this in cons
        }else { 
            return 1.0;
        }
    }

    public double getVelocityX() {
        double velocityX = joystick.getVelocityX();
        return velocityX*getDampeningPercent();
    }

    public double getVelocityY() {
        double velocityY = joystick.getVelocityY();
        return velocityY*getDampeningPercent();
    }

    public double getRotationalRate() {
        double rotationalRate = joystick.getRotationalRate();
        if (m_VisionSubsystem.HasTarget() && joystick.getR1()) {
            double RadTx = m_VisionSubsystem.GetXAngle()*Math.PI/180.0;
            double pidSpeed = rotPID.calculate(RadTx, 0);

            pidSpeed+=Math.copySign(kS, pidSpeed);

            return pidSpeed;
        }else { 
            return rotationalRate;
        }
    }

    public boolean getR1() {
        return joystick.getR1();
    }
}
