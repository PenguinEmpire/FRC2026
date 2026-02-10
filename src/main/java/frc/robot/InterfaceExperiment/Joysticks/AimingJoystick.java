package frc.robot.InterfaceExperiment.Joysticks;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;

public class AimingJoystick implements JoystickInterface{
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
        }else if (joystick.getL2()) { 
            return 0.25;
        } else {
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

    public boolean getL2() {
        return joystick.getL2();
    }
}
