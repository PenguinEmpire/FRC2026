package frc.robot.InterfaceExperiment.Joysticks;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;

public class AimingJoystick implements JoystickInterface{
    private final JoystickInterface joystick;
    private final VisionSubsystem m_VisionSubsystem;

    private PIDController rotPID = new PIDController(0.026,0,0.001);
        
    private double kS = Constants.SwerveConstants.rotationalkS;

    private double targetAngle = 5*(Math.PI)/180;
    

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
        double RadTx = m_VisionSubsystem.GetXAngle()*Math.PI/180.0;
        if (m_VisionSubsystem.HasTarget() && joystick.getR1()&& Math.abs(targetAngle) - Math.abs(RadTx)<0.1) {
            double velocityY = getVelocityY();
            double radius = m_VisionSubsystem.getDistanceEstimate();

            double angularVelocity = velocityY/radius;
            double pidSpeed = rotPID.calculate(RadTx, targetAngle);

            pidSpeed+=Math.copySign(kS, pidSpeed); //replace kS with angularVelocity if wanting to take physics approach

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
