// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.InterfaceExperiment.Joysticks.AimingJoystick;
import frc.robot.InterfaceExperiment.Joysticks.JoystickInterface;
import frc.robot.InterfaceExperiment.Joysticks.PS5Joystick;
import com.ctre.phoenix6.hardware.Pigeon2;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final JoystickInterface joystick = new AimingJoystick(new PS5Joystick(new PS5Controller(0)), new VisionSubsystem());
  private double velocityX = 0;
  private double velocityY = 0;
  private double rotationalRate = 0;
  private boolean overrideX = false;
  private boolean overrideY = false;
  private boolean overrideR = false;

  Pigeon2 pigeon = new Pigeon2(0);

  public SwerveSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!overrideX) {
      velocityX = joystick.getVelocityX();
    }
    if (!overrideY) {
      velocityY = joystick.getVelocityY();
    }
    if (!overrideR) {
      rotationalRate = joystick.getRotationalRate();
    }

    if (joystick.getVelocityX()>1) {
      resetVelocityX();
    }

    if (joystick.getVelocityY()>1) {
      resetVelocityY();
    }

    if (joystick.getRotationalRate()>1) {
      resetRotationalRate();
    }
  } 

  public double getVelocityX() {
    return velocityX;
  }

  public double getVelocityY() {
    return velocityY;
  }

  public double getRotationalRate() {
    return rotationalRate;
  }


  public void setVelocityX(double speed) {
    overrideX = true;
    velocityX = speed;
  }

  public void setVelocityY(double speed) {
    overrideY = true;
    velocityY = speed;
  }

  public void setRotationalRate(double speed) {
    overrideR = true;
    rotationalRate = speed;
  }

  public void resetVelocityX() {
    overrideX = false;
  }

  public void resetVelocityY() {
    overrideY = false;
  }

  public void resetRotationalRate() {
    overrideR = false;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
