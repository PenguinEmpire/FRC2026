// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 
 */

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {
    public static final double kMaxAngularSpeed = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double AlignmentSpeedConstant = 0.5;

    public static final double rotationalkS = 0.47123; //0.47123
  }

  public static class ShooterConstants {
    public static final int kLeftShooterMotorID = 26; //Change with actual motor IDs and change name (add more once # of motors is known)
    public static final int kMiddleShooterMotorID = 27;
    public static final int kRightShooterMotorID = 28;

    public static final int kJointMotorID = 25;
    public static final double kShooterVelocity = -0.75; //Change to the output velocity of the shooter when known
    public static final double kShooterKickerVelocity = 0.3;
    public static final double kTargetShooterRPM = -4000;
  }

  public static class IntakeConstants {
    public static final int kIntakeRollerMotorID = 23; //Change with actual motor IDs and change name (add more once # of motors is known)
    public static final int kIntakeJointMotorID = 22;
    public static final int kConveyorMotorID = 24;
    
    public static final double kRollerMotorPower = -0.6;
    public static final double kConveyorMotorPower = 0.6;
  }

  public static class ClimbConstants {
    public static final int kClimbMotor1ID = 17;//Change with actual motor IDs and change name (add more once # of motors is known)
    public static final int kClimbMotor2ID = 0;
    public static final int kServoLeftID = 0;
    public static final int kServoRightID = 0;
    public static final int kServoMiddleID = 0;
    public static final int kServoGuideID = 0;

    public static final double kAlignmentTx = 0;
    public static final double kAlignmentTy = 0;

    public static final double kServoAngle = 0;

  }

  public static class VisionConstants {
    public static final double kLimelightMountAngleDegrees = 0;
    public static final double kLimelightLensHeightInches = 0;
  }

  public static class AprilTagConstants { 
    //OffsetConstants:
    public static final SpacialPosition kUndefinedOffset = new SpacialPosition(0, 0, 0);

    public static final SpacialPosition kAprilTagID1Offset = new SpacialPosition(0, 0, 0); //The position relative to april tag the vision will be targeting: {x, y, z}
    public static final SpacialPosition kAprilTagID2Offset = new SpacialPosition(0, 0, 0);
    public static final SpacialPosition kAprilTagID3Offset = new SpacialPosition(0, 0, 0);
    public static final SpacialPosition kAprilTagID10Offset = new SpacialPosition(-0.597, 0, 0); //-0.597
    public static final Map<Double, SpacialPosition> kAprilTagOffsets = Map.of(
            -1.0, kUndefinedOffset,
            1.0, kAprilTagID1Offset,
            2.0, kAprilTagID2Offset,
            3.0, kAprilTagID3Offset,
            10.0, kAprilTagID10Offset
        );

    public static final double kAprilTagID10Height = 0.0;
    public static final Map<Double, Double> kAprilTagHeights = Map.of(
      -1.0, 0.0,
      10.0, kAprilTagID10Height
    );
  }
}
