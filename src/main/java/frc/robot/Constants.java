// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import frc.robot.Constants;

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
  }

  public static class ShooterConstants {
    public static final int kLeftShooterMotorID = 0; //Change with actual motor IDs and change name (add more once # of motors is known)
    public static final int kMiddleShooterMotorID = 0;
    public static final int kRightShooterMotorID = 0;

    public static final int kJointMotorID = 0;
    public static final int kShooterVelocity = 0; //Change to the output velocity of the shooter when known
  }

  public static class IntakeConstants {
    public static final int kIntakeRollerMotorID = 0; //Change with actual motor IDs and change name (add more once # of motors is known)
    public static final int kIntakeJointMotorID = 0;
    
    public static final int kRollerMotorPower = 0;
  }

  public static class ClimbConstants {
    public static final int kClimbMotorID = 0;//Change with actual motor IDs and change name (add more once # of motors is known)
  }

  public static class VisionConstants {
    public static final double kLimelightMountAngleDegrees = 0;
    public static final double kLimelightLensHeightInches = 0;
  }

  public static class AprilTagConstants { 
    //OffsetConstants:
    public static final double[] kUndefinedOffset = {0,0,0};

    public static final double[] kAprilTagID1Offset = {-1, 0.0, -1}; //The position relative to april tag the vision will be targeting: {x, y, z}
    public static final double[] kAprilTagID2Offset = {-0.5, 1, 0.0};
    public static final double[] kAprilTagID3Offset = {0.0, 0.0, 0.0};
    public static final double[] kAprilTagID10Offset = {-0.597, 0.0, 0.0}; //-0.597
    public static final Map<Integer, double[]> kAprilTagOffsets = Map.of(
            -1, kUndefinedOffset,
            1, kAprilTagID1Offset,
            2, kAprilTagID2Offset,
            3, kAprilTagID3Offset,
            10, kAprilTagID10Offset
        );
  }
}
