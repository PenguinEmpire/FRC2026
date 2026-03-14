// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AlignToAngle;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.AlignToClimb;
import frc.robot.commands.EnableShooterCommand;
import frc.robot.commands.PositionCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionInterface;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.Pigeon2;

public class RobotContainer {
    //Smart dashboard configuration
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    //Initialize variables
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = Constants.SwerveConstants.kMaxAngularSpeed; // 3/4 of a rotation per second max angular velocity

    //Setting up bindings for necessary control of the swerve drive platform
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS5Controller controller = new CommandPS5Controller(0);

    private final PS5Controller joystick = new PS5Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    private ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

    private VisionSubsystem m_visionSubsystem = new VisionSubsystem(new LimelightVision());

    private SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

    private final EnableShooterCommand enableShooter = new EnableShooterCommand(m_shooterSubsystem);

    //private ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

    Pigeon2 pigeon = new Pigeon2(0);


    //public final AlignToAngle alignToAngle = new AlignToAngle(m_swerveSubsystem, -Math.PI/2.0);
    //public final AlignToClimb alignToClimb = new AlignToClimb(m_swerveSubsystem, 0, 0); pigeon.getYaw().getValueAsDouble()%360.0)+Math.PI/2

    public RobotContainer() {
        //Setup the namedCommands for auto

        NamedCommands.registerCommand("lower_intake", 
                new PositionCommand(m_intakeSubsystem, PositionCommand.Position.IntakeDown));
        
        NamedCommands.registerCommand("raise_intake",
                new PositionCommand(m_intakeSubsystem, PositionCommand.Position.IntakeUp));

         NamedCommands.registerCommand("enable_intake",
                new InstantCommand(()->m_intakeSubsystem.spinRollers(true), m_intakeSubsystem));

         NamedCommands.registerCommand("disable_intake",new InstantCommand(()->m_intakeSubsystem.spinRollers(false), m_intakeSubsystem));
        
        NamedCommands.registerCommand("align_to_apriltag", new AlignToAprilTag(m_swerveSubsystem, m_visionSubsystem, drivetrain));

        //NamedCommands.registerCommand("target_hood", ____ //I dont think we need this command since hood alignment is automatic

        NamedCommands.registerCommand("enable_flywheel", new SequentialCommandGroup(
                    new EnableShooterCommand(m_shooterSubsystem),
                    new WaitCommand(1.5),
                    new InstantCommand(()->m_intakeSubsystem.spinConveyor(Constants.IntakeConstants.kConveyorMotorPower), m_intakeSubsystem),
                    new InstantCommand(()->m_shooterSubsystem.enableKicker(Constants.ShooterConstants.kShooterKickerVelocity), m_intakeSubsystem)));

         NamedCommands.registerCommand("enable_intake_bounce",
            new SequentialCommandGroup(
                        new RunCommand(()->m_intakeSubsystem.setPosition(PositionCommand.Position.IntakeBounce.getEncoderPosition())).until(()->m_intakeSubsystem.getPosition()<30),
                        new PositionCommand(m_intakeSubsystem, PositionCommand.Position.IntakeDown)
                        )
                    .repeatedly());

        NamedCommands.registerCommand("disable_intake_bounce",
                        new PositionCommand(m_intakeSubsystem, PositionCommand.Position.IntakeDown));

        NamedCommands.registerCommand("disable_flywheel", new SequentialCommandGroup(
                    new InstantCommand(()->m_shooterSubsystem.enableShooter(false), m_shooterSubsystem),
                    new InstantCommand(()->m_shooterSubsystem.enableKicker(0), m_shooterSubsystem),
                    new InstantCommand(()->m_intakeSubsystem.spinConveyor(0), m_intakeSubsystem)));

        NamedCommands.registerCommand("mount_climb", new SequentialCommandGroup(
                            new AlignToAngle(m_swerveSubsystem, Math.PI),
                            new AlignToClimb(m_swerveSubsystem, Constants.ClimbConstants.kAlignmentTx, Constants.ClimbConstants.kAlignmentTy),
                            new AlignToAngle(m_swerveSubsystem, 0)
                            /*new InstantCommand(()->m_climbSubsystem.setLeftServo(Constants.ClimbConstants.kServoAngle), m_climbSubsystem),
                            new InstantCommand(()->m_climbSubsystem.setRightServo(Constants.ClimbConstants.kServoAngle), m_climbSubsystem),
                            new WaitCommand(1),
                            new PositionCommand(m_intakeSubsystem, m_climbSubsystem, PositionCommand.Position.ClimbUp)*/
                        )
                    );

        NamedCommands.registerCommand("unmount_climb", new SequentialCommandGroup(
                            new AlignToAngle(m_swerveSubsystem, Math.PI),
                            new AlignToClimb(m_swerveSubsystem, Constants.ClimbConstants.kAlignmentTx, Constants.ClimbConstants.kAlignmentTy),
                            new AlignToAngle(m_swerveSubsystem, 0)
                            /*new InstantCommand(()->m_climbSubsystem.setLeftServo(Constants.ClimbConstants.kServoAngle), m_climbSubsystem),
                            new InstantCommand(()->m_climbSubsystem.setRightServo(Constants.ClimbConstants.kServoAngle), m_climbSubsystem),
                            new WaitCommand(1),
                            new PositionCommand(m_intakeSubsystem, m_climbSubsystem, PositionCommand.Position.ClimbUp)*/
                        )
                    );

        
        //Setup smartdashboard
        autoChooser.addOption("CircleAuto", "CircleAuto"); //Name is the name visible on S.D. object is what is returned by getSelected()
        autoChooser.addOption("LineAuto", "LineAuto"); 
        autoChooser.addOption("RotationAuto", "RotationAuto"); 
        autoChooser.addOption("IntakeAuto", "IntakeAuto"); 
        autoChooser.addOption("ShooterAuto", "ShooterAuto"); 
        autoChooser.addOption("AprilTagAuto", "AprilTagAuto");
        autoChooser.addOption("PreloadMidAuto", "PreloadMidAuto");
        autoChooser.addOption("LeftNeutralAuto", "LeftNeutralAuto");

        SmartDashboard.putBoolean("Within Shooter Range", false);

        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(m_swerveSubsystem.getVelocityX()) // Drive forward with negative Y (forward)
                    .withVelocityY(m_swerveSubsystem.getVelocityY()) // Drive left with negative X (left) //m_swerveSubsystem.getVelocityY()
                    .withRotationalRate(m_swerveSubsystem.getRotationalRate()) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        controller.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.circle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.create().and(controller.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.create().and(controller.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.options().and(controller.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.options().and(controller.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        controller.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        //Intake
        controller.povUp().onTrue( //raise intake
            new SequentialCommandGroup(
                new InstantCommand(()->m_intakeSubsystem.spinRollers(false), m_intakeSubsystem),
                new PositionCommand(m_intakeSubsystem, PositionCommand.Position.IntakeUp)));

        controller.povDown().onTrue( //lower intake
            new SequentialCommandGroup(
                new PositionCommand(m_intakeSubsystem, PositionCommand.Position.IntakeDown)));

        controller.L2() //Enable intake
            .whileTrue(new InstantCommand(()->m_intakeSubsystem.spinRollers(true), m_intakeSubsystem))
            .onFalse(new InstantCommand(()->m_intakeSubsystem.spinRollers(false), m_intakeSubsystem));

        /*controller.L2().whileTrue(new InstantCommand(()->m_climbSubsystem.manualMove(0.1)))
                        .onFalse(new InstantCommand(()->m_climbSubsystem.manualMove(0)));*/

        /*controller.povLeft() //manual climb down
            .whileTrue(new RunCommand(()->m_climbSubsystem.manualMove(0.1), m_climbSubsystem)) //raise
            .onFalse(new RunCommand(()->m_climbSubsystem.manualMove(0), m_climbSubsystem));

        controller.povRight() //manual climb up
            .whileTrue(new RunCommand(()->m_climbSubsystem.manualMove(-0.1), m_climbSubsystem)) //lower
            .onFalse(new RunCommand(()->m_climbSubsystem.manualMove(0), m_climbSubsystem));*/

        controller.R2() //activate shooter/indexer/conveyor
            .onTrue(
                new SequentialCommandGroup(
                    enableShooter,
                    new WaitCommand(1.5),
                    new InstantCommand(()->m_intakeSubsystem.spinConveyor(Constants.IntakeConstants.kConveyorMotorPower), m_intakeSubsystem),
                    new InstantCommand(()->m_shooterSubsystem.enableKicker(Constants.ShooterConstants.kShooterKickerVelocity), m_intakeSubsystem),

                    new SequentialCommandGroup(
                        new RunCommand(()->m_intakeSubsystem.setPosition(PositionCommand.Position.IntakeBounce.getEncoderPosition())).until(()->m_intakeSubsystem.getPosition()<30),
                        new PositionCommand(m_intakeSubsystem, PositionCommand.Position.IntakeDown)
                        )
                    .repeatedly()))

            .onFalse(
                new SequentialCommandGroup(
                    new InstantCommand(()->m_shooterSubsystem.enableShooter(false), m_shooterSubsystem),
                    new InstantCommand(()->m_shooterSubsystem.enableKicker(0), m_shooterSubsystem),
                    new InstantCommand(()->m_intakeSubsystem.spinConveyor(0), m_intakeSubsystem)));

        controller.square() //reverse indexer/conveyor
                    .onTrue(
                        new SequentialCommandGroup(
                            new InstantCommand(()->m_shooterSubsystem.enableKicker(-Constants.ShooterConstants.kShooterKickerVelocity), m_intakeSubsystem),
                            new InstantCommand(()->m_intakeSubsystem.spinConveyor(-Constants.IntakeConstants.kConveyorMotorPower), m_intakeSubsystem)))
        
                    .onFalse(
                        new SequentialCommandGroup(
                            new InstantCommand(()->m_shooterSubsystem.enableKicker(0), m_shooterSubsystem),
                            new InstantCommand(()->m_intakeSubsystem.spinConveyor(0), m_intakeSubsystem)));

       
        /*controller.options() //Activate Climb Sequence
                    .onTrue(
                        new SequentialCommandGroup(
                            new AlignToAngle(m_swerveSubsystem, Math.PI),
                            new AlignToClimb(m_swerveSubsystem, Constants.ClimbConstants.kAlignmentTx, Constants.ClimbConstants.kAlignmentTy),
                            new AlignToAngle(m_swerveSubsystem, 0),
                            new InstantCommand(()->m_climbSubsystem.setLeftServo(Constants.ClimbConstants.kServoAngle), m_climbSubsystem),
                            new InstantCommand(()->m_climbSubsystem.setRightServo(Constants.ClimbConstants.kServoAngle), m_climbSubsystem),
                            new WaitCommand(1),
                            new PositionCommand(m_intakeSubsystem, m_climbSubsystem, PositionCommand.Position.ClimbUp)
                        )
                    );*/

        //Start to unclimb (not implimented)


        //controller.touchpad() for a potential reset button
    }   

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();

        return new PathPlannerAuto(autoChooser.getSelected());
        //return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            /*drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
            */
        //);
    }
}
