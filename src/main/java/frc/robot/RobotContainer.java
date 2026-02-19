// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.InterfaceExperiment.Joysticks.AimingJoystick;
import frc.robot.InterfaceExperiment.Joysticks.JoystickInterface;
import frc.robot.InterfaceExperiment.Joysticks.PS5Joystick;
import frc.robot.commands.AlignToAngle;
import frc.robot.commands.AlignToClimb;
import frc.robot.commands.PositionCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.vision.LimelightVision;

public class RobotContainer {
    //Smart dashboard configuration
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    //Initialize variables
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = Constants.SwerveConstants.kMaxAngularSpeed; // 3/4 of a rotation per second max angular velocity

    //Setting up bindings for necessary control of the swerve drive platform
    /*private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);*/

    private final CommandPS5Controller controller = new CommandPS5Controller(0);

    //private final JoystickInterface joystick = new AimingJoystick(new PS5Joystick(new PS5Controller(0)), new VisionSubsystem(new LimelightVision()));

    private final PS5Controller joystick = new PS5Controller(0);

    //public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    //public final AlignToAngle alignToAngle = new AlignToAngle(m_swerveSubsystem, -Math.PI/2.0);
    //public final AlignToClimb alignToClimb = new AlignToClimb(m_swerveSubsystem, 0, 0);

    public RobotContainer() {
        configureBindings();

        //Setup smartdashboard
        autoChooser.addOption("FRC Auto 1", "FRCAuto"); //Name is the name visible on S.D. object is what is returned by getSelected()
    }

    //private SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        /*drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(m_swerveSubsystem.getVelocityX()) // Drive forward with negative Y (forward)
                    .withVelocityY(m_swerveSubsystem.getVelocityY()) // Drive left with negative X (left)
                    .withRotationalRate(m_swerveSubsystem.getRotationalRate()) // Drive counterclockwise with negative X (left)
            )
        );*/

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        /*final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );*/

        /*joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.circle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.create().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.create().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/

        // Reset the field-centric heading on left bumper press.
        //controller.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        //controller.R2().whileTrue(alignToAngle);

        //drivetrain.registerTelemetry(logger::telemeterize);

        /*if (joystick.getCircleButton()) {
            m_intakeSubsystem.ManualMove(0.1);   
        } else if (joystick.getCrossButton()) {
            m_intakeSubsystem.ManualMove(-0.1);
        }*/

        /*if (joystick.getTriangleButton()) {
            m_intakeSubsystem.spinRollers(true);   
        } else {
            m_intakeSubsystem.spinRollers(false);
        }*/

        controller.triangle()
            .whileTrue(new RunCommand(()->m_intakeSubsystem.spinRollers(true), m_intakeSubsystem))
            .onFalse(new RunCommand(()->m_intakeSubsystem.spinRollers(false), m_intakeSubsystem));

        /*controller.L1()
            .whileTrue(new RunCommand(()->m_intakeSubsystem.ManualMove(-0.1), m_intakeSubsystem)) //raise
            .onFalse(new RunCommand(()->m_intakeSubsystem.ManualMove(0), m_intakeSubsystem));

        controller.L2()
            .whileTrue(new RunCommand(()->m_intakeSubsystem.ManualMove(0.1), m_intakeSubsystem)) //lower
            .onFalse(new RunCommand(()->m_intakeSubsystem.ManualMove(0), m_intakeSubsystem));
    */
        controller.circle().onTrue(
            new SequentialCommandGroup(
                new PositionCommand(m_intakeSubsystem, PositionCommand.Position.IntakeUp), 
                new RunCommand(()->m_intakeSubsystem.spinRollers(true), m_intakeSubsystem)));
        controller.cross().onTrue(new SequentialCommandGroup(
                new RunCommand(()->m_intakeSubsystem.spinRollers(false), m_intakeSubsystem),
                new PositionCommand(m_intakeSubsystem, PositionCommand.Position.IntakeDown)));

    }   

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();

        //return new PathPlannerAuto(autoChooser.getSelected());
        return Commands.sequence(
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
        );
    }
}
