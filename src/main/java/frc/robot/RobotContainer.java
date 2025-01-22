// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.ExponentialProfile.ProfileTiming;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public boolean fieldCentric = false;
    private double speedMultiplier = 1.0;
    private double dummyVar = 0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveFieldRel = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRoboRel = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverCommandController = new CommandXboxController(0);
    XboxController m_driverController = new XboxController(0);

    JoystickButton brakeButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    POVButton resetGyro = new POVButton(m_driverController, 0); // Up on the D-Pad
    Trigger lowSpeedTrigger = driverCommandController.rightTrigger(0.5);
    Trigger reallylowSpeedTrigger = driverCommandController.leftTrigger(0.5);
    JoystickButton sprintTrigger = new JoystickButton(m_driverController, XboxController.Button.kRightStick.value);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            // driveCommand()
            new RunCommand(driveRunnable, drivetrain)
        );

       
        driverCommandController.leftBumper().onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> fieldCentric = true),
            driveDummy()));
        driverCommandController.rightBumper().onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> fieldCentric = false),
            driveDummy()));
        
            driverCommandController.rightTrigger().onTrue(new InstantCommand(() -> speedMultiplier = 0.5));

        brakeButton.whileTrue(drivetrain.applyRequest(() -> brake));
        driverCommandController.y().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverCommandController.getLeftY(), -driverCommandController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverCommandController.back().and(driverCommandController.b()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverCommandController.back().and(driverCommandController.a()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverCommandController.start().and(driverCommandController.b()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverCommandController.start().and(driverCommandController.a()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        resetGyro.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    Runnable driveRunnable = () -> {
        // System.out.println(fieldCentric);
        
        Command driveCom =
            drivetrain.applyRequest(fieldCentric ? 
                () -> driveFieldRel.withVelocityX(-driverCommandController.getLeftY() * MaxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
                .withVelocityY(-driverCommandController.getLeftX() * MaxSpeed * speedMultiplier) // Drive left with negative X (left)
                .withRotationalRate(-driverCommandController.getRightX() * MaxAngularRate) : // Drive counterclockwise with negative X (left)
                () -> driveRoboRel.withVelocityX(-driverCommandController.getLeftY() * MaxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
                .withVelocityY(-driverCommandController.getLeftX() * MaxSpeed * speedMultiplier) // Drive left with negative X (left)
                .withRotationalRate(-driverCommandController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            );

        CommandScheduler.getInstance().schedule(driveCom);

    };

    public Command driveDummy(){
        return drivetrain.runOnce(() -> dummyVar = 0);
    }

}
