// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
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
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // Chaged from 3/4  to 1 of a rotation per second max angular velocity
    public boolean fieldCentric = true;
    private double speedMultiplier = Constants.DriveConstants.driveNormalPercentScale;
    private double rotMultiplier = Constants.DriveConstants.rotNormalRateModifier;

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

    //subsytems
    private boolean hasElevator = false;

    private Elevator elevator;

    private final CommandXboxController driverCommandController = new CommandXboxController(Constants.Controllers.kDriverControllerPort);
    XboxController m_driverController = new XboxController(Constants.Controllers.kDriverControllerPort);

    private final CommandXboxController manipCommandController = new CommandXboxController(Constants.Controllers.kManipulatorControllerPort);

    JoystickButton brakeButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    POVButton resetGyro = new POVButton(m_driverController, 0); // Up on the D-Pad
    Trigger lowSpeedTrigger = driverCommandController.rightTrigger(0.5);
    Trigger reallylowSpeedTrigger = driverCommandController.leftTrigger(0.5);
    JoystickButton sprintTrigger = new JoystickButton(m_driverController, XboxController.Button.kRightStick.value);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {

        if(hasElevator){
        elevator = new Elevator();
        } else {
            elevator = null;
        }
        
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

       
        driverCommandController.leftBumper().onTrue(setFieldCent());
        driverCommandController.rightBumper().onTrue(setRobotCent());
        
        lowSpeedTrigger.onTrue(new InstantCommand(() -> {
            speedMultiplier = Constants.DriveConstants.driveLowPercentScale;
            rotMultiplier = Constants.DriveConstants.rotLowRateModifier;
        }))
                .onFalse(new InstantCommand(() -> {
                    speedMultiplier = Constants.DriveConstants.driveNormalPercentScale;
                    rotMultiplier = Constants.DriveConstants.rotNormalRateModifier;
                }));

        reallylowSpeedTrigger.onTrue(new InstantCommand(() -> {
            speedMultiplier = Constants.DriveConstants.driveLowPercentScale * 0.5;
            rotMultiplier = Constants.DriveConstants.rotLowRateModifier;
        }))
                .onFalse(new InstantCommand(() -> {
                    speedMultiplier = Constants.DriveConstants.driveNormalPercentScale;
                    rotMultiplier = Constants.DriveConstants.rotNormalRateModifier;
                }));

        sprintTrigger.onTrue(new InstantCommand(() -> {
            speedMultiplier = 1.0;
            rotMultiplier = 1.0;
        }))
                .onFalse(new InstantCommand(() -> {
                    speedMultiplier = Constants.DriveConstants.driveNormalPercentScale;
                    rotMultiplier = Constants.DriveConstants.rotNormalRateModifier;
                }));

        // brakeButton.whileTrue(drivetrain.applyRequest(() -> brake));
        // driverCommandController.y().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driverCommandController.getLeftY(), -driverCommandController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverCommandController.povRight().onTrue(new InstantCommand(() -> SignalLogger.start()));
        driverCommandController.povLeft().onTrue(new InstantCommand(() -> SignalLogger.stop()));

        driverCommandController.back().and(driverCommandController.b()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverCommandController.back().and(driverCommandController.a()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverCommandController.start().and(driverCommandController.b()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverCommandController.start().and(driverCommandController.a()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driverCommandController.x().onTrue(new MoveRobot(drivetrain, 1, 0, 0))
            .onFalse(new InstantCommand(driveRunnable, drivetrain));

        // reset the field-centric heading on left bumper press
        resetGyro.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        if (hasElevator) {
            manipCommandController.b().whileTrue(new InstantCommand(elevator.intake::intakeIn))
                    .onFalse(new InstantCommand(elevator.intake::intakeIn));
        }


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
                .withRotationalRate(-driverCommandController.getRightX() * MaxAngularRate * rotMultiplier) : // Drive counterclockwise with negative X (left)
                () -> driveRoboRel.withVelocityX(-driverCommandController.getLeftY() * MaxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
                .withVelocityY(-driverCommandController.getLeftX() * MaxSpeed * speedMultiplier) // Drive left with negative X (left)
                .withRotationalRate(-driverCommandController.getRightX() * MaxAngularRate * rotMultiplier) // Drive counterclockwise with negative X (left)
            );

        CommandScheduler.getInstance().schedule(driveCom);

    };

    // public Command driveDummy(){
    //     return drivetrain.runOnce(() -> dummyVar = 0);
    // }

    private Command setFieldCent(){
        return drivetrain.runOnce(() -> fieldCentric = true);
    }

    private Command setRobotCent(){
        return drivetrain.runOnce(() -> fieldCentric = false);
    }


}
