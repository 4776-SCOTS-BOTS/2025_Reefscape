// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.customClass.SystemPositions.Positions;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.ExponentialProfile.ProfileTiming;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DeliverCoral;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.MoveArmAndElevator;
import frc.robot.commands.MoveRobot;
import frc.robot.customClass.SystemPositions.Positions;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorControlSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ElevatorControlSubsystem.ElevatorMode;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderMode;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // Chaged from 3/4  to 1 of a rotation per second max angular velocity
    public boolean fieldCentric = true;
    private double speedMultiplier = Constants.DriveConstants.driveNormalPercentScale;
    private double rotMultiplier = Constants.DriveConstants.rotNormalRateModifier;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveFieldRel = new SwerveRequest.FieldCentric()
            // .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRoboRel = new SwerveRequest.RobotCentric()
            // .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //Stick scaling factors
    private double deadband = 0.05;
    private double scaleFactor = 1/(1 - deadband);
    private double offset = 1 - scaleFactor;
    private double cubicWeight = 0.5;  

    private double dpadSpeed = 0.3;


    //subsytems
    private boolean hasElevator = true;
    private ElevatorControlSubsystem elevator;

    private boolean hasShoulder = true;
    private ShoulderSubsystem shoulder;

    private boolean hasIntake = true;
    private Intake intake;

    private boolean hasClimber = false;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //Controls
    private final CommandXboxController driverCommandController = new CommandXboxController(Constants.Controllers.kDriverControllerPort);
    XboxController m_driverController = new XboxController(Constants.Controllers.kDriverControllerPort);

    //private final CommandXboxController manipCommandController = new CommandXboxController(Constants.Controllers.kManipulatorControllerPort);
    private final CommandGenericHID manipCommandController = new CommandGenericHID(Constants.Controllers.kManipulatorControllerPort);

    // Testing Controller -- Comment out for comps
    private CommandXboxController testCommandXboxController = new CommandXboxController(4);

    //Driver Controls
    JoystickButton brakeButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    // POVButton resetGyro = new POVButton(m_driverController, 0); // Up on the D-Pad
    Trigger resetGyro = new Trigger(() -> m_driverController.getStartButton());
    Trigger lowSpeedTrigger = driverCommandController.rightTrigger(0.5);
    Trigger reallylowSpeedTrigger = driverCommandController.leftTrigger(0.5);
    JoystickButton sprintTrigger = new JoystickButton(m_driverController, XboxController.Button.kRightStick.value);

    // d-pad field-rel Movement
    POVButton dpadUp = new POVButton(m_driverController, 0);
    POVButton dpadRight = new POVButton(m_driverController, 90);
    POVButton dpadDown = new POVButton(m_driverController, 180);
    POVButton dpadLeft = new POVButton(m_driverController, 270);

    //Manipulator contorls
    private Trigger placeCoral =  manipCommandController.axisGreaterThan(Constants.rightTrigger, 0.5);
    private Trigger holdAlgae =  manipCommandController.axisGreaterThan(Constants.leftTrigger, 0.5);


    //Shuffleboard
    ShuffleboardLayout elevatorLayout;
    ShuffleboardLayout intakeLayout;
    ShuffleboardLayout shoulderLayout;

    public RobotContainer() {
        //Setup Elevator if present
        if(hasElevator){
            elevator = new ElevatorControlSubsystem();
            elevatorLayout = Shuffleboard.getTab("Subsystems").getLayout("ElevatorControl", BuiltInLayouts.kList);
            elevator.addDashboardWidgets(elevatorLayout);
        } else {
            elevator = null;
        }

        if(hasShoulder){
            shoulder = new ShoulderSubsystem();
            shoulderLayout = Shuffleboard.getTab("Subsystems").getLayout("Shoulder", BuiltInLayouts.kList);
            shoulder.addDashboardWidgets(shoulderLayout);
        } else {
            shoulder = null;
        }

        if(hasIntake){
            intake = new Intake();
            intakeLayout = Shuffleboard.getTab("Subsystems").getLayout("Intake", BuiltInLayouts.kList);
            intake.addDashboardWidgets(intakeLayout);
        } else {
            intake = null;
        }

        //Setup Climber if present
        // if(hasClimber){
        //     climber = new Clkimber();
        //     } else {
        //         elevator = null;
        //     }
        
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

        if (hasElevator) {
            elevator.setDefaultCommand(
                    new RunCommand(elevatorRunnable, elevator));

            manipCommandController.pov(90).onTrue(new InstantCommand(() -> elevator.moveToPosition(1), elevator));
            manipCommandController.pov(180).onTrue(new InstantCommand(() -> elevator.parkElevator(), elevator));
            

        if (hasShoulder){
            shoulder.setDefaultCommand(
                new RunCommand(shoulderRunnable, shoulder));

            manipCommandController.button(Constants.rightMenuButton).onTrue(new InstantCommand(() -> {shoulder.setStraightBack();}));
            manipCommandController.button(Constants.leftMenuButton).onTrue(new InstantCommand(() -> {shoulder.setStraightUp();}));            
        }

        if (hasIntake) {
            manipCommandController.button(Constants.leftButton).onTrue(new IntakeCoral(intake));
            manipCommandController.button(Constants.rightButton).onTrue(new InstantCommand(intake::intakeOutFast, intake));
            manipCommandController.button(Constants.bottomButton).onTrue(new InstantCommand(intake::intakeOff, intake));
            manipCommandController.button(Constants.topButton).onTrue(new InstantCommand(intake::wristPickup, intake));
            manipCommandController.button(Constants.leftBumper).onTrue(new InstantCommand(intake::wristDeliver1, intake));
            manipCommandController.button(Constants.rightBumper).onTrue(new InstantCommand(intake::wristDeliver2, intake));
            holdAlgae.onTrue(new InstantCommand(intake::intakeAlgae))
            .onFalse(new InstantCommand(intake::intakeOut));


            // Manual Controls
            // manipCommandController.button(Constants.rightBumper)
            //         .onTrue(new InstantCommand(() -> intake.runWrist(0.35)))
            //         .onFalse(new InstantCommand(() -> intake.runWrist(0)));

            // manipCommandController.button(Constants.leftBumper)
            //         .onTrue(new InstantCommand(() -> intake.runWrist(-0.35)))
            //         .onFalse(new InstantCommand(() -> intake.runWrist(0)));

        }

        if(hasIntake && hasShoulder){
            placeCoral.onTrue(new DeliverCoral(intake, shoulder, false));
        }

        if(hasElevator && hasShoulder){
            manipCommandController.pov(270)
                .onTrue(new MoveArmAndElevator(elevator, shoulder, Positions.SAFE_STATION))
                .onFalse(new MoveArmAndElevator(elevator, shoulder, Positions.INTAKE_STATION));

            manipCommandController.pov(0).onTrue(new MoveArmAndElevator(elevator, shoulder, Positions.L4_READY));
            manipCommandController.pov(90).onTrue(new MoveArmAndElevator(elevator, shoulder, Positions.L3_READY));
            manipCommandController.pov(180).onTrue(new MoveArmAndElevator(elevator, shoulder, Positions.L2_READY));
                
        
        }
        }

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
            rotMultiplier = Constants.DriveConstants.rotLowRateModifier*0.6;
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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // testCommandXboxController.povRight().onTrue(new InstantCommand(() -> SignalLogger.start()));
        // testCommandXboxController.povLeft().onTrue(new InstantCommand(() -> SignalLogger.stop()));

        // testCommandXboxController.back().and(testCommandXboxController.b()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // testCommandXboxController.back().and(testCommandXboxController.a()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // testCommandXboxController.start().and(testCommandXboxController.b()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // testCommandXboxController.start().and(testCommandXboxController.a()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        // Module pointing controller ... not useful for actual driving?
        // driverCommandController.y().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(
        //     new Rotation2d(-driverCommandController.getLeftY(), -driverCommandController.getLeftX()))));
    

        dpadUp.whileTrue(drivetrain.applyRequest(   () -> driveRoboRel.withVelocityX(dpadSpeed).withVelocityY(0).withRotationalRate(0)));
        dpadRight.whileTrue(drivetrain.applyRequest(() -> driveRoboRel.withVelocityX(0).withVelocityY(-dpadSpeed).withRotationalRate(0)));
        dpadDown.whileTrue(drivetrain.applyRequest( () -> driveRoboRel.withVelocityX(-dpadSpeed).withVelocityY(0).withRotationalRate(0)));
        dpadLeft.whileTrue(drivetrain.applyRequest( () -> driveRoboRel.withVelocityX(0).withVelocityY(dpadSpeed).withRotationalRate(0)));
        
        brakeButton.whileTrue(drivetrain.applyRequest(() -> brake));



        // reset the field-centric heading
        resetGyro.onTrue(drivetrain.runOnce(drivetrain::zeroFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);    

        // Development Commands
        // driverCommandController.a().onTrue(new MoveRobot(drivetrain, 1, 0, 0))
        // .onFalse(new InstantCommand(driveRunnable, drivetrain));
        // driverCommandController.b().onTrue(new InstantCommand(() -> drivetrain.forcePoseUpdate("limelight-front")));




        // if (hasElevator) {
        //     manipCommandController.button(Constants.rightButton).whileTrue(new InstantCommand(elevator.intake::intakeIn))
        //             .onFalse(new InstantCommand(elevator.intake::intakeIn));
        // }


    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    // Default Runnables
    Runnable driveRunnable = () -> {
        // System.out.println(fieldCentric);
        // Can we use deferredCommand here?  Or maybe SelectCommand (probably ConditionalCommand)
        
        Command driveCom =
            drivetrain.applyRequest(fieldCentric ? 
                () -> driveFieldRel.withVelocityX(scaleStick(-driverCommandController.getLeftY(), cubicWeight) * MaxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
                .withVelocityY(scaleStick(-driverCommandController.getLeftX(), cubicWeight) * MaxSpeed * speedMultiplier) // Drive left with negative X (left)
                .withRotationalRate(scaleStick(-driverCommandController.getRightX(), cubicWeight) * MaxAngularRate * rotMultiplier) : // Drive counterclockwise with negative X (left)
                () -> driveRoboRel.withVelocityX(scaleStick(-driverCommandController.getLeftY(), cubicWeight) * MaxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
                .withVelocityY(scaleStick(-driverCommandController.getLeftX(), cubicWeight) * MaxSpeed * speedMultiplier) // Drive left with negative X (left)
                .withRotationalRate(scaleStick(-driverCommandController.getRightX(), cubicWeight) * MaxAngularRate * rotMultiplier) // Drive counterclockwise with negative X (left)
            );

        CommandScheduler.getInstance().schedule(driveCom);

    };

    Runnable elevatorRunnable = () ->{
        double elevatorStick = MathUtil.applyDeadband(-manipCommandController.getRawAxis(Constants.leftStickY), 0.03);

        if(elevatorStick == 0 && elevator.getMode() != ElevatorMode.RUN_TO_POSITION){
            elevator.moveElevator(0);
        } else if (elevatorStick !=0) {
            elevator.moveElevator(elevatorStick);
        }

    };

    Runnable shoulderRunnable = () -> {
    double shoulderStick = MathUtil.applyDeadband(manipCommandController.getRawAxis(Constants.rightStickX), 0.03);

    if ((Math.abs(shoulderStick) < 0.03) && shoulder.shoulderMode == ShoulderMode.MANUAL){
      shoulder.runMotor(0);
    } else if (Math.abs(shoulderStick) >= 0.03) {
      shoulder.runMotor(shoulderStick * 0.5);
    }
  };

    private Command setFieldCent(){
        return drivetrain.runOnce(() -> fieldCentric = true);
    }

    private Command setRobotCent(){
        return drivetrain.runOnce(() -> fieldCentric = false);
    }

    private double scaleStick(double input, double weight) {
        input = MathUtil.applyDeadband(input, deadband);
        if (input != 0) {
            input = Math.signum(input) * (scaleFactor * Math.abs(input) + offset);
        }

        double cubedInput = Math.pow(input, 3);
        double remainingWeight = 1 - weight;

        return (weight * cubedInput) + (remainingWeight * input);

    }


}
