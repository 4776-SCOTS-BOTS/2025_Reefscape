// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveRobotProfiledRobotRelative extends Command {
  private static final double KpX = 1.0;
    private static final double KiX = 0;
    private static final double KdX = 0;

    private static final double KpY = 0.5;
    private static final double KiY = 0;
    private static final double KdY = 0;

    private static final double KpRot = 0.2;
    private static final double KiRot = 0;
    private static final double KdRot = 0;

    private ProfiledPIDController pidControllerX = new ProfiledPIDController(KpX, KiX, KdX, new TrapezoidProfile.Constraints(0.5, 0.5));
    private ProfiledPIDController pidControllerY = new ProfiledPIDController(KpY, KiY, KdY, new TrapezoidProfile.Constraints(0.5, 0.5));
    private ProfiledPIDController pidControllerRot = new ProfiledPIDController(KpRot, KiRot, KdRot, new TrapezoidProfile.Constraints(0.5, 0.5));

    private double setpointX = 0.0;
    private double setpointY = 0.0;
    private double setpointRot = 0.0;

    private double targetX;
    private double targetY;
    private double targetRot;

    double outputX;
    double outputRot;
    double outputY;

    static final double translationTol = 0.05;
    static final double rotTol = 5;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // Chaged from 3/4  to 1 of a rotation per second max angular velocity

    private CommandSwerveDrivetrain drivetrain;

    // private final SwerveRequest.FieldCentric driveFieldRel = new SwerveRequest.FieldCentric()
    // .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final SwerveRequest.RobotCentric driveRoboRel = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  /** Creates a new MoveRobot. */
  public MoveRobotProfiledRobotRelative(CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    // set target pose to 3 meters in front of the april tag... I think
    Pose3d targetPose = LimelightHelpers.getBotPose3d_TargetSpace("LimelightFront");
    targetPose = new Pose3d(targetPose.getX() + 3.0, 0.0, 0.0, new Rotation3d(new Rotation2d(0.0)));
    
    this.drivetrain = drivetrain;
    this.targetX = targetPose.getX();
    this.targetY = targetPose.getY();
    this.targetRot = targetPose.getRotation().getAngle();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetPose(new Pose2d());

    setpointX = targetX;
    setpointY = targetY;
    setpointRot = targetRot;

    pidControllerX.setTolerance(translationTol);
    pidControllerY.setTolerance(translationTol);
    pidControllerX.setTolerance(Math.toRadians(rotTol));
    
    pidControllerX.setGoal(setpointX);
    pidControllerY.setGoal(setpointY);
    pidControllerRot.setGoal(setpointRot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPositionX = drivetrain.getState().Pose.getX();
    double currentPositionY = drivetrain.getState().Pose.getY();
    double currentRotation  = drivetrain.getState().Pose.getRotation().getRadians();

    pidControllerX.setTolerance(translationTol);
    pidControllerY.setTolerance(translationTol);
    pidControllerRot.setTolerance(Math.toRadians(rotTol));

    // Calculate the PID outputs
    double outputX = MathUtil.clamp(pidControllerX.calculate(currentPositionX), -0.25, +0.25);
    double outputY = MathUtil.clamp(pidControllerY.calculate(currentPositionY), -0.25, +0.25);
    double outputRot = MathUtil.clamp(pidControllerRot.calculate(currentRotation), -0.25, +0.25);

    // SmartDashboard.putNumber("X Error", pidControllerX.getPositionError());
    // SmartDashboard.putNumber("X Error Tol", pidControllerX.getPositionTolerance());
    // SmartDashboard.putNumber("X Output", outputX);
    // SmartDashboard.putBoolean("X at Setpoint", pidControllerX.atSetpoint());

    // System.out.println("Current X: " + currentPositionX + " Current Y: " + currentPositionY +  " pidx: " + outputX + " pidy: " + outputY);
    // System.out.println("X Error: " + pidControllerX.getError() + " Xpos" + currentPositionX);

    drivetrain.applyRequestMethod(
                () -> driveRoboRel.withVelocityX(outputX * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(outputY * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(outputRot * MaxAngularRate));// Drive counterclockwise with negative X (left)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DONE!");
    drivetrain.applyRequestMethod(
                () -> driveRoboRel.withVelocityX(0) // Drive forward with negative Y (forward)
                .withVelocityY(0) // Drive left with negative X (left)
                .withRotationalRate(0));// Drive counterclockwise with negative X (left)
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidControllerX.atSetpoint();// && pidControllerY.atSetpoint() && pidControllerRot.atSetpoint();
  }
}
