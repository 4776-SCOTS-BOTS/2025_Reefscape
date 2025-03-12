// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.customClass.FieldPositions;
import frc.robot.customClass.FieldPositions.Side;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.text.FieldPosition;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToReefTag extends Command {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(2*Math.PI).in(RadiansPerSecond); // Chaged from 3/4  to 1 of a rotation per second max angular velocity


  private static final double KpX = 6;//5
    private static final double KiX = 0;
    private static final double KdX = 0;

    private static final double KpY = 6;//5
    private static final double KiY = 0;
    private static final double KdY = 0;

    private static final double KpRot = 8;//6
    private static final double KiRot = 0;
    private static final double KdRot = 0;

    // private ProfiledPIDController pidControllerX = new ProfiledPIDController(KpX, KiX, KdX, new TrapezoidProfile.Constraints(MaxSpeed*0.5, MaxSpeed*3));
    // private ProfiledPIDController pidControllerY = new ProfiledPIDController(KpY, KiY, KdY, new TrapezoidProfile.Constraints(MaxSpeed*0.5, MaxSpeed*3));
    // private ProfiledPIDController pidControllerRot = new ProfiledPIDController(KpRot, KiRot, KdRot, new TrapezoidProfile.Constraints(MaxAngularRate * 0.5, 2*MaxAngularRate));

    private PIDController pidControllerX = new PIDController(KpX, KiX, KdX);
    private PIDController pidControllerY = new PIDController(KpY, KiY, KdY);
    private PIDController pidControllerRot = new PIDController(KpRot, KiRot, KdRot);


    private double setpointX = 0.0;
    private double setpointY = 0.0;
    private double setpointRot = 0.0;

    double outputX;
    double outputRot;
    double outputY;

    String limelight = null;
    Side side = null;

    Pose2d targetPose;
    boolean validTarget = false;

    static final double translationTol = 0.01;
    static final double rotTol = 3;

    
    private CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.FieldCentric driveFieldRel = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

  /** Creates a new MoveRobot. */
  public DriveToReefTag(CommandSwerveDrivetrain drivetrain, Side side, String limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.side = side;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PoseEstimate currentLoc = drivetrain.updateOdometryFromLL_CTRE(limelight);
    int nearestTag = drivetrain.getNearestTag(currentLoc);

    targetPose = FieldPositions.getTagCoord(nearestTag, side);

    // System.out.println("Tag: " + nearestTag + " Pose: " + targetPose);

    if (targetPose != null) {

      setpointX = targetPose.getX();
      setpointY = targetPose.getY();
      setpointRot = targetPose.getRotation().getRadians();

      pidControllerX.setTolerance(translationTol);
      pidControllerY.setTolerance(translationTol);
      pidControllerRot.setTolerance(Math.toRadians(rotTol));

      // pidControllerX.setGoal(setpointX);
      // pidControllerY.setGoal(setpointY);
      // pidControllerRot.setGoal(setpointRot);

      pidControllerX.setSetpoint(setpointX);
      pidControllerY.setSetpoint(setpointY);
      pidControllerRot.setSetpoint(setpointRot);


      // pidControllerX.reset(drivetrain.getState().Pose.getX());
      // pidControllerY.reset(drivetrain.getState().Pose.getY());
      // pidControllerRot.reset(drivetrain.getState().Pose.getRotation().getRadians());

      validTarget = true;
    } else {
      validTarget = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (validTarget) {
      double currentPositionX = drivetrain.getState().Pose.getX();
      double currentPositionY = drivetrain.getState().Pose.getY();
      double currentRotation = drivetrain.getState().Pose.getRotation().getRadians();

      // System.out.println("Current Pose: " + drivetrain.getState().Pose);

      // Calculate the PID outputs
      double outputX = pidControllerX.calculate(currentPositionX);
      double outputY = pidControllerY.calculate(currentPositionY);
      double outputRot = pidControllerRot.calculate(currentRotation);

      // SmartDashboard.putNumber("X Error", pidControllerX.getPositionError());
      // SmartDashboard.putNumber("X Error Tol", pidControllerX.getPositionTolerance());
      // SmartDashboard.putNumber("X Output", outputX);
      // SmartDashboard.putBoolean("X at Setpoint", pidControllerX.atSetpoint());
      // SmartDashboard.putNumber("Y Error", pidControllerY.getPositionError());
      // SmartDashboard.putNumber("Y Error Tol", pidControllerY.getPositionTolerance());
      // SmartDashboard.putNumber("Y Output", outputY);
      // SmartDashboard.putBoolean("Y at Setpoint", pidControllerY.atSetpoint());      
      // SmartDashboard.putNumber("Rot Error", pidControllerRot.getPositionError());
      // SmartDashboard.putNumber("Rot Error Tol", pidControllerRot.getPositionTolerance());
      // SmartDashboard.putNumber("Rot Output", outputRot);
      // SmartDashboard.putBoolean("Rot at Setpoint", pidControllerRot.atSetpoint());






      // System.out.println("Current X: " + currentPositionX + " Current Y: " +
      // currentPositionY + " pidx: " + outputX + " pidy: " + outputY);
      // System.out.println("X Error: " + pidControllerX.getError() + " Xpos" +
      // currentPositionX);

      drivetrain.applyRequestMethod(
          () -> driveFieldRel.withVelocityX(outputX) // Drive forward with negative Y (forward)
              .withVelocityY(outputY) // Drive left with negative X (left)
              .withRotationalRate(outputRot));// Drive counterclockwise with negative X (left)

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("DONE!" + interrupted);
    drivetrain.applyRequestMethod(
                () -> driveFieldRel.withVelocityX(0) // Drive forward with negative Y (forward)
                .withVelocityY(0) // Drive left with negative X (left)
                .withRotationalRate(0));// Drive counterclockwise with negative X (left)
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (pidControllerX.atSetpoint() && pidControllerY.atSetpoint() && pidControllerRot.atSetpoint()) || !validTarget;
  }
}
