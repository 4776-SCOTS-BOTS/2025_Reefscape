// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathfindToReefTag extends Command {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(2*Math.PI).in(RadiansPerSecond); // Chaged from 3/4  to 1 of a rotation per second max angular velocity

  String limelight = null;
  Side side = null;

  Pose2d targetPose;
  boolean validTarget = false;

  PathConstraints constraints = new PathConstraints(
      3.0, 4.0,
      9.424778, 12.56637);
  Command pathfindingCommand;

  static final double translationTol = 0.01;
  static final double rotTol = 3;

    
    private CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.FieldCentric driveFieldRel = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

  /** Creates a new MoveRobot. */
  public PathfindToReefTag(CommandSwerveDrivetrain drivetrain, Side side, String limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.side = side;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PoseEstimate currentPoseEst = drivetrain.updateOdometryFromLL_CTRE(limelight);
    int nearestTag = drivetrain.getNearestTag(currentPoseEst);

    targetPose = FieldPositions.getTagCoord(nearestTag, side);

    System.out.println("Tag: " + nearestTag + " Pose: " + targetPose);

    if (targetPose != null) {
      pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints
      );
      validTarget = true;
    } else {
      validTarget = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (validTarget) {
      if (pathfindingCommand != null)
        pathfindingCommand.schedule();
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
    return pathfindingCommand.isFinished();
  }
}
