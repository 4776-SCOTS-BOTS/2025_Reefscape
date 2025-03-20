// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToRange extends Command {
  /** Creates a new AlignToRange. */
  private Timer timer = new Timer();
  private boolean isCompleted = false;
  private double driveSpeed = 0.25;
  private boolean isL4;
  private boolean isLeft;

  private double DELAY = 0.25;

  private CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.RobotCentric driveRoboRel = new SwerveRequest.RobotCentric()
  // .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors;

  public AlignToRange(CommandSwerveDrivetrain drivetrain, boolean isLeft, boolean isL4) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.isL4 = isL4;
    this.isLeft = isLeft;
    driveSpeed = isLeft ? driveSpeed : -driveSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!isL4) {
      if (isLeft) {
        DELAY = DELAY * 0.55;
      } else {
        DELAY = DELAY * 1.1;
      }
    }
    drivetrain.applyRequestMethod(() -> driveRoboRel.withVelocityX(0).withVelocityY(driveSpeed).withRotationalRate(0));
    timer.stop();
    timer.reset();
    isCompleted = false;
    if (isL4 ? drivetrain.isArmInRangeL4() : drivetrain.isArmInRangeLower()) {
      isCompleted = true;
    }
    // System.out.println("Is L4: " + isL4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivetrain.applyRequestMethod(() -> driveRoboRel.withVelocityX(0).withVelocityY(driveSpeed).withRotationalRate(0));
    if (!timer.isRunning()) {
      if (isL4 ? drivetrain.isArmInRangeL4() : drivetrain.isArmInRangeLower()) {
        timer.restart();
      }
      drivetrain.applyRequestMethod(() -> driveRoboRel.withVelocityX(0).withVelocityY(driveSpeed).withRotationalRate(0));
    } else if (timer.hasElapsed(DELAY)) {
      isCompleted = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("AlignToRange ended");
    drivetrain.applyRequestMethod(() -> driveRoboRel.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isCompleted;
  }
}
