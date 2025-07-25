// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.customClass.SystemPositions.Positions;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShoulderSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HandoffCoral extends Command {
  private GroundIntake groundIntake;
  private Intake intake;
  private ShoulderSubsystem arm;

  private Timer timer = new Timer();
  private boolean isCompleted = false;
  private boolean hasHandoff = false;
  private boolean movedArm = false;
  private boolean timerStarted = false;

  /** Creates a new HandoffCoral. */
  public HandoffCoral(GroundIntake groundIntake, Intake intake, ShoulderSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(groundIntake, intake, arm);
    this.groundIntake = groundIntake;
    this.intake = intake;
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    groundIntake.rotateToHandoff();
    isCompleted = false;
    hasHandoff = false;
    movedArm = false;
    timerStarted = true;
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(groundIntake.atHandoff() && !hasHandoff) {
      intake.intakeIn();
      groundIntake.intakeOut();
      timer.restart();
      hasHandoff = true;
    }
    if (hasHandoff && timer.hasElapsed(0.1) && !movedArm) {
      arm.setArmGoal(Positions.AFTER_HANDOFF.armPosition);
      groundIntake.rotateToPackage();
      // groundIntake.intakeOff();
      movedArm = true;
      timer.restart();
    }
    if (movedArm && timer.hasElapsed(0.4)){
      groundIntake.intakeOff();
      intake.intakeOff();
      isCompleted = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isCompleted;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    groundIntake.intakeOff();
    intake.intakeOff();
    groundIntake.rotateToPackage();
  }
}
