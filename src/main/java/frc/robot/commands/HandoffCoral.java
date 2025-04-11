// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HandoffCoral extends Command {
  private GroundIntake groundIntake;
  private Intake intake;

  private Timer timer = new Timer();
  private boolean isCompleted = false;
  private boolean timerStarted = false;

  /** Creates a new HandoffCoral. */
  public HandoffCoral(GroundIntake groundIntake, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(groundIntake, intake);
    this.groundIntake = groundIntake;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    groundIntake.rotateToHandoff();
    isCompleted = false;
    timerStarted = true;
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.hasElapsed(2.0)) {
      intake.intakeIn();
      intake.wristDeliver1();
    }
    if (timer.hasElapsed(2.5)) {
      groundIntake.intakeOut();
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
  }
}
