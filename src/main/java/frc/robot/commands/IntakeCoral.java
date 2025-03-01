// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCoral extends Command {
  private Intake intake;
  private Timer timer = new Timer();
  private Timer timeoutTimer = new Timer();
  private boolean isCompleted = false;
  private boolean timerStarted = false;
  public boolean hasCoral = false;
  private static double timeout = 4.0;

  /** Creates a new IntakeNote. */
  public IntakeCoral(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timeoutTimer.restart();
    intake.intakeIn();
    isCompleted = false;
    timerStarted = true;
    hasCoral = false;
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(intake.getFilteredCurent());
    if (!hasCoral) {
      hasCoral = (intake.getFilteredCurent() > 14) ? true : false;
    }
    if (hasCoral && timer.hasElapsed(1.0)) {
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
      intake.intakeOff();
      isCompleted = false;
      timerStarted = false;
  }

}
