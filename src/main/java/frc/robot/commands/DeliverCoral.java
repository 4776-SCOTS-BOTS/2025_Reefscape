// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShoulderSubsystem;

public class DeliverCoral extends Command {
  private Intake intake;
  private ShoulderSubsystem shoulder;

  private Timer timer = new Timer();
  private Timer timeoutTimer = new Timer();
  private boolean isCompleted = false;
  private boolean timerStarted = false;
  public boolean hasCoral = false;
  private static double timeout = 4.0;
  private double armStart;

  private double ARM_OFFSET = 0.15;

  /** Creates a new IntakeNote. 
   * @param clockwiseArm Shoulde be false for front delivery, true for back delivery
  */
  public DeliverCoral(Intake intake, ShoulderSubsystem shoulder, boolean clockwiseArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shoulder = shoulder;

    if (!clockwiseArm) {
      ARM_OFFSET = -0.1;
    }

    addRequirements(intake, shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timeoutTimer.restart();
    armStart = shoulder.getCurrentPosition();
    shoulder.setArmGoal(armStart + ARM_OFFSET);
    isCompleted = false;
    timerStarted = true;
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(0.4
    )) {
      intake.intakeOut();
    }
    if (timer.hasElapsed(1.5)) {
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
    if (isCompleted == true) {
      intake.intakeOff();
      isCompleted = false;
      timerStarted = false;
    }
  }

  public static void setTimeout(double newTimeout){
    timeout = newTimeout;
  }
}
