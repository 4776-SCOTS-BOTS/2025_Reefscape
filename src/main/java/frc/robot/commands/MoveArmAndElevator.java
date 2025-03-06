// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.customClass.SystemPositions;
import frc.robot.subsystems.ElevatorControlSubsystem;
import frc.robot.subsystems.NOTShoulder;
import frc.robot.subsystems.ShoulderSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveArmAndElevator extends Command {
  /** Creates a new MoveArmAndElevator. */
  ElevatorControlSubsystem elevator;
  ShoulderSubsystem arm;
  SystemPositions.Positions position;
  double delay = 0;

  private Timer timer = new Timer();
  boolean isCompleted = false;

  public MoveArmAndElevator(ElevatorControlSubsystem elevator, ShoulderSubsystem arm, SystemPositions.Positions position, double delay) {
    this.elevator = elevator;
    this.arm = arm;
    this.position = position;
    this.delay = delay;
  }
  
  public MoveArmAndElevator(ElevatorControlSubsystem elevator, ShoulderSubsystem arm, SystemPositions.Positions position) {
    this(elevator, arm, position, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isCompleted = false;
    timer.restart();
    elevator.moveToPosition(position.elevatorHeight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.hasElapsed(delay)){
      arm.setArmGoal(position.armPosition);
      isCompleted = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isCompleted;
  }
}
