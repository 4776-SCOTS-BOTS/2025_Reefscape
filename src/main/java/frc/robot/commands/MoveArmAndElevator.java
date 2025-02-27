// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.customClass.SystemPositions;
import frc.robot.subsystems.ElevatorControlSubsystem;
import frc.robot.subsystems.Shoulder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveArmAndElevator extends Command {
  /** Creates a new MoveArmAndElevator. */
  ElevatorControlSubsystem elevator;
  Shoulder arm;
  SystemPositions.Positions position;

  public MoveArmAndElevator(ElevatorControlSubsystem elevator, Shoulder arm, SystemPositions.Positions position) {
    this.elevator = elevator;
    this.arm = arm;
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.moveToPosition(position.elevatorHeight);
    arm.moveSholderTo(position.armPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
