// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorControlSubsystem;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RemoveAlgaeHigh extends Command {
  /** Creates a new RemoveAlgae. */
  private ElevatorControlSubsystem elevator;
  private Intake intake;
  private Timer timer = new Timer();
  double timeout = 2;

  public RemoveAlgaeHigh(ElevatorControlSubsystem elevator, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, intake);
    this.elevator = elevator;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double newPostion = elevator.getElevatorPosition() - 0.10;
    intake.intakeOutFast();
    elevator.moveToPosition(newPostion);
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeout);
  }
}
