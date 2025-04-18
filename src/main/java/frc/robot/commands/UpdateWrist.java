// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UpdateWrist extends Command {
  /** Creates a new UpdateWrist. */
private Intake intake;
private double targetPos;

  public UpdateWrist(Intake intake, double pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);

    this.intake = intake;
    targetPos = pos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.updateWrist(targetPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.wristMotor.set(0);
    intake.setWrist(targetPos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
