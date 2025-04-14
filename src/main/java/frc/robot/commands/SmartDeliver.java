// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartDeliver extends Command {
  private Intake intake;
  public boolean isCompleted = false;

  /** Creates a new SmartDeliver. */
  public SmartDeliver(Intake intake) {
    this.intake = intake;  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (intake.getCoralPosition()) {
      case beltSide:
        intake.wristDeliver2();
        break;
      case notBeltSide:
        intake.wristDeliver1();
        break;
      case center:
        intake.wristDeliver2();
        break;
      case empty:
        intake.wristDeliver2();
        break;
    }
    isCompleted = true;
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
