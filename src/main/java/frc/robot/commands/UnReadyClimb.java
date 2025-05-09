// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Backups.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UnReadyClimb extends Command {
  /** Creates a new Climb. */
  private Climber climber;
  private boolean startTilt = false;
  private boolean isComplete = false;
  
  public UnReadyClimb(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);

    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("UNReady Climber");
    climber.autoTilt(-0.4);
    startTilt = false;
    isComplete = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber.tiltMotor.getPosition().getValueAsDouble() > 0) {
      climber.autoTilt(-0.4);
    } else {
      climber.autoTilt(0);
      isComplete = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.autoTilt(0);
    climber.autoClimb(0);
    climber.climberMode = Climber.ClimberMode.RUN_TO_POSITION;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isComplete;
  }
}
