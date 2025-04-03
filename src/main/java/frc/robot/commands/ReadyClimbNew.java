// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReadyClimbNew extends Command {
  /** Creates a new Climb. */
  private Climber climber;
  private boolean isComplete = false;
  
  public ReadyClimbNew(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);

    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Ready Climber");
    climber.autoClimb(0.75);
    isComplete = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber.climbMotor.getPosition().getValueAsDouble() <= climber.climbReadyPosition) {
      climber.autoClimb(0.75);
    } else {
      climber.autoClimb(0);
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
