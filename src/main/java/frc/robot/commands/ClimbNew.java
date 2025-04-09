// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDModes;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbNew extends Command {
  /** Creates a new Climb. */
  private Climber climber;
  private boolean isComplete = false;
  LEDSubsystem led;
  
  public ClimbNew(Climber climber, LEDSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    
    this.climber = climber;
    this.led = led;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.autoClimb(-0.25);
    isComplete = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climber.climbMotor.getPosition().getValueAsDouble() >= climber.climbPosition) {
      climber.autoClimb(-0.8);
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
    led.setMode(LEDModes.SPARKLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isComplete;
  }
  
}
