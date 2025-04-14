// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberNew;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDModes;
import frc.robot.subsystems.Backups.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbNew extends Command {
  /** Creates a new Climb. */
  private ClimberNew climber;
  private boolean isComplete = false;
  LEDSubsystem led;
  
  public ClimbNew(ClimberNew climber, LEDSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    
    this.climber = climber;
    this.led = led;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // climber.climbToHang();
    climber.autoClimb(-0.6);
    isComplete = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climber.climbMotor.getPosition().getValueAsDouble() - climber.climbPosition <=2 ) {
      isComplete = true;
      led.setMode(LEDModes.SPARKLE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.autoClimb(-0.05); // Hold voltage
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isComplete;
  }
  
}

