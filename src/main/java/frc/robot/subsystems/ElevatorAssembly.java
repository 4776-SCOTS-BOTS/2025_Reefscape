// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorAssembly extends SubsystemBase {
  /** Creates a new Elevator Assembly. */

  public Intake intake;
  public ElevatorControlSubsystem elevatorControl;
  public ElevatorAssembly() {
    // intake = new Intake();
    elevatorControl = new ElevatorControlSubsystem();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
