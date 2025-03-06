// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final TalonFX climbMotor;
  private final TalonFX tiltMotor;

  private double tiltRange = 5;

  public Climber() {
    climbMotor = new TalonFX(ElevatorConstants.ELEVATOR_LEADER_ID, "rio");
    tiltMotor = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID, "rio");

    TalonFXConfiguration climb_cfg = new TalonFXConfiguration();
    TalonFXConfiguration tilt_cfg = new TalonFXConfiguration();

    Slot0Configs slot0 = climb_cfg.Slot0;
    slot0.kS = 0.05; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.05; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.1; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 10; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kG = 0.2;

    MotorOutputConfigs leader_mo = climb_cfg.MotorOutput;
    leader_mo.Inverted = InvertedValue.Clockwise_Positive;
    leader_mo.NeutralMode = NeutralModeValue.Brake;

    climb_cfg.CurrentLimits.StatorCurrentLimit = 60; // This will help limit total torque the motor can apply to the mechanism. Could be too low for fast operation
    climb_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    climbMotor.getConfigurator().apply(climb_cfg);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
