// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Backups.Climber;
import frc.robot.subsystems.LEDSubsystem.LEDModes;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberNew extends Climber {
  
  LEDSubsystem m_led;
  PositionVoltage m_request = new PositionVoltage(0).withSlot(0).withEnableFOC(true);

  /** Creates a new ClimberNew. */
  public ClimberNew() {

    climbMotor = new TalonFX(Constants.ClimberConstants.climberMotorCANID, "rio");
    tiltMotor = null;

    climbPosition = 85; //40
    climbReadyPosition = 230; //190
    tiltRange = 0;

    TalonFXConfiguration climb_cfg = new TalonFXConfiguration();

    MotorOutputConfigs climb_mo = climb_cfg.MotorOutput;
    climb_mo.Inverted = InvertedValue.Clockwise_Positive;
    climb_mo.NeutralMode = NeutralModeValue.Brake;

    VoltageConfigs climb_volts = climb_cfg.Voltage;
    climb_volts.PeakForwardVoltage = 12;

    Slot0Configs slot0 = climb_cfg.Slot0;
    slot0.kS = 0.; // Add 0.25 V output to overcome static friction
    slot0.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 7; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kG = 1.25;

    climb_cfg.CurrentLimits.StatorCurrentLimit = 90; // This will help limit total torque the motor can apply to the
                                                     // mechanism. Could be too low for fast operation
    climb_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    climbMotor.getConfigurator().apply(climb_cfg);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void runClimber(double speed) {
    if (speed > 0 && climbMotor.getPosition().getValueAsDouble() >= climbReadyPosition * 1.1) {
      speed = 0;
    } else if (speed < 0 && climbMotor.getPosition().getValueAsDouble() <= 0) {
      speed = 0;
    }
    climbMotor.set(speed);
  }

  // No tilt motor so tilt methods are empty
  @Override
  public void runTilt(double speed) {
    // Do nothing
  }

  @Override
  public void manualTilt(double speed) {
    // Do nothing
  }

  @Override
  public void autoTilt(double speed) {
    // Do nothing
  }

  @Override
  public void manualClimb(double speed) {
    runClimber(speed);
    climberMode = ClimberMode.MANUAL;
  }

  @Override
  public void autoClimb(double speed) {
    runClimber(speed);
    climberMode = ClimberMode.RUN_TO_POSITION;
    // m_led.setMode(LEDModes.SPARKLE);
  }

  public void climbToPosition(double position) {
    climbMotor.setControl(m_request.withPosition(position));
  }

  public void climbToHang(){
    climbToPosition(climbPosition);
  }
}
