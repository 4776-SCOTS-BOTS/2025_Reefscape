// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberNew extends Climber {
  /** Creates a new ClimberNew. */
  public ClimberNew() {
    climbMotor = new TalonFX(Constants.ClimberConstants.climberMotorCANID, "rio");
    tiltMotor = null;

    climbPosition = 160;// Actual 160
    climbReadyPosition = 285;
    tiltRange = 0;

    TalonFXConfiguration climb_cfg = new TalonFXConfiguration();

    MotorOutputConfigs climb_mo = climb_cfg.MotorOutput;
    climb_mo.Inverted = InvertedValue.Clockwise_Positive;
    climb_mo.NeutralMode = NeutralModeValue.Brake;

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
  }
}
