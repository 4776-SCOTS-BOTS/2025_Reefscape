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

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public final TalonFX climbMotor;
  public final TalonFX tiltMotor;

  public double climbPosition = 23;
  public double tiltRange = 11.0;//15

  public enum ClimberMode {
    RUN_TO_POSITION,
    MANUAL
  }

  public ClimberMode climberMode = ClimberMode.MANUAL;

  

  public Climber() {
    climbMotor = new TalonFX(Constants.ClimberConstants.climberMotorCANID, "rio");
    tiltMotor = new TalonFX(Constants.ClimberConstants.tiltMotorCANID, "rio");

    TalonFXConfiguration climb_cfg = new TalonFXConfiguration();
    TalonFXConfiguration tilt_cfg = new TalonFXConfiguration();

    MotorOutputConfigs climb_mo = climb_cfg.MotorOutput;
    climb_mo.Inverted = InvertedValue.CounterClockwise_Positive;
    climb_mo.NeutralMode = NeutralModeValue.Brake;

    climb_cfg.CurrentLimits.StatorCurrentLimit = 90; // This will help limit total torque the motor can apply to the mechanism. Could be too low for fast operation
    climb_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    climbMotor.getConfigurator().apply(climb_cfg);

    tilt_cfg = climb_cfg;
    tilt_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tilt_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tiltMotor.getConfigurator().apply(climb_cfg);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runClimber(double speed){
    if(speed > 0 && climbMotor.getPosition().getValueAsDouble() >= climbPosition){
      speed = 0;
    } else if(speed < 0 && climbMotor.getPosition().getValueAsDouble() <= 0){
      speed = 0;
    }
    climbMotor.set(speed);
  }

  public void manualClimb(double speed){
    runClimber(speed);
    climberMode = ClimberMode.MANUAL;
  }

  public void autoClimb(double speed){
    runClimber(speed);
    climberMode = ClimberMode.RUN_TO_POSITION;
  }

  public void runTilt(double speed){
    if(speed > 0 && tiltMotor.getPosition().getValueAsDouble() >= tiltRange*1.5){
      speed = 0;
    } else if(speed < 0 && tiltMotor.getPosition().getValueAsDouble() <= -0.5){
      speed = 0;
    }   
    tiltMotor.set(speed);
  }

  public void manualTilt(double speed){
    runTilt(speed);
    climberMode = ClimberMode.MANUAL;
  }
  public void autoTilt(double speed){
    runTilt(speed);
    climberMode = ClimberMode.RUN_TO_POSITION;
  }



}
