// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sholder extends SubsystemBase {
  TalonFX sholderMotor = new TalonFX(0); //TODO: get device id
  TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
  Slot0Configs slot0 = talonFXConfigs.Slot0;
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  /** Creates a new Sholder. */
  public Sholder() {
    FeedbackConfigs fdb = talonFXConfigs.Feedback;
    fdb.SensorToMechanismRatio = 100; // TODO: find the right gear ratio

    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0.kI = 0; // no output for integrated error
    slot0.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 1; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 1; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1; // Target jerk of 1600 rps/s/s (0.1 seconds)

    sholderMotor.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveSholderTo(double position){
    sholderMotor.setControl(m_request.withPosition(position));
  }
  
  public void moveSholder(double speed) {
    sholderMotor.set(speed);
  }
}
