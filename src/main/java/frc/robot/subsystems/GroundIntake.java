// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake. */
  public TalonFX groundIntakeMotor;
  public TalonFX rotateMotor;

  LinearFilter filter = LinearFilter.movingAverage(6);
  public double rawCurrent;
  public double filteredCurrent;

  public double packagePos = 0.28;
  public double pickupPos = 0;
  public double handoffPos = 0.22;

  PositionVoltage lowerRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
  PositionVoltage raiseRequest = new PositionVoltage(0).withSlot(1).withEnableFOC(true);


  public GroundIntake() {
    groundIntakeMotor = new TalonFX(Constants.IntakeConstants.groundIntakeRollerCANID, "TestBed");
    rotateMotor = new TalonFX(Constants.IntakeConstants.groundIntakeRotateCANID, "TestBed");

    TalonFXConfiguration rotate_cfg = new TalonFXConfiguration();

    MotorOutputConfigs rotate_mo = rotate_cfg.MotorOutput;
    rotate_mo.Inverted = InvertedValue.Clockwise_Positive;
    rotate_mo.NeutralMode = NeutralModeValue.Brake;

    rotate_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    rotate_cfg.Feedback.SensorToMechanismRatio = 15;


    Slot0Configs slot0 = rotate_cfg.Slot0;
    slot0.kS = 0.; // Add 0.25 V output to overcome static friction
    slot0.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 12; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = -0.5;

    Slot1Configs slot1 = rotate_cfg.Slot1;
    slot1.kS = 0.; // Add 0.25 V output to overcome static friction
    slot1.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot1.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot1.kP = 10; // A position error of 0.2 rotations results in 12 V output
    slot1.kI = 0; // No output for integrated error
    slot1.kD = 0; // A velocity error of 1 rps results in 0.5 V output
    slot1.GravityType = GravityTypeValue.Arm_Cosine;
    slot1.kG = 3.0;

    rotate_cfg.CurrentLimits.StatorCurrentLimit = 80; // This will help limit total torque the motor can apply to the
    rotate_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    rotateMotor.getConfigurator().apply(rotate_cfg);

    TalonFXConfiguration intake_cfg = new TalonFXConfiguration();

    MotorOutputConfigs intake_mo = intake_cfg.MotorOutput;
    intake_mo.Inverted = InvertedValue.Clockwise_Positive;
    intake_mo.NeutralMode = NeutralModeValue.Brake;

    groundIntakeMotor.getConfigurator().apply(intake_cfg);

    rotateMotor.setPosition(packagePos);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(DriverStation.isAutonomousEnabled()){
      rotateToPackage();
    }

    rawCurrent = getRawCurrent();
    filteredCurrent = filter.calculate(rawCurrent);
  }

  public void intakeIn() {
    groundIntakeMotor.set(0.4);
  }

  public void intakeOut() {
    groundIntakeMotor.set(-0.4);
  }

  public void intakeOff() {
    groundIntakeMotor.set(0);
  }

  public void rotateToPickup() {
    rotateMotor.setControl(lowerRequest.withPosition(pickupPos));
  }

  public void rotateToHandoff() {
    rotateMotor.setControl(raiseRequest.withPosition(handoffPos));
  }

  public void rotateToPackage() {
    rotateMotor.setControl(raiseRequest.withPosition(packagePos));
  }

  public double getFilteredCurrent() {
    return filteredCurrent;
  }

  public double getRawCurrent() {
    return groundIntakeMotor.getStatorCurrent().getValueAsDouble();
  }

  public void setRotateMotor(double speed) {
    rotateMotor.set(speed);
  }

  public boolean atHandoff(){
    return (Math.abs(rotateMotor.getPosition().getValueAsDouble() - handoffPos) <= 0.08);
  }
}
