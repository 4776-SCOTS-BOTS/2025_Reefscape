// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake. */
  public SparkMax groundIntakeMotor;
  public TalonFX rotateMotor;

  LinearFilter filter = LinearFilter.movingAverage(6);
  public double rawCurrent;
  public double filteredCurrent;

  public double pickupPos = 0;
  public double handoffPos = 0;

  public GroundIntake() {
    groundIntakeMotor = new SparkMax(Constants.IntakeConstants.groundIntakeRollerCANID, MotorType.kBrushless);
    rotateMotor = new TalonFX(Constants.IntakeConstants.groundIntakeRotateCANID, "rio");

    SparkMaxConfig groundIntakeConfig = new SparkMaxConfig();
    groundIntakeConfig.idleMode(IdleMode.kBrake);
    groundIntakeConfig.smartCurrentLimit(20, 20);
    groundIntakeMotor.configure(groundIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    rawCurrent = groundIntakeMotor.getOutputCurrent();
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
    rotateMotor.setPosition(0.5);
  }

  public void rotateToHandoff() {
    rotateMotor.setPosition(0.2);
  }

  public double getFilteredCurrent() {
    return filteredCurrent;
  }

  public double getRawCurrent() {
    return rawCurrent;
  }
}
