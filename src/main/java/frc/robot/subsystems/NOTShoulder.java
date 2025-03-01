// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NOTShoulder extends SubsystemBase {
  double manualSpeedLimit = 0.5;

  boolean useMotionMagic = false;
  TalonFX shoulderMotor = new TalonFX(Constants.ShoulderConstants.shoulderMotorCANID, "rio");
  CANcoder cancoder; //= new CANcoder(Constants.ShoulderConstants.shoudlderCANcoderID, "rio");
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  /** Creates a new Sholder. */
  public NOTShoulder() {

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    if (useMotionMagic) {
      /*
       * Configure CANcoder to zero the magnet appropriately
       * All positions are in rotations --> limited to less than 1 total rotation
       * Horizontal pointing right is defined as 0 position looking into the arm
       * Horizontal pointing left is -0.5
       * Initially, assume limits at 45° pointing down on both sides
       * 
       */
      CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
      cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.25; // Defines sensor range as -0.75 to +0.25 and cannot
                                                                   // pass through straight down.
      cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // Might need to flip ths with the
                                                                                     // encoder on the backside.
      cc_cfg.MagnetSensor.withMagnetOffset(Rotations.of(Constants.ShoulderConstants.zeroOffset));
      cancoder.getConfigurator().apply(cc_cfg);

      FeedbackConfigs fdb = talonFXConfigs.Feedback;
      fdb.FeedbackRemoteSensorID = cancoder.getDeviceID();
      fdb.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      fdb.SensorToMechanismRatio = 1; // Cancoder directly measures the arm
      fdb.RotorToSensorRatio = 68.75; // Chain sprockets: 16:44 = 2.75 + 25:1 gearbox

      Slot0Configs slot0 = talonFXConfigs.Slot0;
      slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
      slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
      slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
      slot0.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
      slot0.kI = 0; // no output for integrated error
      slot0.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
      slot0.GravityType = GravityTypeValue.Arm_Cosine;
      slot0.kG = 0.5; //

      // set Motion Magic settings
      MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = 0.5; // Target cruise velocity of 0.5 rps
      motionMagicConfigs.MotionMagicAcceleration = 1; // Target acceleration of 1 rps/s (0.5 seconds)
      motionMagicConfigs.MotionMagicJerk = 10; // Target jerk of 10 rps/s/s (0.1 seconds)
    }

    // Other motor settings -- Lots of limits here. May need to tune if behaivng weirdly
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigs.MotorOutput.PeakForwardDutyCycle = manualSpeedLimit;
    talonFXConfigs.MotorOutput.PeakReverseDutyCycle = -manualSpeedLimit;
    talonFXConfigs.Voltage.PeakForwardVoltage = 12 * manualSpeedLimit;
    talonFXConfigs.Voltage.PeakReverseVoltage = -12 * manualSpeedLimit;

    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80; // This will help limit total torque the motor can apply to the mechanism
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    shoulderMotor.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveSholderTo(double position) {
    shoulderMotor.setControl(m_request.withPosition(position));
  }

  public void moveSholder(double speed) {
    shoulderMotor.set(speed);
  }
}
