// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.customClass.ArmJointConstants;
import frc.robot.customClass.ShoulderFeedFowardController;

/** A robot arm subsystem that moves with a motion profile. */
public class ShoulderSubsystem extends SubsystemBase {
  private SparkMax shoulderMotor = new SparkMax(Constants.ShoulderConstants.shoulderMotorCANID, MotorType.kBrushless);
  SparkClosedLoopController controller = shoulderMotor.getClosedLoopController();
  private SparkAbsoluteEncoder shoulderEncoder = shoulderMotor.getAbsoluteEncoder();

  private double minRot = 0.125;
  private double maxRot = 0.875;

  private static double kDt = 0.02;

  private double kS = 0;
  private double kG = 0;
  private double kV = 0;

  private final ArmFeedforward m_feedforward = new ArmFeedforward(kS, kG, kV, kDt);

    // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(60, 60));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  
  // Need to treat straight down as zero in order to prevent movement through the bottom of rotation
  // However, ArmFeedFoward assumes zero is horizontal.
  // Apply offsetAngleRads to the input to the ArmFeedForward calculate method
  // Also assume clockwise positive rotation looking at the front of the arm
  private double offsetAngleRads = -Math.PI/2;

  public enum ShoulderMode {
    RUN_TO_POSITION,
    MANUAL
  }

  public ShoulderMode shoulderMode = ShoulderMode.MANUAL;

  /** Create a new ArmSubsystem. */
  public ShoulderSubsystem() {

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    
    motorConfig.signals.absoluteEncoderPositionPeriodMs(20);

    motorConfig
      .idleMode(IdleMode.kCoast); // Coast will be safer for tuning.  Eventually use brake

    motorConfig.absoluteEncoder
    .zeroOffset(Constants.ShoulderConstants.zeroOffset)
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    motorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    // Set PID values for position control. We don't need to pass a closed
    // loop slot, as it will default to slot 0.
    .p(1.0)
    .i(0)
    .d(0)
    .outputRange(-0.5, 0.5);

    // motorConfig.closedLoop.maxMotion
    // // Set MAXMotion parameters for position control. We don't need to pass
    // // a closed loop slot, as it will default to slot 0.
    // // Won't use this if we use trap profile
    // .maxVelocity(60)
    // .maxAcceleration(300)
    // .allowedClosedLoopError(0.01)
    // .positionMode(null);

    shoulderMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if ((shoulderMode == ShoulderMode.RUN_TO_POSITION) && isValidGoal(m_goal)) {
      // Retrieve the profiled setpoint for the next timestep. This setpoint moves
      // toward the goal while obeying the constraints.
      m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

      // Send setpoint to offboard controller PID
      controller.setReference(m_setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0,
          m_feedforward.calculateWithVelocities(
              2 * Math.PI * getCurrentPosition(),
              2 * Math.PI * getCurrentVelocity(),
              m_setpoint.velocity));
    }
  }


  public Command setArmGoalCommand(double goal) {
    m_goal = new TrapezoidProfile.State(goal, 0);
    if ((shoulderMode == ShoulderMode.RUN_TO_POSITION) && isValidGoal(m_goal)) {
      return Commands.runOnce(() -> {
        m_goal = new TrapezoidProfile.State(goal, 0);
        shoulderMode = ShoulderMode.RUN_TO_POSITION;
      }, this);
    } else {
      return Commands.none();
    }
  }

  public Command holdArmPositionCommand() {
    return Commands.runOnce(() -> {
      holdArmPosition();
    }, this);
  }

  public void holdArmPosition() {
    double goal = shoulderEncoder.getPosition();
    shoulderMode = ShoulderMode.RUN_TO_POSITION;
    m_goal = new TrapezoidProfile.State(goal, 0);
  }

  public double getOffset() {
      return offsetAngleRads;
  }

  public void setOffsetAngleRads(double offsetAngleRads) {
    this.offsetAngleRads = offsetAngleRads;
  }

  public double getCurrentPosition(){
    //Returns units of rotations
    return shoulderEncoder.getPosition();
  }

  public double getCurrentVelocity(){
    //Returns units of rotations
    return shoulderEncoder.getVelocity();
  }

  public boolean isInRange(double val, double min, double max) {
    return (val >= min) && (val <= max);
  }

  public boolean isValidGoal(TrapezoidProfile.State goalState){
    return isInRange(goalState.position, minRot, maxRot);
  }

}
