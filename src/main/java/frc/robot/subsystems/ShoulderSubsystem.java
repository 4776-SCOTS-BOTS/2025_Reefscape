// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import frc.robot.customClass.SystemPositions;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** A robot arm subsystem that moves with a motion profile. */
public class ShoulderSubsystem extends SubsystemBase {
  private SparkMax shoulderMotor = new SparkMax(Constants.ShoulderConstants.shoulderMotorCANID, MotorType.kBrushless);
  SparkClosedLoopController controller = shoulderMotor.getClosedLoopController();
  private SparkAbsoluteEncoder shoulderEncoder = shoulderMotor.getAbsoluteEncoder();

  private double minRot = 0.083;
  private double maxRot = 0.95;

  private static double kDt = 0.02;

  private double kS = 0;//0.27
  private double kG = 0.4; // was 0.17 from test arm
  private double kV = 1;//1.5 - 4.77

  private final ArmFeedforward m_feedforward = new ArmFeedforward(kS, kG, kV);

    // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint. Values are rotations / s
  // Max speed is about 0.4 rev/s at 225:1
  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.8, 2));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private double arbFF;

  
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
    
    motorConfig.signals.absoluteEncoderPositionPeriodMs((int)(kDt*1000));

    motorConfig
      .smartCurrentLimit(80, 80)
      .inverted(true)
      .idleMode(IdleMode.kBrake); // Coast will be safer for tuning.  Eventually use brake

    motorConfig.absoluteEncoder
    .zeroOffset(Constants.ShoulderConstants.zeroOffset)
    .inverted(false)
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    motorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    // Set PID values for position control. We don't need to pass a closed
    // loop slot, as it will default to slot 0.
    .p(3)
    .i(0)
    .d(0)
    .outputRange(-1, 1);

    motorConfig
      .voltageCompensation(11);

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
    // // This method will be called once per scheduler run
    if(shoulderMode == ShoulderMode.MANUAL){
      m_setpoint = new TrapezoidProfile.State(getCurrentPosition(), 0);
    }

    // System.out.println("Arm Position" + getCurrentPosition());

    if ((shoulderMode == ShoulderMode.RUN_TO_POSITION) && isValidGoal(m_goal)) {
      // Retrieve the profiled setpoint for the next timestep. This setpoint moves
      // toward the goal while obeying the constraints.
      m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
      arbFF = m_feedforward.calculate(2 * Math.PI * getCurrentPosition() - Math.PI / 2, 2 * Math.PI * getCurrentVelocity());
      
      
      // m_feedforward.calculateWithVelocities(
      //     2 * Math.PI * getCurrentPosition() - Math.PI / 2,
      //     2 * Math.PI * getCurrentVelocity(),
      //     m_setpoint.velocity);

      // Send setpoint to offboard controller PID
      controller.setReference(m_setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF);


    }
    SmartDashboard.putNumber("Shoulder Cur", shoulderMotor.getAppliedOutput());
  }

   public void setArmGoal(double goal) {
     m_goal = new TrapezoidProfile.State(goal, 0);
     shoulderMode = ShoulderMode.RUN_TO_POSITION;
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

  public void runMotor(double speed){
    speed = limitPower(speed);
    shoulderMotor.set(speed);
    shoulderMode = ShoulderMode.MANUAL;
  }

  public double limitPower(double power) {
    double slow = 0.3;
    double min = minRot;
    double minSlow = minRot + 0.05;
    double max = maxRot;
    double maxSlow = maxRot - 0.05;

    double pos = getCurrentPosition();

    // System.out.print("speed = " + power + " pos = " + pos + " | " + min + "/" + minSlow);
    // String output;

    if ((power >= 0) && isInRange(pos, maxSlow, max)) {
      // output = " | Max Slow";
      // System.out.println(output);
      return power * slow;
    } else if ((power >= 0) && (pos >= max)) {
      // output = " | @Max";
      // System.out.println(output);
      return 0;
    } else if ((power < 0) && isInRange(pos, min, minSlow)) {
      // output = " | Min Slow";
      // System.out.println(output);
      return power * slow;
    } else if ((power < 0) && (pos <= min)) {
      // output = " | @Min";
      // System.out.println(output);
      return 0;
    } else {
      // output = " | Normal";
      // System.out.println(output);
      return power;
    }
  }

  public void setStraightBack(){
    setArmGoal(0.75);
  }

  public void setStraightUp(){
    setArmGoal(0.5);
  }

  public void setStraightForward(){
    setArmGoal(0.25);
  }

  public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 4));
    layout.addNumber("Current Position", this::getCurrentPosition).withPosition(0, 0);
    layout.addNumber("Goal Setpoint", () -> {
      return m_goal.position;
    }).withPosition(0, 1);
    layout.addNumber("Current Setpoint", () -> {
      return m_setpoint.position;
    }).withPosition(0, 2);
    layout.addNumber("ArbFF", () -> {
      return arbFF;
    }).withPosition(0, 3);
    // layout.addNumber("Position Meters",
    // this::getElevatorPosition).withPosition(0, 1);
    // layout.addNumber("Target Position Meters", () ->
    // targetPosition).withPosition(0, 2);

    // SmartDashboard.putNumber("arbFF", arbFF);
    // SmartDashboard.putNumber("m_goal.position", m_goal.position);
    // SmartDashboard.putNumber("m_setpoint.position", m_setpoint.position);
    // SmartDashboard.putNumber("Current Pos", getCurrentPosition());
    // SmartDashboard.putNumber("m_setpoint.velocity", m_setpoint.velocity);
  }

}
