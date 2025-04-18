package frc.robot.subsystems.Backups;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder.TriFunction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.customClass.SystemPositions.Positions;

/**
 * Subsystem for elevator mechanism
 */
public class ElevatorControlSubsystemOld extends SubsystemBase {

  // Elevator travel distance, in meters
  // 1.75in from bottom of frame to the ground
  // 29.5in from bottom of frame to Shoulder Pivot

  // Motor's encoder limits, in encoder ticks
  private static final double MOTOR_BOTTOM = 0;
  private static final double MOTOR_TOP = 186.6;

  // Mutiply by sensor position to get meters
  private final double MOTOR_ENCODER_POSITION_COEFFICIENT = (Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT.in(Meters) - Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters))
                                                                    / (MOTOR_TOP - MOTOR_BOTTOM); // m/rot

  // Convert Elevator Speed and Acceleration to rotations
  private final double MAX_LINEAR_SPEED = 1.5; // m/s
  private final double MAX_LINEAR_ACCEL = 2.0; // m / s^2
  private final double MAX_ROT_SPEED = MAX_LINEAR_SPEED/ MOTOR_ENCODER_POSITION_COEFFICIENT; // rot / s
  private final double MAX_ROT_ACCEL = MAX_LINEAR_ACCEL / MOTOR_ENCODER_POSITION_COEFFICIENT;// rot /s^2
  private final double MAX_ROT_JERK = MAX_ROT_ACCEL * 10;

  private final double SLOW_LINEAR_SPEED = 0.75; // m/s
  private final double SLOW_LINEAR_ACCEL = 1.1; // m / s^2
  private final double SLOW_ROT_SPEED = SLOW_LINEAR_SPEED/ MOTOR_ENCODER_POSITION_COEFFICIENT; // rot / s
  private final double SLOW_ROT_ACCEL = SLOW_LINEAR_ACCEL / MOTOR_ENCODER_POSITION_COEFFICIENT;// rot /s^2
  private final double SLOW_ROT_JERK = SLOW_ROT_ACCEL * 5;

  public enum ElevatorMode {
    RUN_TO_POSITION,
    MANUAL
  }

  public ElevatorMode elevatorMode = ElevatorMode.MANUAL;

  public boolean useLeader = false;
  public boolean useDynamic = true;


  private final TalonFX elevatorLeader;
  private final TalonFX elevatorFollower;

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  private DynamicMotionMagicVoltage m_dynamicRequest;
  
  // Limit switches - FALSE means at limit
  // private final DigitalInput bottomLimitSwitch = new DigitalInput(9); //TODO: Need to update.  Do we use?
  // private final Trigger bottomLimitSwitchTrigger = new Trigger(() -> bottomLimitSwitch.get());

  private double targetPosition = Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters);

  public ElevatorControlSubsystemOld() {
    elevatorLeader = new TalonFX(ElevatorConstants.ELEVATOR_LEADER_ID, "TestBed");
    elevatorFollower = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID, "TestBed");

    TalonFXConfiguration leader_cfg = new TalonFXConfiguration();
    TalonFXConfiguration follower_cfg = new TalonFXConfiguration();

    /* Configure Motion Magic */
    if (!useDynamic) {
      MotionMagicConfigs mm = leader_cfg.MotionMagic;
      mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(MAX_ROT_SPEED)) // (motor) rotations per second cruise
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(MAX_ROT_ACCEL)) // Take approximately 0.5 seconds
                                                                                      // to reach max vel
          .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(MAX_ROT_JERK)); // Take approximately
                                                                                                // 0.1 seconds to reach
                                                                                                // max accel
    } else {
      m_dynamicRequest = new DynamicMotionMagicVoltage(0, MAX_ROT_SPEED, MAX_ROT_ACCEL, MAX_ROT_JERK);
    }

    Slot0Configs slot0 = leader_cfg.Slot0;
    slot0.kS = 0.05; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.05; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.1; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 10; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kG = 0.2;

    MotorOutputConfigs leader_mo = leader_cfg.MotorOutput;
    leader_mo.Inverted = InvertedValue.Clockwise_Positive;
    leader_mo.NeutralMode = NeutralModeValue.Brake;

    leader_cfg.CurrentLimits.StatorCurrentLimit = 70; // This will help limit total torque the motor can apply to the mechanism. Could be too low for fast operation
    leader_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    elevatorLeader.getConfigurator().apply(leader_cfg);
    
    //Setup Follower
    MotorOutputConfigs follower_mo = follower_cfg.MotorOutput;
    follower_mo.Inverted = InvertedValue.Clockwise_Positive;
    follower_mo.NeutralMode = NeutralModeValue.Brake;

    follower_cfg.CurrentLimits.StatorCurrentLimit = 70; // This will help limit total torque the motor can apply to the mechanism. Could be too low for fast operation
    follower_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    if(!useLeader){
      follower_cfg.MotionMagic = leader_cfg.MotionMagic;
      follower_cfg.Slot0 = leader_cfg.Slot0;
    }

    elevatorFollower.getConfigurator().apply(follower_cfg);
    
    if (useLeader) {
      // StricFollower seemed to have issues.
      // elevatorFollower.setControl(new StrictFollower(elevatorLeader.getDeviceID()));
      
      elevatorFollower.setControl(new Follower(elevatorLeader.getDeviceID(), false));
    }

  }

  public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
    layout.addNumber("Position Raw", () -> elevatorLeader.getRotorPosition().getValueAsDouble()).withPosition(0, 0);
    layout.addNumber("Position Meters", this::getElevatorPosition).withPosition(0, 1);
    layout.addNumber("Target Position Meters", () -> targetPosition).withPosition(0, 2);
    // var limitsLayout = layout.getLayout("Limits", BuiltInLayouts.kGrid)
    //     .withProperties(Map.of("Number of columns", 2, "Number of rows", 1)).withPosition(0, 3).withSize(2,1);
    // limitsLayout.addBoolean("Top Limit", this::isAtTopLimit).withPosition(0, 0);
    // limitsLayout.addBoolean("Botton Limit", this::isAtBottomLimit).withPosition(1, 0);
  }
  
  @Override
  public void periodic() {
    // Handle elevator limit switches
    // if (isAtBottomLimit()) {
    //   elevatorLeader.setControl(m_request.withPosition(MOTOR_BOTTOM));
    // } else if (isAtTopLimit()) {
    //   elevatorLeader.setControl(m_request.withPosition(MOTOR_TOP));
    // }
    // bottomLimitSwitchTrigger.onTrue(new InstantCommand(() -> {resetPosition();}));
  }

  /**
   * Moves the elevator using duty cycle
   * 
   * @param speed duty cycle [-1, 1]
   */
  public void moveElevator(double speed) {

    speed = limitPower(speed);

    elevatorLeader.set(speed / 2);
    if (!useLeader) {
      elevatorFollower.set(speed / 2);
    }
    elevatorMode = ElevatorMode.MANUAL;
    targetPosition = getElevatorPosition();
  }

  /**
   * Moves the elevator to a position
   * 
   * @param meters position in meters
   */
  public void moveToPosition(double meters) {
    if (meters < Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters)) {
      meters = Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters);
    } else if (meters > Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT.in(Meters)) {
      meters = Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT.in(Meters);
    }

    targetPosition = meters;

    if (useDynamic) {
      if (targetPosition == Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters)) {
        goSlow();
      } else if (Math.abs((targetPosition - Positions.L4_READY.elevatorHeight)/Positions.L4_READY.elevatorHeight) < 0.1) {
        goFast();
      } else {
        goSlow();
      }

      elevatorLeader.setControl(m_dynamicRequest.withPosition(metersToMotorPosition(meters)));
      if (!useLeader) {
        elevatorFollower.setControl(m_dynamicRequest.withPosition(metersToMotorPosition(meters)));
      }
    } else {
      elevatorLeader.setControl(m_request.withPosition(metersToMotorPosition(meters)));
      if (!useLeader) {
        elevatorFollower.setControl(m_request.withPosition(metersToMotorPosition(meters)));
      }
    }

    elevatorMode = ElevatorMode.RUN_TO_POSITION;
    
  }
  
  /**
   * Get the elevator position in meters
   * @return position in meters
   */
  public double getElevatorPosition() {
    return motorPositionToMeters(elevatorLeader.getRotorPosition().getValueAsDouble());
  }
  
  /**
   * Moves the elevator to the park/transit position.
   */
  public void parkElevator() {
    moveToPosition(Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters));
  }

  /**
   * Returns true if the elevator is below park position, within a tolerance.
   * @return true if elevator is parked
   */
  public boolean isParked() {
    return getElevatorPosition() < (Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters) + 0.1);
  }

  /**
   * Stop the elevator
   */
  public void stop() {
    elevatorLeader.stopMotor();
    if(!useLeader){
      elevatorFollower.stopMotor();
    }
  }

  // public boolean isAtBottomLimit() {
  //   return !bottomLimitSwitch.get();
  // }

  public ElevatorMode getMode(){
    return elevatorMode;
  }

  public double motorPositionToMeters(double motorPosition) {
    return (motorPosition * MOTOR_ENCODER_POSITION_COEFFICIENT + Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters));
  }

  public double metersToMotorPosition(double positionMeters) {
    return ((positionMeters - Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters)) / MOTOR_ENCODER_POSITION_COEFFICIENT);
  }

  public double limitPower(double power) {
    double slow = 0.3;
    double min = Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters);
    double minSlow = Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters) + 0.1;
    double max = Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT.in(Meters);
    double maxSlow = Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT.in(Meters) - 0.1;

    double pos = getElevatorPosition();

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

  public boolean isInRange(double val, double min, double max) {
    return (val >= min) && (val <= max);
  }

  public void resetPosition(){
    elevatorLeader.setPosition(0);
    elevatorFollower.setPosition(0);
  }

  private void goFast(){
    m_dynamicRequest.Velocity = MAX_ROT_SPEED;
    m_dynamicRequest.Acceleration = MAX_ROT_ACCEL;
    m_dynamicRequest.Jerk = MAX_ROT_JERK;
  }

  private void goSlow(){
    m_dynamicRequest.Velocity = SLOW_ROT_SPEED;
    m_dynamicRequest.Acceleration = SLOW_ROT_ACCEL;
    m_dynamicRequest.Jerk = SLOW_ROT_JERK;
  }
  
}