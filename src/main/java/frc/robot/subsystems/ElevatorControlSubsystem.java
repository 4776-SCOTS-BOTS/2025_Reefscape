package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

/**
 * Subsystem for elevator mechanism
 */
public class ElevatorControlSubsystem extends SubsystemBase {

  // Elevator travel distance, in meters
  // 1.75in from bottom of frame to the ground
  // 29.5in from bottom of frame to Shoulder Pivot

  public final Distance ELEVATOR_BASE_HEIGHT = Inches.of(1.75 + 29.5);
  public final Distance ELEVATOR_HEIGHT = Inches.of(1.75 + 71.5);

  // Motor's encoder limits, in encoder ticks
  private static final double MOTOR_BOTTOM = 0; //TODO: Need to update
  private static final double MOTOR_TOP = 186.6; //TODO: Need to update

  // Mutiply by sensor position to get meters
  private final double MOTOR_ENCODER_POSITION_COEFFICIENT = (ELEVATOR_HEIGHT.in(Meters) - ELEVATOR_BASE_HEIGHT.in(Meters))
                                                                    / (MOTOR_TOP - MOTOR_BOTTOM); // m/rot

  // Convert Elevator Speed and Acceleration to rotations
  private final double MAX_LINEAR_SPEED = 0.2; // m/s
  private final double MAX_LINEAR_ACCEL = 1; // m / s^2
  private final double MAX_ROT_SPEED = MAX_LINEAR_SPEED/ MOTOR_ENCODER_POSITION_COEFFICIENT; // rot / s
  private final double MAX_ROT_ACCEL = MAX_LINEAR_ACCEL / MOTOR_ENCODER_POSITION_COEFFICIENT;// rot /s^2


  private final TalonFX elevatorLeader;
  private final TalonFX elevatorFollower;

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0); 

  // Limit switches - FALSE means at limit
  private final DigitalInput topLimitSwitch = new DigitalInput(8); //TODO: Need to update.  Do we use?
  private final DigitalInput bottomLimitSwitch = new DigitalInput(9); //TODO: Need to update.  Do we use?

  private double targetPosition = ELEVATOR_BASE_HEIGHT.in(Meters);

  public ElevatorControlSubsystem() {
    elevatorLeader = new TalonFX(ElevatorConstants.ELEVATOR_LEADER_ID, "rio");
    elevatorFollower = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID, "rio");

    // // Configure closed-loop control
    // double kP = 0.13;
    // double kI = 0;
    // double kD = 0; 
    // double kIz = 0;
    // double kF = 0.00;
    // double kMaxOutput = 0.7;
    // double kMinOutput = -.4;
    // double allowedErr = 1;

    // // Magic Motion Coefficients
    // double maxVel = 10000;
    // double maxAcc = 30000;

    TalonFXConfiguration leader_cfg = new TalonFXConfiguration();
    TalonFXConfiguration follower_cfg = new TalonFXConfiguration();

    /* Configure Motion Magic */
    MotionMagicConfigs mm = leader_cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(MAX_ROT_SPEED)) // (motor) rotations per second cruise
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(MAX_ROT_ACCEL)) // Take approximately 0.5 seconds to reach max vel
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(MAX_ROT_SPEED * 10)); // Take approximately 0.1 seconds to reach max accel 

    Slot0Configs slot0 = leader_cfg.Slot0;
    slot0.kS = 0.0; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 30; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kG = 0.2;

    MotorOutputConfigs leader_mo = leader_cfg.MotorOutput;
    leader_mo.Inverted = InvertedValue.Clockwise_Positive;
    leader_mo.NeutralMode = NeutralModeValue.Brake;

    leader_cfg.CurrentLimits.StatorCurrentLimit = 60; // This will help limit total torque the motor can apply to the mechanism. Could be too low for fast operation
    leader_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    elevatorLeader.getConfigurator().apply(leader_cfg);
    
    //Setup Follower
    MotorOutputConfigs follower_mo = follower_cfg.MotorOutput;
    follower_mo.Inverted = InvertedValue.Clockwise_Positive;
    follower_mo.NeutralMode = NeutralModeValue.Brake;

    follower_cfg.CurrentLimits.StatorCurrentLimit = 60; // This will help limit total torque the motor can apply to the mechanism. Could be too low for fast operation
    follower_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    elevatorFollower.getConfigurator().apply(follower_cfg);
    
    elevatorFollower.setControl(new StrictFollower(elevatorLeader.getDeviceID()));

  }

  public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 4));
    layout.addNumber("Position Raw", () -> elevatorLeader.getRotorPosition().getValueAsDouble()).withPosition(0, 0);
    layout.addNumber("Position Meters", this::getElevatorPosition).withPosition(0, 1);
    layout.addNumber("Target Position Meters", () -> targetPosition).withPosition(0, 2);
    layout.addNumber("MOTOR_ENCODER_POSITION_COEFFICIENT", () -> MOTOR_ENCODER_POSITION_COEFFICIENT).withPosition(0, 3);
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
  }

  /**
   * Moves the elevator using duty cycle
   * @param speed duty cycle [-1, 1]
   */
  public void moveElevator(double speed) {
    targetPosition = 0;
    elevatorLeader.set(speed / 2);
  }

  /**
   * Moves the elevator to a position
   * @param meters position in meters
   */
  public void moveToPosition(double meters) {
    if(meters < ELEVATOR_BASE_HEIGHT.in(Meters)){
      meters = ELEVATOR_BASE_HEIGHT.in(Meters);
    } else if (meters > ELEVATOR_HEIGHT.in(Meters)){
      meters = ELEVATOR_HEIGHT.in(Meters);
    }
    targetPosition = meters;
    elevatorLeader.setControl(m_request.withPosition(metersToMotorPosition(meters)));
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
    moveToPosition(ELEVATOR_BASE_HEIGHT.in(Meters));
  }

  /**
   * Returns true if the elevator is below park position, within a tolerance.
   * @return true if elevator is parked
   */
  public boolean isParked() {
    return getElevatorPosition() < (ELEVATOR_BASE_HEIGHT.in(Meters) + 0.1);
  }

  /**
   * Stop the elevator
   */
  public void stop() {
    elevatorLeader.stopMotor();
  }

  public boolean isAtBottomLimit() {
    return !bottomLimitSwitch.get();
  }

  public boolean isAtTopLimit() {
    return !topLimitSwitch.get();
  }

  public double motorPositionToMeters(double motorPosition) {
    return (motorPosition * MOTOR_ENCODER_POSITION_COEFFICIENT + ELEVATOR_BASE_HEIGHT.in(Meters));
  }

  public double metersToMotorPosition(double positionMeters) {
    return ((positionMeters - ELEVATOR_BASE_HEIGHT.in(Meters)) / MOTOR_ENCODER_POSITION_COEFFICIENT);
  }
  
}