package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_PARK_HEIGHT;

import java.util.Map;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
  private static final double ELEVATOR_HEIGHT = 1.002; //TODO: Need to update

  // Motor's encoder limits, in encoder ticks
  private static final double MOTOR_BOTTOM = 0; //TODO: Need to update
  private static final double MOTOR_TOP = 56530; //TODO: Need to update

  // Mutiply by sensor position to get meters
  private static final double MOTOR_ENCODER_POSITION_COEFFICIENT = ELEVATOR_HEIGHT / (MOTOR_TOP - MOTOR_BOTTOM); // m/rot

  // Convert Elevator Speed and Acceleration to rotations
  private static final double MAX_LINEAR_SPEED = 1; // m/s
  private static final double MAX_LINEAR_ACCEL = 2.0; // m / s^2
  private static final double MAX_ROT_SPEED = MOTOR_ENCODER_POSITION_COEFFICIENT / MAX_LINEAR_SPEED; // rot / s
  private static final double MAX_ROT_ACCEL = MOTOR_ENCODER_POSITION_COEFFICIENT / MAX_LINEAR_ACCEL;// rot /s^2


  private static final int ANALOG_BOTTOM = 758;
  private static final int ANALOG_TOP = 1796;

  private static final double GRAVITY_FEED_FORWARD = 0.05; //TODO: Need to update

//   // Mutiply by sensor position to get meters
  private static final double ANALOG_SENSOR_COEFFICIENT = ELEVATOR_HEIGHT / (ANALOG_TOP - ANALOG_BOTTOM);

  private final TalonFX elevatorLeader;
  private final TalonFX elevatorFollower;
  private final AnalogInput analogSensor = new AnalogInput(0); //! this is not correct, change before running

  // Limit switches - FALSE means at limit
  private final DigitalInput topLimitSwitch = new DigitalInput(8); //TODO: Need to update.  Do we use?
  private final DigitalInput bottomLimitSwitch = new DigitalInput(9); //TODO: Need to update.  Do we use?

  private double targetPosition = 0;

  public ElevatorControlSubsystem() {
    elevatorLeader = new TalonFX(ElevatorConstants.ELEVATOR_LEADER_ID, "rio");
    elevatorFollower = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID, "rio");

    // Configure closed-loop control
    double kP = 0.13;
    double kI = 0;
    double kD = 0; 
    double kIz = 0;
    double kF = 0.00;
    double kMaxOutput = 0.7;
    double kMinOutput = -.4;
    double allowedErr = 1;

    // Magic Motion Coefficients
    double maxVel = 10000;
    double maxAcc = 30000;

    // TalonFXConfiguration config = new TalonFXConfiguration();
    // config.MotionMagic.
    // config.slot0.kP = kP;
    // config.slot0.kI = kI;
    // config.slot0.kD = kD;
    // config.slot0.integralZone = kIz;
    // config.slot0.kF = kF;
    // config.slot0.allowableClosedloopError = allowedErr;
    
    // config.motionCruiseVelocity = maxVel;
    // config.motionAcceleration = maxAcc;

    // // Voltage compensation and current limits
    // config.voltageCompSaturation = 12;
    // // config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(); // TODO research this

    // Configure soft limits
    // config.forwardSoftLimitEnable = true;
    // config.forwardSoftLimitThreshold = MOTOR_TOP - metersToMotorPosition(0.005);
    // config.reverseSoftLimitEnable = true;
    // config.reverseSoftLimitThreshold = MOTOR_BOTTOM + metersToMotorPosition(0.02);
    // config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    // config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    // config.neutralDeadband = .02;

    // elevatorLeader.configAllSettings(config);
    // elevatorFollower.configAllSettings(config);

    // elevatorLeader.configPeakOutputForward(kMaxOutput);
    // elevatorLeader.configPeakOutputReverse(kMinOutput);
    // elevatorLeader.enableVoltageCompensation(true);

    // elevatorLeader.setInverted(true);
    // elevatorFollower.setInverted(true);

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure Motion Magic */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(MAX_ROT_SPEED)) // (mechanism) rotations per second cruise
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(MAX_ROT_ACCEL)) // Take approximately 0.5 seconds to reach max vel
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(MAX_ROT_SPEED * 10)); // Take approximately 0.1 seconds to reach max accel 

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kG = 0.5;

    elevatorLeader.getConfigurator().apply(cfg);

    // Mathew start here:
    // elevatorFollower.follow(elevatorLeader); //TODO: Set Follower to follow leader
    elevatorFollower.setControl(new Follower(elevatorLeader.getDeviceID(), false));

    // Brake mode helps hold the elevator in place TODO: Update to v6 code
    elevatorLeader.setNeutralMode(NeutralModeValue.Brake);
    elevatorFollower.setNeutralMode(NeutralModeValue.Brake);

    //TODO: Set inverts correctly. Leader is inverted, follower is not inverted (need to use strict follow mode I think?)


  }

  public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
    layout.addNumber("Position Raw", () -> elevatorLeader.getRotorPosition().getValueAsDouble()).withPosition(0, 0);
    layout.addNumber("Position Meters", this::getElevatorPosition).withPosition(0, 1);
    layout.addNumber("Target Position Meters", () -> targetPosition).withPosition(0, 2);
    var limitsLayout = layout.getLayout("Limits", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 2, "Number of rows", 1)).withPosition(0, 3).withSize(2,1);
    limitsLayout.addBoolean("Top Limit", this::isAtTopLimit).withPosition(0, 0);
    limitsLayout.addBoolean("Botton Limit", this::isAtBottomLimit).withPosition(1, 0);
  }

  // ! this is porbably not correct but it gets the red line to go away
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0); 
  
  @Override
  public void periodic() {
    // Handle elevator limit switches
    if (isAtBottomLimit()) {
      elevatorLeader.setControl(m_request.withPosition(MOTOR_BOTTOM));
    } else if (isAtTopLimit()) {
      elevatorLeader.setControl(m_request.withPosition(MOTOR_TOP));
    }
  }

  /**
   * Moves the elevator using duty cycle
   * @param speed duty cycle [-1, 1]
   */
  public void moveElevator(double speed) {
    targetPosition = 0;
    elevatorLeader.set(speed);
  }

  /**
   * Moves the elevator to a position
   * @param meters position in meters
   */
  public void moveToPosition(double meters) {
    //Mathew need to use the Motion Magic control requests.
    targetPosition = meters;
    elevatorLeader.setControl(m_request.withPosition(MOTOR_TOP));
  }
  
  /**
   * Get the elevator position in meters
   * @return position in meters
   */
  public double getElevatorPosition() {
    return motorPositionToMeters(elevatorLeader.getRotorPosition().getValueAsDouble());
  }

  /**
   * Gets the position of the elevator top / first stage, where the Limelight is mounted.
   * This moves slower than the wrist / second stage.
   */
  public double getElevatorTopPosition() {
    return getElevatorPosition() * .489;
  }
  
  /**
   * Moves the elevator to the park/transit position.
   */
  public void parkElevator() {
    moveToPosition(ELEVATOR_PARK_HEIGHT);
  }

  /**
   * Returns true if the elevator is below park position, within a tolerance.
   * @return true if elevator is parked
   */
  public boolean isParked() {
    return getElevatorPosition() < (ELEVATOR_PARK_HEIGHT + 0.1);
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

  private double getElevatorAnalogRawPosition() {
    return 4096 - analogSensor.getValue(); // Invert sensor so up is positive
  }

  private double getElevatorAnalogPositionMeters() {
    return (getElevatorAnalogRawPosition() - ANALOG_BOTTOM) * ANALOG_SENSOR_COEFFICIENT;
  }

  static double motorPositionToMeters(double motorPosition) {
    return (motorPosition * MOTOR_ENCODER_POSITION_COEFFICIENT);
  }

  static double metersToMotorPosition(double positionMeters) {
    return (positionMeters / MOTOR_ENCODER_POSITION_COEFFICIENT);
  }
  
}