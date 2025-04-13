// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  public enum coralPosition{
    empty,
    beltSide,
    notBeltSide,
    center
  }

  public SparkMax intakeMotor, wristMotor;
  LinearFilter filter = LinearFilter.movingAverage(6);
  private SparkMaxConfig wristMotorConfig;
  private SparkClosedLoopController wristControl;
  private RelativeEncoder encoder;

  public boolean hasCoral = false;
  public double rawCurrent;
  public double filteredCurrent;

  public double pickupPos = 0;
  public double deliverPos1 = 0.205;
  public double deliverPos2 = -0.205;

  public enum WRIST_POSTION{
    PICKUP,
    DELIVER1,
    DELIVER2
  }

  public WRIST_POSTION wristPos = WRIST_POSTION.PICKUP;

  
  public Intake() {

    intakeMotor = new SparkMax(Constants.IntakeConstants.intakeMotorCANID, MotorType.kBrushless);
    wristMotor = new SparkMax(Constants.IntakeConstants.wristMotorCANID, MotorType.kBrushless);

    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    wristControl = wristMotor.getClosedLoopController();
    encoder = wristMotor.getEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    wristMotorConfig = new SparkMaxConfig();

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    wristMotorConfig.encoder
        .positionConversionFactor(1.0/125)
        .velocityConversionFactor(1.0/125);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    wristMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(1.0)
        .i(0)
        .d(0)
        .outputRange(-0.5, 0.5);

    wristMotorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(60)
        .maxAcceleration(300)
        .allowedClosedLoopError(0.01);

    wristMotorConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.smartCurrentLimit(20, 20);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  @Override
  public void periodic(){
    rawCurrent = intakeMotor.getOutputCurrent();
    filteredCurrent = filter.calculate(rawCurrent);

    intakeMotor.getForwardLimitSwitch();
    
    // if(filter.calculate(intakeMotor.getOutputCurrent()) > 5){ //! number is temporary
    //   Timer.delay(0.5);
    //   intakeOff();
    // }
    // This method will be called once per scheduler run

   
  }


  public void intakeIn(){
    intakeMotor.set(.4);
  }

  public void intakeAlgae(){
    intakeMotor.set(.8);
  }

  public void intakeOut(){
    intakeMotor.set(-.25);
  }

  public void intakeOutFast(){
    intakeMotor.set(-.5);
  }
  public void intakeOff(){
    intakeMotor.set(0);
  }

  public void wristPickup(){
    wristControl.setReference(pickupPos, ControlType.kMAXMotionPositionControl);
    wristPos = WRIST_POSTION.PICKUP;
  }

  public void wristDeliver1(){
    wristControl.setReference(deliverPos1, ControlType.kMAXMotionPositionControl);
    wristPos = WRIST_POSTION.DELIVER1;
  }

  public void wristDeliver2(){
    wristControl.setReference(deliverPos2, ControlType.kMAXMotionPositionControl);
    wristPos = WRIST_POSTION.DELIVER2;
  }

  public double getFilteredCurrent(){
    return filteredCurrent;
  }

  public double getRawCurrent(){
    return intakeMotor.getOutputCurrent();
  }

  public void toggleWrist(){
    if(wristPos == WRIST_POSTION.PICKUP){
      wristDeliver1();
    } else {
      wristPickup();
    }
  }

  public void runWrist(double speed){
    wristMotor.set(speed);
  }

    public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
    layout.addNumber("Filtered Current", this::getFilteredCurrent).withPosition(0, 0);
    layout.addNumber("Raw Current", this::getRawCurrent).withPosition(0, 2);
    // layout.addNumber("Position Meters", this::getElevatorPosition).withPosition(0, 1);
    // layout.addNumber("Target Position Meters", () -> targetPosition).withPosition(0, 2);
  }

  public void setWrist(double pos){
    wristMotor.getEncoder().setPosition(pos);
  }

  public void updateWrist(double targetPos) {
    if (targetPos < 0) {
      wristMotor.set(-0.1);
    } else {
      wristMotor.set(0.1);
    }
  }

  public void stopWrist() {
    wristMotor.set(0);
  }

  public void adjustWristPos() {
    wristMotor.set(0.08);
  }

  public void adjustWristNeg() {
    wristMotor.set(-0.08);
  }

  public double getWristPosValue(WRIST_POSTION pos){
    switch(pos){
      case PICKUP:
        return pickupPos;
      case DELIVER1:
        return deliverPos1;
      case DELIVER2:
        return deliverPos2;
      default:
        return 0;
    }
  }

  public void setWristbyCurrentSetpoint(){
    setWrist(getWristPosValue(wristPos));
  }

  public coralPosition getCoralPosition()
  {
    if (intakeMotor.getForwardLimitSwitch().isPressed())
    {
      if (intakeMotor.getReverseLimitSwitch().isPressed())
      {
        return coralPosition.center;
      }
      return coralPosition.beltSide;
    } else if (intakeMotor.getReverseLimitSwitch().isPressed())
    {
      return coralPosition.notBeltSide;
    }
    return coralPosition.empty;
  }

}
