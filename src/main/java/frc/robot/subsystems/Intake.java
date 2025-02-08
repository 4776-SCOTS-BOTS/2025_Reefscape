// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  public SparkMax intakeMotor, wristMotor;
  LinearFilter filter = LinearFilter.movingAverage(4);
  
  public Intake() {

    intakeMotor = new SparkMax(Constants.IntakeConstants.intakeMotorCANID, MotorType.kBrushless);
    wristMotor = new SparkMax(Constants.IntakeConstants.wristMotorCANID, MotorType.kBrushless);

  }

  @Override
  public void periodic(){
    if(filter.calculate(intakeMotor.getOutputCurrent()) > 1){ //! number is temporary
      Timer.delay(0.5);
      intakeOff();
    }
    // This method will be called once per scheduler run
  }


  public void intakeIn(){
    intakeMotor.set(.5);
  }

  public void intakeOut(){
    intakeMotor.set(-.5);
  }
  public void intakeOff(){
    intakeMotor.set(0);
  }
}
