// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    Constants.controllerConstants();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_robotContainer.drivetrain.standyLimelightUpdate("limelight-front");
    m_robotContainer.drivetrain.standyLimelightUpdate("limelight-rear");

  }

  @Override
  public void disabledExit() {}

  public void robotInit() {
    Pathfinding.setPathfinder(new LocalADStar());
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // Orchestra musicMotors = new Orchestra();
    // musicMotors.addInstrument(m_robotContainer.drivetrain.getModule(0).getDriveMotor());
    // musicMotors.addInstrument(m_robotContainer.drivetrain.getModule(0).getSteerMotor());
    // musicMotors.addInstrument(m_robotContainer.drivetrain.getModule(1).getDriveMotor());
    // musicMotors.addInstrument(m_robotContainer.drivetrain.getModule(1).getSteerMotor());
    // musicMotors.addInstrument(m_robotContainer.drivetrain.getModule(2).getDriveMotor());
    // musicMotors.addInstrument(m_robotContainer.drivetrain.getModule(2).getSteerMotor());
    // musicMotors.addInstrument(m_robotContainer.drivetrain.getModule(3).getDriveMotor());
    // musicMotors.addInstrument(m_robotContainer.drivetrain.getModule(3).getSteerMotor());
    // musicMotors.loadMusic("/home/lvuser/deploy/output.chrp"); // I think that's it
    // musicMotors.play();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Switch Limelight Megatag2 mode to internal IMU
    // m_robotContainer.drivetrain.activeLimelightUpdate("limelight-front");
    // m_robotContainer.drivetrain.activeLimelightUpdate("limelight-rear");
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
