// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveRobot extends Command {
    private static final double KpX = 0.1;
    private static final double KiX = 0.01;
    private static final double KdX = 0.1;

    private static final double KpY = 0.1;
    private static final double KiY = 0.01;
    private static final double KdY = 0.1;

    private static final double KpRot = 0.1;
    private static final double KiRot = 0.01;
    private static final double KdRot = 0.1;

    private PIDController pidControllerX = new PIDController(KpX, KiX, KdX);
    private PIDController pidControllerY = new PIDController(KpY, KiY, KdY);
    private PIDController pidControllerRot = new PIDController(KpRot, KiRot, KdRot);

    private double setpointX = 0.0;
    private double setpointY = 0.0;
    private double setpointRot = 0.0;

    private double targetX;
    private double targetY;
    private double targetRot;

    double outputX;
    double outputRot;
    double outputY;

    private CommandSwerveDrivetrain drivetrain;

  /** Creates a new MoveRobot. */
  public MoveRobot(CommandSwerveDrivetrain drivetrain, double targetX, double targetY, double targetRot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.targetX = targetX;
    this.targetY = targetY;
    this.targetRot = targetRot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpointX = targetX;
    setpointY = targetY;
    setpointRot = targetRot;
    
    pidControllerX.setSetpoint(setpointX);
    pidControllerY.setSetpoint(setpointY);
    pidControllerRot.setSetpoint(setpointRot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPositionX = drivetrain.getState().Pose.getX();
    double currentPositionY = drivetrain.getState().Pose.getY();
    double currentRotation  = drivetrain.getState().Pose.getRotation().getRadians();

    // Calculate the PID outputs
    double outputX = pidControllerX.calculate(currentPositionX);
    double outputY = pidControllerY.calculate(currentPositionY);
    double outputRot = pidControllerRot.calculate(currentRotation);

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // move the robot to the target position
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
