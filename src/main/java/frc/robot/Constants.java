// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.customClass.CRGB;

import edu.wpi.first.units.measure.Distance.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 * 16"x22"
 */
public final class Constants {
  public static enum RobotType {
    CompBot,
    PracticeBot
  }

  public static final RobotType robotType = RobotType.CompBot;

  public static final class ConfigConstants {
    public static boolean fullShuffleBoardOutput = false;
    public static boolean hasCamera = true;
    public static Alliance alliance = Alliance.Blue;
  }

  public static final CRGB kRGB_red = new CRGB(255, 0, 0);
  public static final CRGB kRGB_green = new CRGB(0, 255, 0);
  public static final CRGB kRGB_blue = new CRGB(0, 0, 255);
  public static final CRGB kRGB_yellow = new CRGB(204, 104, 0);
  public static final CRGB kRGB_purple = new CRGB(70, 0, 40);
  public static final CRGB kRGB_boaz = new CRGB(0, 59, 111);
  public static final CRGB kRGB_greenLow = new CRGB(0, 20, 0);
  public static final CRGB kRGB_black = new CRGB(0, 0, 0);

  public static int topButton, rightButton, bottomButton, leftButton, leftBumper, rightBumper, leftTrigger,
      rightTrigger, leftStickButton, rightStickButton, dpadUp, dpadRight, dpadDown, dpadLeft,
      PS5MuteButton, PS5HomeButton, leftMenuButton, rightMenuButton, PS5TouchpadButton, leftStickX,
      leftStickY, rightStickX, rightStickY;

  public static final class IntakeConstants {
    public static final int intakeMotorCANID = 20;
    public static final int wristMotorCANID = 21;
    public static final int groundIntakeRollerCANID = 22;
    public static final int groundIntakeRotateCANID = 23;
  }

  public static final class ShoulderConstants {
    public static final int shoulderMotorCANID = 25;
    // public static final int shoudlderCANcoderID = 26;
    public static final double zeroOffset = 0.177; // TODO: needs update
  }

  public static final class ElevatorConstants {
    public static final Distance ELEVATOR_BASE_HEIGHT = Inches.of(1.75 + 31.0);//Was 29.5
    public static final Distance ELEVATOR_MAX_HEIGHT = Inches.of(1.75 + 72.5);//Was 71.5

    public static final int ELEVATOR_LEADER_ID = 28;
    public static final int ELEVATOR_FOLLOWER_ID = 29;
  }

  public static final class ClimberConstants {
    public static final int climberMotorCANID = 30;
    public static final int tiltMotorCANID = 31;
  }

  public static final class DriveConstants {
    // Any constants that are not final can and should be update in
    // GenerateConstants
    // Non-final constants are initialized with the values of the practice bot
    // below.

    public static final double driveNormalPercentScale = 0.8; // 0.8
    public static final double rotNormalRateModifier = 0.8; // 0.8
    public static final double driveLowPercentScale = 0.45;
    public static final double rotLowRateModifier = 0.6;

    public static final double judahDriveNormalPercentScale = 0.56;
    public static final double judahRotNormalRateModifier = 0.56;
    public static final double judahDriveLowPercentScale = 0.4;
    public static final double judahRotLowRateModifier = 0.42;

    public static final double childDriveNormalPercentScale = 0.35;
    public static final double childRotNormalRateModifier = 0.35;
    public static final double childDriveLowPercentScale = 0.25;
    public static final double childRotLowRateModifier = 0.27;

    public static double drivePercentScale = driveNormalPercentScale;
    public static double rotRateModifier = rotNormalRateModifier;

  }

  public static final class Controllers {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
    // public static final int kLogitechControllerPort = 2;
  }

  public static void controllerConstants() {
    boolean usePS5 = true;
    if (usePS5) {
      // PS5 Controller
      topButton = 4;
      rightButton = 3;
      bottomButton = 2;
      leftButton = 1;
      leftBumper = 5;
      rightBumper = 6;
      leftTrigger = 3; // Axis, called X Rotate in DriverStation
      rightTrigger = 4; // Axis, called Y Rotate in DriverStation
      leftStickButton = 11;
      rightStickButton = 12;
      dpadUp = 0;
      dpadRight = 90;
      dpadDown = 180;
      dpadLeft = 270;
      PS5MuteButton = 15;
      PS5HomeButton = 13;
      leftMenuButton = 9;
      rightMenuButton = 10;
      PS5TouchpadButton = 14;
      leftStickX = 0; // Axis, called X Axis in DriverStation
      leftStickY = 1; // Axis, called Y Axis in DriverStation
      rightStickX = 2; // Axis, called Z Axis in DriverStation
      rightStickY = 5; // Axis, called Z Rotate in DriverStation
    } else {
      // Controllers.kManipulatorControllerPort = Controllers.kLogitechControllerPort;
      // Xbox Controller
      topButton = 4;
      rightButton = 2;
      bottomButton = 1;
      leftButton = 3;
      leftBumper = 5;
      rightBumper = 6;
      leftTrigger = 2; // Axis, 0 or 1
      rightTrigger = 3; // Axis, 0 or 1
      leftStickButton = 9;
      rightStickButton = 10;
      dpadUp = 0;
      dpadRight = 90;
      dpadDown = 180;
      dpadLeft = 270;
      // PS5MuteButton = null;
      // PS5HomeButton = null;
      leftMenuButton = 7;
      rightMenuButton = 8;
      // PS5TouchpadButton = null;

      leftStickX = 0; // Axis
      leftStickY = 1; // Axis
      rightStickX = 4; // Axis
      rightStickY = 5; // Axis
    }
  }

public static final class AutoConstants {
  public static final double kMaxSpeedMetersPerSecond = 4.0;// was 3
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;// was 3
  public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;// was Pi
  public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  public static final double kPickupSpeed = 1.5;// was 3

  public static final double kPXController = 1;
  public static final double kPYController = 1;
  public static final double kPThetaController = 1;

  // Constraint for the motion profilied robot angle controller
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
}

 
  
  public static final class LEDConstants {
    // LED Port
    public static final int kLEDPort = 0;
    public static final int kStrandLength = 24;
  }

  public static RobotType GenerateConstants(RobotType robot) {
    switch (robot) {

      case CompBot: {
        ConfigConstants.hasCamera = true;

        
      }
        break;

      case PracticeBot: {
        ConfigConstants.hasCamera = false;
      }
        break;
    }
    return robot;
  }

}
