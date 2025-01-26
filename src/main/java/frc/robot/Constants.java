// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.jsontype.impl.StdSubtypeResolver;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.customClass.CRGB;

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
  
  public static final class IntakeConstants {
    public static final int intakeMotorCANID = 20;
    public static final int wristMotorCANID = 21;
  }

  public static final class ElevatorConstants {
    public static final double ELEVATOR_PARK_HEIGHT = 0.0;

    public static final int ELEVATOR_LEADER_ID = 28;
    public static final int ELEVATOR_FOLLOWER_ID = 29;
  }

  
  
  public static final class ShooterConstants {
    //Rename or remove. Keeping for reference for now.
    public static final int motor1CANID = 58;
    public static final int motor2CANID = 59;
    public static final int feederCANID = 57;
    public static final int leftAngleCAN = 55;
    public static final int rightAngleCAN = 56;
    public static final int cancoder = 50;

    public static final double cancoderOffset = +0.039;


    public static final double kP = 0.0003; // 0.0005
    public static final double kI = 0; //1
    public static final double kD = 0.00000;
    public static final double kIz = 0;
    public static final double kFF = 0.0002;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -0.5;
    public static final double kmaxRPM = 5500;
    public static final double kTypRPM = 4750;

  }




  public static final class DriveConstants {
    // Any constants that are not final can and should be update in
    // GenerateConstants
    // Non-final constants are initialized with the values of the practice bot
    // below.

    public static final double driveNormalPercentScale = 0.8;
    public static final double rotNormalRateModifier = 0.8;
    public static final double driveLowPercentScale = 0.45;
    public static final double rotLowRateModifier = 0.75;

    public static double drivePercentScale = driveNormalPercentScale;
    public static double rotRateModifier = rotNormalRateModifier;

  }

  public static final class Controllers {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
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
