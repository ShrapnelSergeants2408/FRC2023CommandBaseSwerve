// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    //CAN Address - drive motors - CAN port 20s
    public static final int kFrontLeftDriveMotorPort = 20;
    public static final int kRearLeftDriveMotorPort = 21;
    public static final int kFrontRightDriveMotorPort = 22;
    public static final int kRearRightDriveMotorPort = 23;

    //CAN Address - steer motors - CAN port 30s
    public static final int kFrontLeftTurningMotorPort = 30;
    public static final int kRearLeftTurningMotorPort = 31;
    public static final int kFrontRightTurningMotorPort = 32;
    public static final int kRearRightTurningMotorPort = 33;

    /*
    public static final int[] kFrontLeftTurningEncoderPorts = new int[] {0, 1};
    public static final int[] kRearLeftTurningEncoderPorts = new int[] {2, 3};
    public static final int[] kFrontRightTurningEncoderPorts = new int[] {4, 5};
    public static final int[] kRearRightTurningEncoderPorts = new int[] {6, 7};
    */

    //MA3 encoders for steering (Analog 0 - 3)
    public static final int kFrontLeftTurningEncoderPorts = 0;
    public static final int kRearLeftTurningEncoderPorts = 1;
    public static final int kFrontRightTurningEncoderPorts = 2;
    public static final int kRearRightTurningEncoderPorts = 3;

    //TODO:  verify if any of these values need to be changed
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = true;

    //relative encoders from NEO motor and SparkMAX are used.  Port values are
    //placeholders for example code
    public static final int kFrontLeftDriveEncoderPorts = 90;
    public static final int kRearLeftDriveEncoderPorts = 91;
    public static final int kFrontRightDriveEncoderPorts = 92;
    public static final int kRearRightDriveEncoderPorts = 93;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = true;

    //Set speed multiplier from 0-1 (0%-100%).  Set to 0 for testing opposite motor (ex 0 drive to test turn)
    public static final int kDriveSpeedMultiplier = 1;
    public static final int kTurnSpeedMultiplier = 1;

    // Distance between centers of right and left wheels on robot
    // 18.375 in = 0.467 m
    public static final double kTrackWidth = 0.467;

    // Distance between front and back wheels on robot
    // 20 in = 0.508 m
    public static final double kWheelBase = 0.508;

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // TODO: These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;
  }

  public static final class ModuleConstants {


    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kTurningEncoderCPR = 1024;

    public static final int kNEOEncoderCPR = 42;
    public static final double kDriveGearRatio = 6.67;
    public static final double kDriveEncoderCPR = kNEOEncoderCPR * kDriveGearRatio;

    public static final double kWheelDiameterInches = 4.0;
    public static final double kInchesToMeters = 0.0254;
    public static final double kWheelDiameterMeters = kWheelDiameterInches * kInchesToMeters;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        // NEO 42 cpr * 6.67 gear ratio
        (kWheelDiameterMeters * Math.PI) / (double) kDriveEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.

        (2 * Math.PI) / (double) kTurningEncoderCPR;

    // Gear ratio 48 motor gear:40 encoder gear
    public static final double kTurningEncoderDistancePerRotation =
        //(40/48)*(2 * Math.PI);
        (48/40)*(2* Math.PI);

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    
    //slew rate 3 --> 1/3 sec from 0 to 1
    public static final int kD_Joystick_Left_X_Speed_Limit = 3;
    public static final int kD_Joystick_Left_Y_Speed_Limit = 3;
    public static final int kD_Joystick_Right_X_Speed_Limit = 3;
    public static final int kD_Joystick_Right_Y_Speed_Limit = 3;
    
    public static final int kO_Joystick_Left_X_Speed_Limit = 3;
    public static final int kO_Joystick_Left_Y_Speed_Limit = 3;
    public static final int kO_Joystick_Right_X_Speed_Limit = 3;
    public static final int kO_Joystick_Right_Y_Speed_Limit = 3;

	//Joystick button/axis maps
  public static final int kD_Joystick_Left_X_Axis = 0;
  public static final int kD_Joystick_Left_Y_Axis = 1;
  public static final int kD_Joystick_Right_X_Axis = 2;
  public static final int kD_Joystick_Right_Y_Axis = 3;

  public static final int kD_Left = 1;
  public static final int kD_Down = 2;
  public static final int kD_Right = 3;
  public static final int kD_Up = 4;
  public static final int kD_Shoulder_Top_Left = 5;
  public static final int kD_Shoulder_Top_Right = 6;
  public static final int kD_Shoulder_Bottom_Left = 7;
  public static final int kD_Shoulder_Bottom_Right = 8;
  //public static final int kD_Mid_Left = 9;
  /* TODO:
  I believe there is a mid left and mid right button at 9 and maybe 10
   * As a result, the left and right joystick button values may need to change
   * 
   * If so, adjust operator values as well and add bindings to RobotContainer
   */
  public static final int kD_Left_Joystick = 10;
  public static final int kD_Right_Joystick = 11;

  public static final int kO_Joystick_Left_X_Axis = 0;
  public static final int kO_Joystick_Left_Y_Axis = 1;
  public static final int kO_Joystick_Right_X_Axis = 2;
  public static final int kO_Joystick_Right_Y_Axis = 3;

  public static final int kO_Left = 1;
  public static final int kO_Down = 2;
  public static final int kO_Right = 3;
  public static final int kO_Up = 4;
  public static final int kO_Shoulder_Top_Left = 5;
  public static final int kO_Shoulder_Top_Right = 6;
  public static final int kO_Shoulder_Bottom_Left = 7;
  public static final int kO_Shoulder_Bottom_Right = 8;
  public static final int kO_Left_Joystick = 10;
  public static final int kO_Right_Joystick = 11;

  }

  public static final class AutoConstants {
    //TODO: tweek max speed/acceleration
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ArmConstants {  
    //CAN Address - Arm Motors 40s
    public static final int kArmLiftMotor = 40;
    public static final int kArmExtensionMotor = 41;

    public static final boolean kArmLiftMotorInverted = false;
    public static final boolean kArmExtensionMotorInverted = false;

    //CAN Address - Gripper Motor 50s
    public static final int kGripperExtension = 50;

    public static final boolean kGripperExtensionInverted = false;


    //PCM channels
    public static final int kGripperOpen = 0;
    public static final int kGripperClosed = 1;

    //Gripper accelerometer port
    public static final int kGripperPosition = 0;

    //Address Ultrasonic PWM or DIO? ports 0, 1
    public static final int kUltrasonicPingChannel = 0;
    public static final int kUltrasonicEchoChannel = 1;

  }
}
