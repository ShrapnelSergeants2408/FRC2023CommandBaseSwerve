// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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

    public static final boolean kFrontLeftDriveMotorReversed = true;
    public static final boolean kRearLeftDriveMotorReversed =  true;
    public static final boolean kFrontRightDriveMotorReversed = false;
    public static final boolean kRearRightDriveMotorReversed = false;

    //CAN Address - turning motors - CAN port 30s
    public static final int kFrontLeftTurningMotorPort = 30;
    public static final int kRearLeftTurningMotorPort = 31;
    public static final int kFrontRightTurningMotorPort = 32;
    public static final int kRearRightTurningMotorPort = 33;
    
    public static final boolean kFrontLeftTurningMotorReversed = false;
    public static final boolean kRearLeftTurningMotorReversed = false;
    public static final boolean kFrontRightTurningMotorReversed = false;
    public static final boolean kRearRightTurningMotorReversed = false;

    //MA3 encoders for steering (Analog 0 - 3) //TODO: double check ports
    public static final int kFrontLeftAbsoluteEncoderPort = 0;
    public static final int kRearLeftAbsoluteEncoderPort = 1;
    public static final int kFrontRightAbsoluteEncoderPort = 2;
    public static final int kRearRightAbsoluteEncoderPort = 3;

    //PG71 hall effect encoders for steering (DIO 0-7)
    public static final int kFrontLeftTurningEncoderPortA = 1;
    public static final int kFrontLeftTurningEncoderPortB = 0;
    public static final int kRearLeftTurningEncoderPortA = 3;
    public static final int kRearLeftTurningEncoderPortB = 2;
    public static final int kFrontRightTurningEncoderPortA = 5;
    public static final int kFrontRightTurningEncoderPortB = 4;
    public static final int kRearRightTurningEncoderPortA = 7;
    public static final int kRearRightTurningEncoderPortB = 6;
    



    //TODO:  verify if any of these values need to be changed
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;


    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    //TODO: find offset values
    public static final double kFrontLeftAbsoluteEncoderOffset = (3.48/5);//* 2 * Math.PI; // old raw value 0.708692;
    public static final double kRearLeftAbsoluteEncoderOffset = (2.79/5);// * 2 * Math.PI; // old raw value 0.587064;
    public static final double kFrontRightAbsoluteEncoderOffset = (3.36/5);//* 2 * Math.PI; // old raw value0.679514;
    public static final double kRearRightAbsoluteEncoderOffset = (1.06/5);// * 2 * Math.PI; //0.167231; update with new value

    //Gyro
    public static final boolean kGyroReversed = true;
    //TODO: double check that navx gyro needs to be reversed

/*
    // TODO: What are these values for?
    // TODO: These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;
*/


    //values for TurnToAngle from example -- sets when turn to angle is complete (stationary)
    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

    //vision processing 
    //scoring distance - distance from AprilTag to location of robot for scoring
    public static final double kScoringDistance = Units.inchesToMeters(14);
  }




  public static final class PhysicalConstants{
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

    //Max Drive speed (calculated).  May be able to adjust.

    //TODO: tweek max speed/acceleration
    public static final double kMaxSpeedFeetPerSecond = 12.33; //from JVN calc (maybe adjust up to 14.5)
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(kMaxSpeedFeetPerSecond);
    public static final double kMaxAccelerationMetersPerSecondSquared = kMaxSpeedMetersPerSecond * 0.75; //modify?

    public static final double kMaxSteerSpeedRPM = 90.0; //from andymark
    public static final double kMaxAngularSpeedRadiansPerSecond = (kMaxSteerSpeedRPM * 2 * Math.PI) / 60.0;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = kMaxAngularSpeedRadiansPerSecond * 0.75; //modify?
  }



  
  public static final class ModuleConstants {

    //TODO: determine this value  ?where is this used?
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kTurningGearRatio = 40.0/48.0; //reverse?

    public static final double kDriveGearRatio = 6.67;
    //public static final int kDriveCPR = (int)(42 * kDriveGearRatio);
    public static final int kDriveCPR = 42; //required value for NEO hall effect encoder; alter value using conversion factor

    public static final double kWheelDiameterInches = 4.0;
    public static final double kDriveEncoderRot2Meter = (Math.PI * Units.inchesToMeters(kWheelDiameterInches))/kDriveGearRatio; 
    
    public static final double kTurningEncoderRot2Rad = 2 * Math.PI/kTurningGearRatio;
    //public static final double kTurningEncoderRot2Deg = 360/kTurningGearRatio;
    public static final double kTurningEncoderDistancePerPulse = (2*Math.PI)/(7.0*71.16); //2pi rad / 7 cpr*71.16 gear ratio

    //velocity
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter * (5676 / kDriveGearRatio) / 60; //meters per rev * motor rpm / gearing / sec
    //public static final double kDriveEncoderRPM2MeterPerSec = 4.0; //calculated 4.5 ; adjusted 
    //public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad * (75)/ 60; //6.5 radians per sec
    


    //TODO: update P values
    public static final double kPModuleTurningController = 1.75;
    public static final double kPModuleDriveController = 1;  

    public static final double kIModuleTurningController = 0.0;
    public static final double kIModuleDriveController = 0;

    public static final double kDModuleTurningController = 0;
    public static final double kDModuleDriveController = 0;


  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    
    //slew rate 3 --> 1/3 sec from 0 to 1
    //public static final double kSlewRateLimit = 1.0; // adjust?

    public static final int kD_Joystick_Left_X_Speed_Limit = 1;
    public static final int kD_Joystick_Left_Y_Speed_Limit = 1;
    public static final int kD_Joystick_Right_X_Speed_Limit = 1;
    public static final int kD_Joystick_Right_Y_Speed_Limit = 1;
    
    public static final int kO_Joystick_Left_X_Speed_Limit = 1;
    public static final int kO_Joystick_Left_Y_Speed_Limit = 1;
    public static final int kO_Joystick_Right_X_Speed_Limit = 1;
    public static final int kO_Joystick_Right_Y_Speed_Limit = 1;

    //deadband  0.03 adjust?
    public static final double kJoystick_Deadband = 0.06;


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
  //public static final int kD_Mid_Right = 10;
  /* TODO:
  I believe there is a mid left and mid right button at 9 and maybe 10
   * As a result, the left and right joystick button values may need to change
   * 
   * If so, adjust operator values as well and add bindings to RobotContainer
   */
  public static final int kD_Left_Joystick = 9;
  public static final int kD_Right_Joystick = 10;

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



    //TODO: do these values need to be changed?
    //Controller PID
    public static final double kPXController = 0.1;
    public static final double kPYController = 0.1;
    public static final double kPThetaController = 0.1;

    public static final double kIXController = 0;
    public static final double kIYController = 0;
    public static final double kIThetaController = 0;

    public static final double kDXController = 0;
    public static final double kDYController = 0;
    public static final double kDThetaController = 0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            PhysicalConstants.kMaxAngularSpeedRadiansPerSecond, 
            PhysicalConstants.kMaxAngularSpeedRadiansPerSecondSquared);
  }




  public static final class ArmConstants {  

    //Arm Conversions
    //public static final double kArmLiftGearRatio = 100.0/1.0;
    //public static final double kWristGearRatio = 27.0/1.0 //TODO: is this correct?
    //public static final double kArmLiftCPR = 42.0 * kArmLiftGearRatio;
    //public static final double kArmLiftRadPerTick = kArmLiftRadPerRotation / kArmLiftCPR;
    //public static final double kArmLiftDegPerRotation = 360 / kArmLiftGearRatio;
    //public static final double kArmLiftDegPerTick = kArmLiftDegPerRotation/kArmLiftCPR;
    public static final double kArmLiftPositionConversionFactor = 3.6; //360 degrees / 100 ticks per revolution
    public static final double kWristPositionConversionFactor = 13.3; //360 degrees / 27 ticks per revolution

    //CAN Address - Arm Motors 40s
    public static final int kArmLiftMotor = 40;
    public static final int kArmExtensionMotor = 41;

    public static final boolean kArmLiftMotorInverted = false;
    public static final boolean kArmExtensionMotorInverted = false;
    public static final boolean kWristMotorInverted = false;

    //CAN Address - Gripper Motor 50s
    public static final int kWristMotor = 50;


    public static final int kGripper = 51;  //TODO: if using motor for gripper


    public static final boolean kGripperMotorInverted = false;


    //PCM channels //TODO: if using pneumatics for gripper
    public static final int kGripperOpen = 0;
    public static final int kGripperClosed = 1;

    //Wrist Gyro navx analog port //TODO: determine actual port number
    public static final int kWristPositionPort = 1;

    //Address Ultrasonic navX analog port //TODO: determine actual port number 

    public static final int kArmExtensionRangefinderPort=0;

      //TODO: tune values;  
    //ArmLift PID
    public static final double kPArmLiftMotor = 0.1;
    public static final double kIArmLiftMotor = 0.0;
    public static final double kDArmLiftMotor = 0.0;
    public static final double kIzArmLiftMotor = 0.0;
    public static final double kFFArmLiftMotor = 0.0;
    public static final double kMaxOutputArmLiftMotor = 1.0;
    public static final double kMinOutputArmLiftMotor = -1.0;
    public static final double kSVoltsArmLiftMotor = 0;
    public static final double kGVoltsArmLiftMotor = 0;
    public static final double kVVoltSecondPerRadArmLiftMotor = 0;
    public static final double kAVoltSecondSquaredPerRadArmLiftMotor = 0;
    public static final double kMaxVelocityRadPerSecondArmLiftMotor = 0;
    public static final double kMaxAccelerationRadPerSecondSquaredArmLiftMotor = 0;
    //public static final double kArmLiftMotorOffsetRads = 0;
    public static final double kArmLiftMotorSpeed = 0.3; //TODO:  adjust constant arm speed


    //arm encoder soft limits
    public static final double kArmLiftMinHeightDeg = 0; 
    public static final double kArmLiftMaxHeightDeg = 90;
    //max/min wrist angles
    public static final double kWristMinAngleDeg = -90;
    public static final double kWristMaxAngleDeg = 0;

    //ArmExtension PID
    public static final double kPArmExtensionMotor = 0.1;
    public static final double kIArmExtensionMotor = 0.0;
    public static final double kDArmExtensionMotor = 0.0;
    public static final double kMaxOutputArmExtensionMotor = 1.0;
    public static final double kMinOutputArmExtensionMotor = -1.0;
    public static final double kArmExtensionMotorSpeed = 0.3; //TODO: adjust constant extension speed
    //max/min rangefinder limits
    public static final double kArmExtensionMinDistance = 30;
    public static final double kArmExtensionMaxDistance = 50;
  



    //WristMotor PID
    public static final double kPWristMotor = 0.1;
    public static final double kIWristMotor = 0.0;
    public static final double kDWristMotor = 0.0;
    public static final double kMaxOutputWristMotor = 1.0;
    public static final double kMinOutputWristMotor = -1.0;
    public static final double kWristMotorSpeed = 0.3; //TODO: adjust constant extension speed
    //need max/min rangefinder limits
    public static final double kWristMotorMinDistance = 0;
    public static final double kWristMotorMaxDistance = 90;

    public static final double kWristStowed = 0; //TODO: update to correct angle
    public static final double kWristDeployed = 90;


  }


  public static final class FieldConstants {
    //Measurements in partial arm rotations (Lift) and inches (Extension)
    //Heights based on field drawing heights + 8 inches with arm in extended position
    //TODO: after testing need to adjust heights
    // scoring positions as double[] xxx = {lift angle, extension distance, wrist angle}
    // lift angle in degrees on rotated axis - stowed = 0, high cone = 100?
    // extension in inches beginning at 12 - stowed = 12, high cone = 32?
    public static double[] kArmStowed = {0,12, 0};
    public static double[] kGroundPickup = {15, 14, 0};
    public static double[] kDriveHeight = {30, 14, 0};
    public static double[] kMidCube = {60, 25, 0};
    public static double[] kMidCone = {65, 25, 0};
    public static double[] kHighCube = {90, 32, 0};
    public static double[] kHighCone = {110, 32, 0};

    //Field positions - (x, y, z, rotation) (inches, inches, inches, degrees)
    //Red alliance Outer, Coop, Inner
    public static final double[] kAprilTag1 = {610.77, 42.19, 18.22, 180};
    public static final double[] kAprilTag2 = {610.77, 108.19, 18.22, 180};
    public static final double[] kAprilTag3 = {610.77, 174.19, 18.22, 180};
    //Blue alliance substation
    public static final double[] kAprilTag4 = {636.96, 265.74, 27.38, 180};
    //Red alliance substation
    public static final double[] kAprilTag5 = {14.25, 265.74, 27.38, 0};
    //Blue alliance Inner, Coop, Outer
    public static final double[] kAprilTag6 = {40.45, 174.19, 18.22, 0};
    public static final double[] kAprilTag7 = {40.45, 108.19, 18.22, 0};
    public static final double[] kAprilTag8 = {40.45, 42.19, 18.22, 0};

    //Blue alliance scoring positions 
    public static final double[] kBlueOuterLeft = {54.25, 20.19, 0, 180};
    public static final double[] kBlueOuterMid = {54.25, 42.19, 0, 180};
    public static final double[] kBlueOuterRight = {54.25, 64.19, 0 ,180};
    
    public static final double[] kBlueCoopLeft = {54.25, 86.19, 0, 180};
    public static final double[] kBlueCoopMid = {54.25, 108.19, 0, 180};
    public static final double[] kBlueCoopRight = {54.25, 130.19, 0, 180};

    public static final double[] kBlueInnerLeft = {54.25, 152.19, 0, 180};
    public static final double[] kBlueInnerMid = {54.25, 174.19, 0, 180};
    public static final double[] kBlueInnerRight = {54.25, 196.19, 0, 180};

    //Blue alliance staging positions
    public static final double[] kBlueStagingOuterRight = {278.25, 36.19};
    public static final double[] kBlueStagingInnerRight = {278.25, 84.19};
    public static final double[] kBlueStagingInnerLeft = {278.25, 132.19};
    public static final double[] kBlueStagingOuterLeft = {278.25, 180.19};

    //Blue Charging station 
    public static final double[] kBlueCharging = {292, 109.7};

    //Red alliance scoring positions 
    public static final double[] kRedOuterLeft = {596.97, 20.19, 0, 0};
    public static final double[] kRedOuterMid = {596.97, 42.19, 0, 0};
    public static final double[] kRedOuterRight = {596.97, 64.19, 0, 0};
    
    public static final double[] kRedCoopLeft = {596.97, 86.19, 0, 0};
    public static final double[] kRedCoopMid = {596.97, 108.19, 0, 0};
    public static final double[] kRedCoopRight = {596.97, 130.19, 0, 0};

    public static final double[] kRedInnerLeft = {596.97, 152.19, 0, 0};
    public static final double[] kRedInnerMid = {596.97, 174.19, 0, 0};
    public static final double[] kRedInnerRight = {596.97, 196.19, 0, 0};

    //Red alliance staging positions
    public static final double[] kRedStagingOuterRight = {372.97, 180.19};
    public static final double[] kRedStagingInnerRight = {372.97, 132.19};
    public static final double[] kRedStagingInnerLeft = {372.97, 84.19};
    public static final double[] kRedStagingOuterLeft = {372.97, 36.19};

    //Red Charging station 
    public static final double[] kRedCharging = {508.5, 109.7};


  }

  public static final class VisionConstants {
    public static final String kCameraName = "tempCamera";
    public static final double kCameraHeightMeters = Units.inchesToMeters(36); //TODO: measure this value
    public static final double kCameraPitchRadians = Units.degreesToRadians(0);
    
    public static final double kAutoTargetRange = 2.0; //only auto target within 2 meters
    public static final double kAprilTagHeightMeters = Units.inchesToMeters(FieldConstants.kAprilTag1[2]);

    public static final Transform3d kRobotToCam =
    new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(
                    0, 
                    0,
                    0));  // Cam mounted facing forward, half a meter forward of center, half a meter up
                              // from center.
    
    //auto drive distance from april tag in meters
    //tag to line = 13.8"
    //position 24" behind line
    public static final double kGoalDistanceToTarget = Units.inchesToMeters(37.8);

    //cone scoring positions are 20" to either side of the aprilTag
    public static final double kConeStrafeDistance = Units.inchesToMeters(20);

  }
}
