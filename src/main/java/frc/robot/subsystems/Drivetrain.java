// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import static frc.robot.Constants.DriveConstants.*;

import com.kauailabs.navx.frc.AHRS;


//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalConstants;

public class Drivetrain extends SubsystemBase {
  // Robot swerve modules
    /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor SparkMax
   * @param turningMotorChannel The channel of the turning motor VictorSPX
   * @param driveMotorReversed Is the drive motor reversed
   * @param turningMotorReversed Is the turning motor reversed
   * @param turningEncoderChannel ANA port of MA3 turning encoder
   * @param turningEncoderOffset calculated value for offset
   * @param turningEncoderReversed is the turning encoder reversed
   */
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          kFrontLeftDriveMotorPort,
          kFrontLeftTurningMotorPort,
          kFrontLeftDriveMotorReversed,
          kFrontLeftTurningMotorReversed,
          kFrontLeftTurningEncoderPorts,
          kFrontLeftTurningEncoderOffset
          );

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          kRearLeftDriveMotorPort,
          kRearLeftTurningMotorPort,
          kRearLeftDriveMotorReversed,
          kRearLeftTurningMotorReversed,
          kRearLeftTurningEncoderPorts,
          kRearLeftTurningEncoderOffset
          );

  private final SwerveModule m_frontRight =
      new SwerveModule(
          kFrontRightDriveMotorPort,
          kFrontRightTurningMotorPort,
          kFrontRightDriveMotorReversed,
          kFrontRightTurningMotorReversed,
          kFrontRightTurningEncoderPorts,
          kFrontRightTurningEncoderOffset
          );

  private final SwerveModule m_rearRight =
      new SwerveModule(
          kRearRightDriveMotorPort,
          kRearRightTurningMotorPort,
          kRearRightDriveMotorReversed,
          kRearRightTurningMotorReversed,
          kRearRightTurningEncoderPorts,
          kRearRightTurningEncoderOffset
          );

  // The navX MXP gyro sensor
  private final AHRS m_gyro = new AHRS();

  // Odometry class for tracking robot pose
  
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          PhysicalConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });



  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    //pause gyro reset 1 sec to avoid interferring with gyro calibration
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    // Update sensor readings
    SmartDashboard.putNumber("Gyro Angle",m_gyro.getAngle());
    SmartDashboard.putNumber("Gyro Rate",m_gyro.getRate());
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    






  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */

  // Is this section necessary with the DriveWithJoystics command? 
  // Used in autonomous example

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
          PhysicalConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
  


  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(),360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
  }

  /* 
  //method commands
  public CommandBase doNothing() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          // one-time action goes here 
          drive(0,0,0,true);
        });
      }
      */
    public void stopModules() {
      m_frontLeft.stop();
      m_rearLeft.stop();
      m_frontRight.stop();
      m_rearRight.stop();
    }


}

