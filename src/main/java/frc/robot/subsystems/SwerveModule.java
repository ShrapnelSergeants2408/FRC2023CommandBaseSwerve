// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.DriveConstants.*;

//import edu.wpi.first.wpilibj.Encoder;
import static frc.robot.Constants.ModuleConstants.*;


import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final VictorSPX m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  //private final double m_turningEncoder;
  private final DutyCycleEncoder m_turningEncoder;

  private final double m_turningEncoderOffset;
  private final boolean m_turningEncoderReversed;

  //TODO: update ki, kd values for drivePID and turningPID controllers

  private final PIDController m_drivePIDController =
      new PIDController(kPModuleDriveController, kIModuleDriveController, kDModuleDriveController);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          kPModuleTurningController,
          kIModuleTurningController,
          kDModuleTurningController,
          new TrapezoidProfile.Constraints(
              kMaxModuleAngularSpeedRadiansPerSecond,
              kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   * @param turningEncoderOffset 
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int driveEncoderChannels,
      int turningEncoderChannels,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed,
      double turningEncoderOffset) {

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new VictorSPX(turningMotorChannel);

    m_turningEncoder = new DutyCycleEncoder(turningEncoderChannels);

    m_driveEncoder = m_driveMotor.getEncoder(); //update with cpr and gear ratio?
    m_turningEncoder.getAbsolutePosition();

    m_turningEncoderOffset = turningEncoderOffset;
    m_turningEncoderReversed = turningEncoderReversed;

    sendTelemetry(driveMotorChannel);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor(kDriveEncoderRot2Meter); //check this value
    m_driveEncoder.setVelocityConversionFactor(kDriveEncoderRPM2MeterPerSec); //check this value


    // Set whether drive encoder should be reversed or not
    m_driveEncoder.setInverted(driveEncoderReversed);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //m_turningEncoder.setDistancePerPulse(kTurningEncoderDistancePerPulse);

    // Set the distance (in this case, angle) in radians per revolution for the turning encoder 
    // (duty cycle encoder does not have distance per pulse method).  The full rotation (2 * pi)
    // will need to be divided by the encoder resolution when finding the distance

    m_turningEncoder.setDistancePerRotation(kTurningEncoderRot2Rad);
    //m_DutyCycleEncoder.setVelocityConversionFactor(kTurningEncoderRPM2RadPerSec);
    

    // Set whether turning encoder should be reversed or not
    // Duty cycle encoder not reversible.  If it needs to be reversed it will need to 
    // happen when direction is used

    //m_turningEncoder.setReverseDirection(turningEncoderReversed);
   

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();

  }

  /*  
   *  These next 5 may not be necessary.  Added following the example from
   *  FRC0toAutonomous.
   */

  public double getDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return m_driveEncoder.getVelocity();
  }

  public double getTurnPosition() {
    return m_turningEncoder.get();
  }
/* is turn velocity necessary?
  public double getTurnVelocity() {
    return 
  }
*/

  public double getAbsoluteEncoderRad() {
    double angle = m_turningEncoder.get()*kTurningEncoderRot2Rad;
    angle -= m_turningEncoderOffset; //adjust for wheel offset
    return angle*(m_turningEncoderReversed ? -1.0:1.0);
  }



  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        //m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
        // .getVelocity() is divided by 60 to convert from revolutions per minute to 
        //  revolutions per second
        //
        // if m_turningEncoder needs to be reversed add - here or include conditional in Rotation2d
        //(m_driveEncoder.getVelocity()/60), new Rotation2d(m_turningEncoder.getDistance()));
        (m_driveEncoder.getVelocity()/60), new Rotation2d(getTurnPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        //m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));

        // if m_turningEncoder needs to be reversed add - here or include conditional in Rotation2d
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {


    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        // if m_turningEncoder needs to be reversed add - here or include conditional in Rotation2d
        //SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getDistance()));
        SwerveModuleState.optimize(desiredState,getState().angle);

    //don't reset wheels to 0 if not in motion
    if (Math.abs(state.speedMetersPerSecond)< 0.001) {
      stop();
      return;
    }

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        // .getVelocity() is divided by 60 to convert from revolutions per minute to 
        //  revolutions per second
        m_drivePIDController.calculate((m_driveEncoder.getVelocity()/60), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    //m_turningMotor.set(turnOutput);
    m_turningMotor.set(VictorSPXControlMode.PercentOutput,turnOutput);

    SmartDashboard.putString("Swerve["+ m_turningEncoder.getSourceChannel()+"] state", state.toString());
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    //m_driveEncoder.reset();
    m_driveEncoder.setPosition(0.0);
    //m_turningEncoder.setPosition(getAbsoluteEncoderRad());
    m_turningEncoder.reset();
  }

  public void sendTelemetry(int module){
    if (module == kFrontLeftDriveMotorPort)
    {
      SmartDashboard.putNumber("FL Drive Encoder Position", m_driveEncoder.getPosition());
      SmartDashboard.putNumber("FL Drive Encoder Velocity",m_driveEncoder.getVelocity());
      SmartDashboard.putNumber("FL Drive Encoder Pos Conversion Factor", m_driveEncoder.getPositionConversionFactor());
      SmartDashboard.putNumber("FL Drive Encoder Vel Conversion Factor", m_driveEncoder.getVelocityConversionFactor());

      SmartDashboard.putNumber("FL Turning Encoder value", m_turningEncoder.get());
      SmartDashboard.putNumber("FL Turning Encoder distance", m_turningEncoder.getDistance());
      SmartDashboard.putNumber("FL Turning Encoder absolute position", m_turningEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("FL Turning Encoder distance per rotation", m_turningEncoder.getDistancePerRotation());

    }
    else if (module == kRearLeftDriveMotorPort)
    {
      SmartDashboard.putNumber("RL Drive Encoder Position", m_driveEncoder.getPosition());
      SmartDashboard.putNumber("RL Drive Encoder Velocity",m_driveEncoder.getVelocity());
      SmartDashboard.putNumber("RL Drive Encoder Pos Conversion Factor", m_driveEncoder.getPositionConversionFactor());
      SmartDashboard.putNumber("RL Drive Encoder Vel Conversion Factor", m_driveEncoder.getVelocityConversionFactor());

      SmartDashboard.putNumber("RL Turning Encoder value", m_turningEncoder.get());
      SmartDashboard.putNumber("RL Turning Encoder distance", m_turningEncoder.getDistance());
      SmartDashboard.putNumber("RL Turning Encoder absolute position", m_turningEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("RL Turning Encoder distance per rotation", m_turningEncoder.getDistancePerRotation());

    }
    else if (module == kFrontRightDriveMotorPort)
    {
      SmartDashboard.putNumber("FR Drive Encoder Position", m_driveEncoder.getPosition());
      SmartDashboard.putNumber("FR Drive Encoder Velocity",m_driveEncoder.getVelocity());
      SmartDashboard.putNumber("FR Drive Encoder Pos Conversion Factor", m_driveEncoder.getPositionConversionFactor());
      SmartDashboard.putNumber("FR Drive Encoder Vel Conversion Factor", m_driveEncoder.getVelocityConversionFactor());

      SmartDashboard.putNumber("FR Turning Encoder value", m_turningEncoder.get());
      SmartDashboard.putNumber("FR Turning Encoder distance", m_turningEncoder.getDistance());
      SmartDashboard.putNumber("FR Turning Encoder absolute position", m_turningEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("FR Turning Encoder distance per rotation", m_turningEncoder.getDistancePerRotation());

    }
    else if (module == kRearRightDriveMotorPort)
    {
      SmartDashboard.putNumber("RR Drive Encoder Position", m_driveEncoder.getPosition());
      SmartDashboard.putNumber("RR Drive Encoder Velocity",m_driveEncoder.getVelocity());
      SmartDashboard.putNumber("RR Drive Encoder Pos Conversion Factor", m_driveEncoder.getPositionConversionFactor());
      SmartDashboard.putNumber("RR Drive Encoder Vel Conversion Factor", m_driveEncoder.getVelocityConversionFactor());

      SmartDashboard.putNumber("RR Turning Encoder value", m_turningEncoder.get());
      SmartDashboard.putNumber("RR Turning Encoder distance", m_turningEncoder.getDistance());
      SmartDashboard.putNumber("RR Turning Encoder absolute position", m_turningEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("RR Turning Encoder distance per rotation", m_turningEncoder.getDistancePerRotation());

    }
  }
  
}
