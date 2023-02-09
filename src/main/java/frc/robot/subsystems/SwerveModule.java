// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ModuleConstants;
import static frc.robot.Constants.DriveConstants.*;
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

  private final DutyCycleEncoder m_turningEncoder;
  private final double m_turningEncoderOffsetRad;
  //private final boolean m_turningEncoderReversed;  is this needed?

  //TODO: update ki, kd values for drivePID and turningPID controllers

  private final PIDController m_drivePIDController; 
  private final PIDController m_turningPIDController;

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

  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      int turningEncoderChannel,
      double turningEncoderOffset)
      //,boolean turningEncoderReversed) 
  {

    m_turningEncoderOffsetRad = turningEncoderOffset;
    //m_turningEncoderReversed = turningEncoderReversed;
    m_turningEncoder = new DutyCycleEncoder(turningEncoderChannel);

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new VictorSPX(turningMotorChannel);

    m_driveMotor.setInverted(driveMotorReversed);
    m_turningMotor.setInverted(turningMotorReversed);

    m_driveEncoder = m_driveMotor.getEncoder(); 
    m_turningEncoder.getAbsolutePosition();

    m_driveEncoder.setPositionConversionFactor(kDriveEncoderRot2Meter); //check this value
    m_driveEncoder.setVelocityConversionFactor(kDriveEncoderRPM2MeterPerSec); //check this value

    //TODO: verify encoder voltage ranges
    //m_turningEncoder.setDuty(min, max); //may need to send voltages if MA3 encoders are not 
    
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController = new PIDController(
        ModuleConstants.kPModuleTurningController,
        ModuleConstants.kIModuleTurningController,
        ModuleConstants.kDModuleTurningController);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_drivePIDController = new PIDController(
        ModuleConstants.kPModuleDriveController,
        ModuleConstants.kIModuleDriveController,
        ModuleConstants.kDModuleDriveController);
    
    

    resetEncoders();
    sendTelemetry(driveMotorChannel);
  }

  /*  
   *  These next 5 may not be necessary.  Added following the example from
   *  FRC0toAutonomous.
   */

  public double getDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  public double getTurnPosition() {
    return m_turningEncoder.get();
  }

  public double getDriveVelocity() {
    return m_driveEncoder.getVelocity();
  }


/* is turn velocity necessary?
  public double getTurnVelocity() {
    return 
  }
*/

/* not needed for DutyCycleEncoder.  Offset included in resetEncoders.

  public double getAbsoluteEncoderRad() {
    double angle = m_turningEncoder.getAbsolutePosition();
    angle *= 2.0 * Math.PI;
    angle -= m_turningEncoderOffsetRad; //adjust for wheel offset
    return angle*(m_turningEncoderReversed ? -1.0:1.0);
  }
*/

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    //m_driveEncoder.reset();
    m_driveEncoder.setPosition(0.0);
    //m_turningEncoder.setPosition(getAbsoluteEncoderRad());
    m_turningEncoder.reset();
    m_turningEncoder.setPositionOffset(m_turningEncoderOffsetRad);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        //
        // .getVelocity() is divided by 60 to convert from revolutions per minute to 
        //  revolutions per second
        //
        (m_driveEncoder.getVelocity()/60), new Rotation2d(getTurnPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
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


  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(VictorSPXControlMode.PercentOutput, 0);
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
