// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ModuleConstants;

import edu.wpi.first.wpilibj.AnalogEncoder;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;


public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final VictorSPX m_turningMotor;

  private final RelativeEncoder m_driveEncoder;

  private final AnalogEncoder m_turningEncoder;
  private final double m_turningEncoderOffset;
  private final boolean m_turningEncoderReversed; 

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
      double turningEncoderOffset,
      boolean turningEncoderReversed) 
  {

    m_turningEncoderOffset = turningEncoderOffset;
    m_turningEncoderReversed = turningEncoderReversed;
    m_turningEncoder = new AnalogEncoder(turningEncoderChannel);

    //SmartDashboard.putNumber("turning encoder position"+ turningMotorChannel, m_turningEncoder.getAbsolutePosition());

    //motor setup
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new VictorSPX(turningMotorChannel);

    m_turningMotor.configFactoryDefault(); //maybe remove?

    m_driveMotor.setInverted(driveMotorReversed);
    m_turningMotor.setInverted(turningMotorReversed);

    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);

    //encoder setup
    m_driveEncoder = m_driveMotor.getEncoder(Type.kHallSensor,ModuleConstants.kDriveCPR);


    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter); 
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec); 

    m_turningEncoder.setDistancePerRotation(ModuleConstants.kTurningEncoderRot2Rad); //set in radians
    //m_turningEncoder.setDistancePerRotation(ModuleConstants.kTurningEncoderRot2Deg); //set in degrees
 

    // Limit the PID Controller's input range between -180 and 180 (-pi and pi) and set the input
    // to be continuous.
    m_turningPIDController = new PIDController(
        ModuleConstants.kPModuleTurningController,
        ModuleConstants.kIModuleTurningController,
        ModuleConstants.kDModuleTurningController);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    //m_turningPIDController.enableContinuousInput(-180,180);

    m_drivePIDController = new PIDController(
        ModuleConstants.kPModuleDriveController,
        ModuleConstants.kIModuleDriveController,
        ModuleConstants.kDModuleDriveController);

    resetEncoders();
    //sendTelemetry(driveMotorChannel);
  }


  public double getDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  public double getTurnPosition() {
    return m_turningEncoder.getDistance(); 
  }

  public double getDriveVelocity() {
    return m_driveEncoder.getVelocity();
  }


  public double getAbsoluteEncoderRad() {
    double angle = m_turningEncoder.getAbsolutePosition();
    //account for 1/6pi rad wraparound
    if (m_turningEncoder.getDistance() > ModuleConstants.kTurningEncoderRot2Rad)
    {
      angle += 1; //absolute position should be 0-1 so this will account for the rollover
    }
 
    angle -= m_turningEncoderOffset; //adjust for wheel offset
    angle *= 2.0 * Math.PI;  //conver to radians
    return angle*(m_turningEncoderReversed ? -1.0:1.0);
  }


  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0.0);
    //m_turningEncoder.setPosition(getAbsoluteEncoderRad());
    m_turningEncoder.setPositionOffset(m_turningEncoderOffset);
    m_turningEncoder.reset();

  }



  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
   
  public SwerveModuleState getState() {
    return new SwerveModuleState(

        (m_driveEncoder.getVelocity()), new Rotation2d(getAbsoluteEncoderRad())); //may need to divide velocity by 60
  }





  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        //SwerveModuleState.optimize(desiredState,getState().angle);
        SwerveModuleState.optimize(desiredState,getPosition().angle);

    //don't reset wheels to 0 if not in motion
    if (Math.abs(state.speedMetersPerSecond)< 0.001) {
      stop();
      return;
    }

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate((m_driveEncoder.getVelocity()), state.speedMetersPerSecond); //may need to divide velocity by 60

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        //m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());
        m_turningPIDController.calculate(getTurnPosition(), state.angle.getRadians());

    m_driveMotor.set(driveOutput);
    //m_turningMotor.set(turnOutput);
    m_turningMotor.set(VictorSPXControlMode.PercentOutput,turnOutput);

    SmartDashboard.putString("Swerve["+ m_turningEncoder.getChannel()+"] state", state.toString());
    SmartDashboard.putNumber("Swerve["+ m_turningEncoder.getChannel()+"] drive output", driveOutput);
    SmartDashboard.putNumber("Swerve["+ m_turningEncoder.getChannel()+"] turn angle", getAbsoluteEncoderRad()*180/Math.PI);    
  }
  

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        //m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));

        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getDistance()*Math.PI/180));
        

  }


  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

}
