// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PhysicalConstants;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;

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


  //private final AnalogEncoder m_turningEncoder;
  private final Encoder m_turningEncoder;
  private final AnalogInput m_absoluteEncoder;  //switch to input vs encoder
  private final double m_absoluteEncoderOffset;
  private final boolean m_turningEncoderReversed; 
  private final int m_absoluteEncoderChannel;

  //TODO: update ki, kd values for drivePID and turningPID controllers

  //private final PIDController m_drivePIDController; 
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
      int turningEncoderChannelA,
      int turningEncoderChannelB,
      double absoluteEncoderOffset,
      boolean turningEncoderReversed,
      int absoluteEncoderChannel) 
  {
    m_absoluteEncoderChannel = absoluteEncoderChannel;

    //motor setup
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new VictorSPX(turningMotorChannel);

    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.configFactoryDefault(); //maybe remove?

    m_driveMotor.setInverted(driveMotorReversed);
    m_turningMotor.setInverted(turningMotorReversed);

    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);

    //encoder setup
    
    m_absoluteEncoderOffset = absoluteEncoderOffset;
    m_turningEncoderReversed = turningEncoderReversed;

    m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB) ;
    m_absoluteEncoder = new AnalogInput(absoluteEncoderChannel);

    m_driveEncoder = m_driveMotor.getEncoder(Type.kHallSensor,ModuleConstants.kDriveCPR);
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter); 
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec); 

    m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);
 

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous
    m_turningPIDController = new PIDController(
        ModuleConstants.kPModuleTurningController,
        ModuleConstants.kIModuleTurningController,
        ModuleConstants.kDModuleTurningController);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);


    resetEncoders();

  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */

   public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      getDrivePosition(),
      //new Rotation2d(getAbsoluteEncoderRad())
      new Rotation2d(getTurnPosition())
      /* I think we should only use absolute encoder to initialize wheels to 0 */
    );

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

    double angle = m_absoluteEncoder.getVoltage()/RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;  //convert to radians
    angle -= m_absoluteEncoderOffset; //adjust for wheel offset

    return angle*(m_turningEncoderReversed ? -1.0:1.0);
  }


  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    stop();
    m_driveEncoder.setPosition(0.0);
    //m_turningEncoder.setPosition(getAbsoluteEncoderRad());
    m_turningEncoder.reset();
    //m_turningEncoder.get

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
   
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      (m_driveEncoder.getVelocity()), new Rotation2d(getTurnPosition())); 
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


    /*    removed so wheels WILL reset to 0
    //don't reset wheels to 0 if not in motion
    if (Math.abs(state.speedMetersPerSecond)< 0.001) { //TODO: tune to robot values  is there sensor drift at 0
      stop();
      return;
    }
*/
    // Drive output from joysticks.  joystick converted to m/s divided by max m/s
    final double driveOutput = desiredState.speedMetersPerSecond/PhysicalConstants.kMaxSpeedMetersPerSecond;
    
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getTurnPosition(), state.angle.getRadians());


    m_driveMotor.set(driveOutput);
    m_turningMotor.set(VictorSPXControlMode.PercentOutput,turnOutput);

    SmartDashboard.putNumber("Swerve["+  m_absoluteEncoderChannel +"] drive velocity", driveOutput);
    SmartDashboard.putNumber("Swerve["+  m_absoluteEncoderChannel +"] current turn angle", Units.radiansToDegrees(getTurnPosition())); 
    SmartDashboard.putNumber("Swerve["+  m_absoluteEncoderChannel +"] desired turn angle", state.angle.getDegrees());    
  }
  




  public void stop() {
    m_driveMotor.set(0);

    //set wheels forward
    m_turningMotor.set(VictorSPXControlMode.PercentOutput,
                        m_turningPIDController.calculate(getTurnPosition(), 0.0)
                      );
  }

}
