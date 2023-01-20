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
  private final DutyCycleEncoder m_turningEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          kPModuleTurningController,
          0,
          0,
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
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int driveEncoderChannels,
      int turningEncoderChannels,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new VictorSPX(turningMotorChannel);
    m_driveEncoder = m_driveMotor.getEncoder(); //update with cpr and gear ratio?
    m_turningEncoder = new DutyCycleEncoder(turningEncoderChannels);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor(kDriveEncoderDistancePerPulse); //check this value
    m_driveEncoder.setVelocityConversionFactor(kDriveEncoderDistancePerPulse); //check this value


    // Set whether drive encoder should be reversed or not
    m_driveEncoder.setInverted(driveEncoderReversed);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //m_turningEncoder.setDistancePerPulse(kTurningEncoderDistancePerPulse);

    // Set the distance (in this case, angle) in radians per revolution for the turning encoder 
    // (duty cycle encoder does not have distance per pulse method).  The full rotation (2 * pi)
    // will need to be divided by the encoder resolution when finding the distance

    m_turningEncoder.setDistancePerRotation(kTurningEncoderDistancePerRotation);
    

    // Set whether turning encoder should be reversed or not
    // Duty cycle encoder not reversible.  If it needs to be reversed it will need to 
    // happen when direction is used

    //m_turningEncoder.setReverseDirection(turningEncoderReversed);
    

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
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
        (m_driveEncoder.getVelocity()/60), new Rotation2d(m_turningEncoder.getDistance()));
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
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getDistance()));

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
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    //m_driveEncoder.reset();
    m_driveEncoder.setPosition(0.0);
    m_turningEncoder.reset();
  }
}
