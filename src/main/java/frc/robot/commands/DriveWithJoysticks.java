// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PhysicalConstants;

import frc.robot.subsystems.Drivetrain;

public class DriveWithJoysticks extends CommandBase {
  private final Drivetrain driveTrain;
  private final Supplier<Double> xSpeedFunction, ySpeedFunction, turningSpeedFunction;
  private final boolean fieldOriented;
  //private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;


  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(
      Drivetrain driveTrain, 
      Supplier<Double> xSpeedFunction, 
      Supplier<Double> ySpeedFunction,
      Supplier<Double> turningSpeedFunction,
      boolean fieldOriented) {
    this.driveTrain = driveTrain;
    this.xSpeedFunction = xSpeedFunction;
    this.ySpeedFunction = ySpeedFunction;
    this.turningSpeedFunction = turningSpeedFunction;
    this.fieldOriented = fieldOriented;
    
    //this.xLimiter = new SlewRateLimiter(OIConstants.kSlewRateLimit);
    //this.yLimiter = new SlewRateLimiter(OIConstants.kSlewRateLimit);
    //this.turnLimiter = new SlewRateLimiter(OIConstants.kSlewRateLimit);
    

    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_xSpeed = xSpeedFunction.get()* PhysicalConstants.kMaxSpeedMetersPerSecond;
    double m_ySpeed = ySpeedFunction.get()* PhysicalConstants.kMaxSpeedMetersPerSecond;
    double m_turningSpeed = turningSpeedFunction.get() * PhysicalConstants.kMaxAngularSpeedRadiansPerSecond;

    //apply deadband
    m_xSpeed = Math.abs(m_xSpeed) > OIConstants.kJoystick_Deadband ? m_xSpeed : 0;
    m_ySpeed = Math.abs(m_ySpeed) > OIConstants.kJoystick_Deadband ? m_ySpeed : 0;
    m_turningSpeed = Math.abs(m_turningSpeed) > OIConstants.kJoystick_Deadband ? m_turningSpeed : 0;

    //apply slew rate limiter
    //m_xSpeed = xLimiter.calculate(m_xSpeed) * PhysicalConstants.kMaxSpeedMetersPerSecond;
    //m_ySpeed = yLimiter.calculate(m_ySpeed) * PhysicalConstants.kMaxSpeedMetersPerSecond;
    //m_turningSpeed = turnLimiter.calculate(m_turningSpeed) * PhysicalConstants.kMaxAngularSpeedRadiansPerSecond;

    //set chassis speeds
    ChassisSpeeds chassisSpeeds;
    if(!fieldOriented){ //TODO: remove !
      //relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        m_xSpeed,//  *PhysicalConstants.kMaxSpeedMetersPerSecond,
        m_ySpeed, // *PhysicalConstants.kMaxSpeedMetersPerSecond,
        m_turningSpeed, // *PhysicalConstants.kMaxAngularSpeedRadiansPerSecond,
        driveTrain.getRotation2d());
    } else {
      //relative to robot
      chassisSpeeds = new ChassisSpeeds(m_xSpeed, // * PhysicalConstants.kMaxSpeedMetersPerSecond,
                                        m_ySpeed, // * PhysicalConstants.kMaxSpeedMetersPerSecond, 
                                        m_turningSpeed // * PhysicalConstants.kMaxAngularSpeedRadiansPerSecond
                                        );
    }
    SmartDashboard.putBoolean("Field Relative", fieldOriented);
    SmartDashboard.putNumber("Joystick 1 X", m_xSpeed );
    SmartDashboard.putNumber("Joystick 1 Y", m_ySpeed );
    SmartDashboard.putNumber("Joystick 2 X", m_turningSpeed );
    
    //convert chassis speed to individucal module states
    SwerveModuleState[] moduleStates = PhysicalConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    //output desired module states to wheels
    driveTrain.setModuleStates(moduleStates);

  } 


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
