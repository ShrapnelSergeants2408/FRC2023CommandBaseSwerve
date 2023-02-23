// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

//import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PhysicalConstants;

import frc.robot.subsystems.Drivetrain;

public class DriveWithJoysticks extends CommandBase {
  private final Drivetrain driveTrain;
  private final Supplier<Double> xSpeedFunction, ySpeedFunction, turningSpeedFunction;
  private final Supplier<Boolean> fieldOriented;
  //private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;


  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(
      Drivetrain driveTrain, 
      Supplier<Double> xSpeedFunction, 
      Supplier<Double> ySpeedFunction,
      Supplier<Double> turningSpeedFunction,
      Supplier<Boolean> fieldOriented) {
    this.driveTrain = driveTrain;
    this.xSpeedFunction = xSpeedFunction;
    this.ySpeedFunction = ySpeedFunction;
    this.turningSpeedFunction = turningSpeedFunction;
    this.fieldOriented = fieldOriented;
    /*
    this.xLimiter = new SlewRateLimiter(OIConstants.kSlewRateLimit);
    this.yLimiter = new SlewRateLimiter(OIConstants.kSlewRateLimit);
    this.turnLimiter = new SlewRateLimiter(OIConstants.kSlewRateLimit);
    */

    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_xSpeed = xSpeedFunction.get();
    double m_ySpeed = ySpeedFunction.get();
    double m_turningSpeed = turningSpeedFunction.get();

    //apply deadband
    //m_xSpeed = Math.abs(m_xSpeed) > OIConstants.kJoystick_Deadband ? m_xSpeed : 0;
    //m_ySpeed = Math.abs(m_ySpeed) > OIConstants.kJoystick_Deadband ? m_ySpeed : 0;
    //m_turningSpeed = Math.abs(m_turningSpeed) > OIConstants.kJoystick_Deadband ? m_turningSpeed : 0;

    //apply slew rate limiter
    //TODO:may add speed scaling multiplier to slow max speed for control purposes
    //m_xSpeed = xLimiter.calculate(m_xSpeed);
    //m_ySpeed = yLimiter.calculate(m_ySpeed);
    //m_turningSpeed = turnLimiter.calculate(m_turningSpeed);

    //set chassis speeds
    ChassisSpeeds chassisSpeeds;
    if(fieldOriented.get()){
      //relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        m_xSpeed,
        m_ySpeed,
        m_turningSpeed,
        driveTrain.getRotation2d());
    } else {
      //relative to robot
      chassisSpeeds = new ChassisSpeeds(m_xSpeed, m_ySpeed, m_turningSpeed);
    }

    SwerveModuleState[] moduleStates = PhysicalConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
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
