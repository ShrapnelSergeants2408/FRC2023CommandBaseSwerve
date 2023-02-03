// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveWithJoysticks extends CommandBase {
  private final Drivetrain driveTrain;
  private final Supplier<Double> xSpeedFunction, ySpeedFunction, turningSpeedFunction;
  private final Supplier<Boolean> fieldOriented;
  private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;




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
    this.xLimiter = new SlewRateLimiter(OIConstants.kSlewRateLimit);
    this.yLimiter = new SlewRateLimiter(OIConstants.kSlewRateLimit);
    this.turnLimiter = new SlewRateLimiter(OIConstants.kSlewRateLimit);

    addRequirements(driveTrain);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpeedFunction.get();
    double ySpeed = ySpeedFunction.get();
    double turningSpeed = turningSpeedFunction.get();

    //apply deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kJoystick_Deadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kJoystick_Deadband ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kJoystick_Deadband ? turningSpeed : 0;

    //apply slew rate limiter
    //TODO:may add speed scaling multiplier to slow max speed for control purposes
    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);
    turningSpeed = turnLimiter.calculate(turningSpeed);

    //set chassis speeds
    ChassisSpeeds chassisSpeeds;
    if(fieldOriented.get()){
      //relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed,
        ySpeed,
        turningSpeed,
        driveTrain.getRotation2d());
    } else {
      //relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }
  } 


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
