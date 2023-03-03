// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PhysicalConstants;

import frc.robot.subsystems.Drivetrain;

public class DriveWithJoysticks extends CommandBase {
  private final Drivetrain driveTrain;
  private final Supplier<Double> xSpeedFunction, ySpeedFunction, turningSpeedFunction;
  private final boolean fieldOriented;

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
    
    
    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Get joystick values and deadband
    double m_xSpeed = MathUtil.applyDeadband(xSpeedFunction.get(), OIConstants.kJoystick_Deadband);
    double m_ySpeed = MathUtil.applyDeadband(ySpeedFunction.get(), OIConstants.kJoystick_Deadband);
    double m_turningSpeed = MathUtil.applyDeadband(turningSpeedFunction.get(), OIConstants.kJoystick_Deadband);

    //Drive
    driveTrain.drive(
      m_xSpeed*PhysicalConstants.kMaxSpeedMetersPerSecond,
      m_ySpeed*PhysicalConstants.kMaxSpeedMetersPerSecond,
      m_turningSpeed*PhysicalConstants.kMaxAngularSpeedRadiansPerSecond,
      //fieldOriented
      false
    );

    SmartDashboard.putBoolean("Field Relative", fieldOriented);
    SmartDashboard.putNumber("Joystick 1 X", m_xSpeed );
    SmartDashboard.putNumber("Joystick 1 Y", m_ySpeed );
    SmartDashboard.putNumber("Joystick 2 X", m_turningSpeed );
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
