// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;

public class ArmWithJoysticks extends CommandBase {
  private final Arm m_armSubsystem;
  private final Supplier<Double> armLift, armExtend;

  /** Creates a new ArmWithJoysticks. */
  public ArmWithJoysticks(
    Arm armSubsystem, 
    Supplier<Double> armLift, 
    Supplier<Double> armExtend)  
  {
    m_armSubsystem = armSubsystem;
    this.armLift = armLift;
    this.armExtend = armExtend;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //get values and apply deadband
    double m_armLift = MathUtil.applyDeadband(armLift.get(), OIConstants.kJoystick_Deadband);
    double m_armExtend = MathUtil.applyDeadband(armExtend.get(), OIConstants.kJoystick_Deadband);
    
    //double m_armExtentionDistance = armSubsystem.
    
    //scale for safety TODO: modify once tested
    m_armLift = m_armLift * ArmConstants.kArmLiftMotorSpeed;
    m_armExtend = m_armExtend * ArmConstants.kArmExtensionMotorSpeed;

    //Apply soft limits
    if ((m_armSubsystem.getArmLiftMeasurement()>= ArmConstants.kArmLiftMaxHeightDeg) &&
        (m_armLift > 0) ||
        (m_armSubsystem.getArmLiftMeasurement()<= ArmConstants.kArmLiftMinHeightDeg) &&
        (m_armLift <0))
        {
          m_armLift = 0;
        }
/* TODO: activate when sonar set up
    if ((m_armSubsystem.getArmExtensionMeasurement()>= ArmConstants.kArmExtensionMaxDistance) &&
        (m_armExtend > 0) ||
        (m_armSubsystem.getArmExtensionMeasurement()<= ArmConstants.kArmExtensionMinDistance) &&
        (m_armExtend <0))
        {
          m_armExtend = 0;
        }
*/
        
    m_armSubsystem.armMovement(m_armLift, m_armExtend);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
