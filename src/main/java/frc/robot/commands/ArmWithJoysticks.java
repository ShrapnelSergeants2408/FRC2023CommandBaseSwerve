// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;

public class ArmWithJoysticks extends CommandBase {
  private final Arm armSubsystem;
  private final Supplier<Double> armLift, armExtend;

  /** Creates a new ArmWithJoysticks. */
  public ArmWithJoysticks(
    Arm armSubsystem, 
    Supplier<Double> armLift, 
    Supplier<Double> armExtend)  
  {
    this.armSubsystem = armSubsystem;
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
    double m_armLift = armLift.get();
    double m_armExtend = armExtend.get();

    //double m_armExtentionDistance = armSubsystem.

    //apply deadband
    m_armLift = Math.abs(m_armLift) > OIConstants.kJoystick_Deadband ? m_armLift : 0;
    m_armExtend = Math.abs(m_armExtend) > OIConstants.kJoystick_Deadband ? m_armExtend : 0;

    //Apply soft limit
    if ((armSubsystem.getArmLiftMeasurement()>= ArmConstants.kArmLiftMaxHeight) &&
        (m_armLift > 0) ||
        (armSubsystem.getArmLiftMeasurement()<= ArmConstants.kArmLiftMinHeight) &&
        (m_armLift <0))
        {
          m_armLift = 0;
        }

    if ((armSubsystem.getArmExtensionMeasurement()>= ArmConstants.kArmExtensionMaxDistance) &&
        (m_armExtend > 0) ||
        (armSubsystem.getArmExtensionMeasurement()<= ArmConstants.kArmExtensionMinDistance) &&
        (m_armExtend <0))
        {
          m_armExtend = 0;
        }

        
    armSubsystem.armMovement(m_armLift, m_armExtend);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
