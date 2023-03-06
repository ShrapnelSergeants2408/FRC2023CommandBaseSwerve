// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;

public class ArmWithJoysticks extends CommandBase {
  private final Arm m_armSubsystem;
  private final Supplier<Double> armLift, armExtend, wristAngle;

  /** Creates a new ArmWithJoysticks. */
  public ArmWithJoysticks(
    Arm armSubsystem, 
    Supplier<Double> armLift, 
    Supplier<Double> armExtend,
    Supplier<Double> wristAngle)  
  {
    m_armSubsystem = armSubsystem;
    this.armLift = armLift;
    this.armExtend = armExtend;
    this.wristAngle = wristAngle;

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
    double m_wristAngle = MathUtil.applyDeadband(wristAngle.get(), OIConstants.kJoystick_Deadband);
    
    //double m_armExtentionDistance = armSubsystem.
    
    //scale for safety TODO: modify once tested
    if (m_armLift > 0.0) {
      m_armLift = 1.0;}
    else if (m_armLift < 0){
      m_armLift = -1;}

    if (m_armExtend > 0.0) {
      m_armExtend = 1.0;}
    else if (m_armExtend < 0){
      m_armExtend = -1;}
    
    if (m_wristAngle > 0.0) {
      m_wristAngle = 1.0;}
    else if (m_wristAngle < 0){
      m_wristAngle = -1;}
    
    
    
    m_armLift = m_armLift * ArmConstants.kArmLiftMotorSpeed;
    m_armExtend = m_armExtend * ArmConstants.kArmExtensionMotorSpeed;
    m_wristAngle = m_wristAngle * ArmConstants.kWristMotorSpeed;


    //Apply soft limits
    if ((m_armSubsystem.getArmLiftAngle()>= ArmConstants.kArmLiftMaxHeightDeg) &&
        (m_armLift > 0) ||
        (m_armSubsystem.getArmLiftAngle()<= ArmConstants.kArmLiftMinHeightDeg) &&
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
    if ((m_armSubsystem.getWristAngle()>= ArmConstants.kWristMaxAngleDeg) &&
        (m_wristAngle > 0) ||
        (m_armSubsystem.getArmLiftAngle()<= ArmConstants.kWristMinAngleDeg) &&
        (m_wristAngle <0))
        {
          m_armLift = 0;
        }

    SmartDashboard.putNumber("Arm Lift input", m_armLift);
    SmartDashboard.putNumber("Arm Extend input", m_armExtend);
    SmartDashboard.putNumber("Wrist input", m_wristAngle);

    SmartDashboard.putNumber("Arm Lift encoder", m_armSubsystem.getArmLiftAngle());
    SmartDashboard.putNumber("Arm Extend input", m_armSubsystem.getArmExtensionDistance());
    SmartDashboard.putNumber("Wrist encoder", m_armSubsystem.getWristAngle());
    m_armSubsystem.armMovement(m_armLift, m_armExtend, m_wristAngle);
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
