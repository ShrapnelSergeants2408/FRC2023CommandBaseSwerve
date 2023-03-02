// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmToHeight extends CommandBase {
  /** Creates a new ArmToHeight. */
  private final Arm m_armSubsystem;
  
  private final PIDController m_ArmLiftMotorPIDController;
  private final PIDController m_ArmExtensionMotorPIDController;

  public ArmToHeight(Arm armSubsystem, 
                     //PIDController armLiftMotorPID, 
                    // PIDController armExtensionPID,
                     double armHeightSetpointDegrees,
                     double armExtensionSetpointInches) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = armSubsystem;
    addRequirements(armSubsystem);

    m_ArmLiftMotorPIDController = m_armSubsystem.setArmLiftPID();
    m_ArmLiftMotorPIDController.setSetpoint(armHeightSetpointDegrees);

    m_ArmExtensionMotorPIDController = m_armSubsystem.setArmExtensionPID();
    m_ArmExtensionMotorPIDController.setSetpoint(armExtensionSetpointInches);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmLiftMotorPIDController.reset();
    m_ArmExtensionMotorPIDController.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double armLiftSpeed = m_ArmLiftMotorPIDController.calculate(m_armSubsystem.getArmLiftAngle());
    double armExtensionSpeed = m_ArmExtensionMotorPIDController.calculate(m_armSubsystem.getArmExtensionDistance());

    m_armSubsystem.armMovement(armLiftSpeed, armExtensionSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.armMovement(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
