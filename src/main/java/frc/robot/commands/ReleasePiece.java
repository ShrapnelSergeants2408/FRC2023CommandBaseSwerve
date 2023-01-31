// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class ReleasePiece extends CommandBase {
  /** Creates a new ReleasePiece. */
  private final Gripper m_Gripper;
  
  public ReleasePiece(Gripper gripperSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Gripper = gripperSubsystem;
    addRequirements(m_Gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Gripper.releasePiece();
    SmartDashboard.putString("Gripper State", "Open");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
