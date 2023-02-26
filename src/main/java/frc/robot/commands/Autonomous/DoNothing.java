// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DoNothing extends CommandBase {
  /** Creates a new DoNothing. */
  private final Drivetrain m_drivetrain;

  public DoNothing(Drivetrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = dt;
    addRequirements(dt);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.stopModules();;
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
