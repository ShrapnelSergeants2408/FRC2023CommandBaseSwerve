// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
//import static frc.robot.Constants.ArmConstants.*;

public class WristLevel extends CommandBase {
  /** Creates a new WristLevel. */
  private final Wrist wristSubsystem;
  private final PIDController wristPID;


  
  public WristLevel(Wrist wristSubsystem, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wristSubsystem = wristSubsystem;

    wristPID = wristSubsystem.setWristPID();
    wristPID.setSetpoint(setpoint);

    addRequirements(wristSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = wristPID.calculate(wristSubsystem.getWristPosition());
    wristSubsystem.setWristMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setWristMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
