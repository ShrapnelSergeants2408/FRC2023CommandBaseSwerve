// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {

  //Do nothing auto
  public static CommandBase autoDoNothing(Drivetrain dt){
    return Commands.sequence(dt.doNothing());

  }
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
