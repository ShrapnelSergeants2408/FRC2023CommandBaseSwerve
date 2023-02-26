// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// This may or may not work.  Not sure which P,I,D values to use.
// I chose the module turing constants since it is a turning command.
//
// If it works, consider changing x and y values to values from controller
// and possibly chassis speeds so specific heading rotation can be achieved
// while driving


package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.Drivetrain;


public class TurnToAngle extends PIDCommand {
  /** Creates a new TurnToAngle. */
  public TurnToAngle(double targetAngleDegree, Drivetrain driveTrain) {
    super(
      // The controller that the command will use
      new PIDController(ModuleConstants.kPModuleTurningController, 
                        ModuleConstants.kIModuleTurningController, 
                        ModuleConstants.kDModuleTurningController),
      // This should return the measurement
      driveTrain::getHeading,
      // This should return the setpoint (can also be a constant)
      targetAngleDegree,
      // This uses the output
      //TODO: update xSpeed/ySpeed to take joystick inputs
      output -> 
        driveTrain.stopModules(),
      // subsystem
      driveTrain
    );

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference)
    getController()
      .setTolerance(DriveConstants.kTurnToleranceDeg, 
                    DriveConstants.kTurnRateToleranceDegPerS);
      
    addRequirements(driveTrain);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
