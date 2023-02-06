// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmToHeight extends CommandBase {
  /** Creates a new ArmToHeight. */
  private final ArmSubsystem m_armSubsystem;
  private final SparkMaxPIDController m_ArmLiftMotorPIDController;
  private final RelativeEncoder m_ArmLiftMotorEncoder;

  public ArmToHeight(ArmSubsystem armSubsystem, SparkMaxPIDController armLiftMotorPID, RelativeEncoder armLiftMotorEncoder, double armSetpointInches) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = armSubsystem;
    addRequirements(armSubsystem);

    m_ArmLiftMotorPIDController = armLiftMotorPID;
    m_ArmLiftMotorEncoder = armLiftMotorEncoder;
    double m_armSetpointInches = armSetpointInches;

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      // PID coefficients
      double kP = ArmConstants.kPArmLiftMotor; 
      double kI = ArmConstants.kIArmLiftMotor;
      double kD = ArmConstants.kDArmLiftMotor;
      double kIz = ArmConstants.kIzArmLiftMotor;
      double kFF = ArmConstants.kFFArmLiftMotor;
      double kMaxOutput = ArmConstants.kMaxOutputArmLiftMotor; 
      double kMinOutput = ArmConstants.kMinOutputArmLiftMotor;
      
      // read PID coefficients from SmartDashboard
       double p = SmartDashboard.getNumber("P Gain Arm Lift", 0);
       double i = SmartDashboard.getNumber("I Gain Arm Lift", 0);
       double d = SmartDashboard.getNumber("D Gain Arm Lift", 0);
       double iz = SmartDashboard.getNumber("I Zone Arm Lift", 0);
       double ff = SmartDashboard.getNumber("Feed Forward Arm Lift", 0);
       double max = SmartDashboard.getNumber("Max Output Arm Lift", 0);
       double min = SmartDashboard.getNumber("Min Output Arm Lift", 0);
       double rotations = SmartDashboard.getNumber("Set Rotations Arm Lift", 0);
   
       // if PID coefficients on SmartDashboard have changed, write new values to controller
       if((p != kP)) { m_ArmLiftMotorPIDController.setP(p); kP = p; }
       if((i != kI)) { m_ArmLiftMotorPIDController.setI(i); kI = i; }
       if((d != kD)) { m_ArmLiftMotorPIDController.setD(d); kD = d; }
       if((iz != kIz)) { m_ArmLiftMotorPIDController.setIZone(iz); kIz = iz; }
       if((ff != kFF)) { m_ArmLiftMotorPIDController.setFF(ff); kFF = ff; }
       if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_ArmLiftMotorPIDController.setOutputRange(min, max); 
        kMaxOutput = max; 
        kMinOutput = min;
       }

       m_ArmLiftMotorPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
       SmartDashboard.putNumber("SetPoint", rotations);
       SmartDashboard.putNumber("ProcessVariable", m_ArmLiftMotorEncoder.getPosition());
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
