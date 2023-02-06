// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends TrapezoidProfileSubsystem {
  private final CANSparkMax m_ArmLiftMotor = new CANSparkMax(ArmConstants.kArmLiftMotor,MotorType.kBrushless);
  private final SparkMaxPIDController m_ArmLiftMotorPID;
  private final RelativeEncoder m_ArmLiftEncoder;
  private final ArmFeedforward m_ArmLiftFF = new ArmFeedforward(
                     ArmConstants.kSVoltsArmLiftMotor,
                     ArmConstants.kGVoltsArmLiftMotor,
                     ArmConstants.kVVoltSecondPerRadArmLiftMotor,
                     ArmConstants.kAVoltSecondSquaredPerRadArmLiftMotor);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    super(
      // The constraints for the generated profiles
      new TrapezoidProfile.Constraints(
          ArmConstants.kMaxVelocityRadPerSecondArmLiftMotor,
          ArmConstants.kMaxAccelerationRadPerSecondSquaredArmLiftMotor),
          ArmConstants.kArmLiftMotorOffsetRads);

      // The initial position of the mechanism
    //m_ArmLiftMotor = new CANSparkMax(ArmConstants.kArmLiftMotor,MotorType.kBrushless);
    m_ArmLiftMotor.restoreFactoryDefaults();
    m_ArmLiftMotorPID = m_ArmLiftMotor.getPIDController();
    m_ArmLiftEncoder = m_ArmLiftMotor.getEncoder();

    //set ArmLift PID values
    m_ArmLiftMotorPID.setP(ArmConstants.kPArmLiftMotor);
    m_ArmLiftMotorPID.setI(ArmConstants.kIArmLiftMotor);
    m_ArmLiftMotorPID.setD(ArmConstants.kDArmLiftMotor);
    m_ArmLiftMotorPID.setIZone(ArmConstants.kIzArmLiftMotor);
    m_ArmLiftMotorPID.setFF(ArmConstants.kFFArmLiftMotor);
    m_ArmLiftMotorPID.setOutputRange(ArmConstants.kMinOutputArmLiftMotor,ArmConstants.kMaxOutputArmLiftMotor);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain Arm Lift", ArmConstants.kPArmLiftMotor);
    SmartDashboard.putNumber("I Gain Arm Lift", ArmConstants.kIArmLiftMotor);
    SmartDashboard.putNumber("D Gain Arm Lift", ArmConstants.kDArmLiftMotor);
    SmartDashboard.putNumber("I Zone Arm Lift", ArmConstants.kIzArmLiftMotor);
    SmartDashboard.putNumber("Feed Forward Arm Lift", ArmConstants.kFFArmLiftMotor);
    SmartDashboard.putNumber("Max Output Arm Lift", ArmConstants.kMinOutputArmLiftMotor);
    SmartDashboard.putNumber("Min Output Arm Lift", ArmConstants.kMaxOutputArmLiftMotor);
    SmartDashboard.putNumber("Set Rotations Arm Lift", 0);
    

    
      
  }

  @Override
  protected void useState(TrapezoidProfile.State setpoint) {
    // Use the computed profile state here.
    //Calculate feedForward from setpoint
    double feedForward = m_ArmLiftFF.calculate(setpoint.position, setpoint.velocity);

    //add feedForward to PID output to get motor output
    //m_ArmLiftMotorPID.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    m_ArmLiftMotorPID.setReference(feedForward, CANSparkMax.ControlType.kPosition);


  }
}
