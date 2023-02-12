// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final CANSparkMax m_ArmLiftMotor;
  private final SparkMaxPIDController m_ArmLiftMotorPID;
  private final RelativeEncoder m_ArmLiftEncoder;

  private final VictorSPX m_ArmExtensionMotor;
  private final PIDController m_ArmExtensionMotorPID;
  private final AnalogInput m_ArmExtensionRangefinder;


  
  public Arm() {
    //m_ArmLiftMotor = new CANSparkMax(ArmConstants.kArmLiftMotor, MotorType.kBrushless);
    m_ArmLiftMotor = new CANSparkMax(ArmConstants.kArmLiftMotor, MotorType.kBrushless);
    m_ArmLiftMotor.restoreFactoryDefaults();
    m_ArmLiftMotorPID = m_ArmLiftMotor.getPIDController();
    m_ArmLiftEncoder = m_ArmLiftMotor.getEncoder();
    m_ArmLiftMotor.setInverted(ArmConstants.kArmExtensionMotorInverted);  

    m_ArmExtensionMotor = new VictorSPX(ArmConstants.kArmExtensionMotor);
    m_ArmExtensionMotor.setInverted(ArmConstants.kArmExtensionMotorInverted);
    m_ArmExtensionMotorPID = new PIDController(ArmConstants.kPArmExtensionMotor,
                                               ArmConstants.kIArmExtensionMotor, 
                                               ArmConstants.kDArmExtensionMotor);
    m_ArmExtensionRangefinder = new AnalogInput(ArmConstants.kArmExtensionRangefinderPort);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
