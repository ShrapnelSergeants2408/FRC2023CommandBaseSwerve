// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {

  private final CANSparkMax m_ArmLiftMotor;
  private final VictorSPX m_ArmExtensionMotor;

  //private final RelativeEncoder m_ArmLiftEncoder;
  private final RelativeEncoder m_ArmLiftEncoder;

  //add sensors to determine arm extension position
  //limit switch?  encoder on motor? ultrasonic rangefinder?
  private final Ultrasonic m_ArmExtensionDistance;

  public Arm() {
    m_ArmLiftMotor = new CANSparkMax(kArmExtensionMotor, MotorType.kBrushless);
    m_ArmLiftMotor.setInverted(kArmLiftMotorInverted);
    m_ArmLiftEncoder = m_ArmLiftMotor.getEncoder();

    m_ArmExtensionMotor = new VictorSPX(kArmExtensionMotor);
    m_ArmExtensionMotor.setInverted(kArmExtensionMotorInverted);

    m_ArmExtensionDistance = new Ultrasonic(kUltrasonicPingChannel,kUltrasonicEchoChannel);

  }

/*  speed controllers, encoders, ultrasonic sensor defined
 * 
 * TODO: need to write the rest of this code
 */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
