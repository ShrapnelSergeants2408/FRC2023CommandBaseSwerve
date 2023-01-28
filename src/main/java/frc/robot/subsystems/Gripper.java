// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;

public class Gripper extends SubsystemBase {
  /** Creates a new Gripper. */
  private final VictorSPX m_GripperExtensionMotor;
  private final DoubleSolenoid m_Gripper;
  
  private final Accelerometer m_GripperPosition;
  
  public Gripper() {

    m_GripperExtensionMotor = new VictorSPX(kGripperExtension);
    m_Gripper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,kGripperOpen, kGripperClosed);

    m_GripperPosition = new Accelerometer(kGripperPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
