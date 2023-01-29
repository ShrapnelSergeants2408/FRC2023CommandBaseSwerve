// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;


public class Gripper extends SubsystemBase {
  /** Creates a new Gripper. */
  private final VictorSPX m_GripperExtensionMotor;
  private final DoubleSolenoid m_Gripper;
  
  private final AHRS m_GripperPosition;
  //private final Accelerometer m_GripperPosition;

  
  public Gripper() {

    m_GripperExtensionMotor = new VictorSPX(kGripperExtension);
    m_GripperExtensionMotor.setInverted(kGripperExtensionInverted);

    m_Gripper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,kGripperOpen, kGripperClosed);
    

    m_GripperPosition = new AHRS();
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("GripperYaw", m_GripperPosition.getYaw());
    SmartDashboard.putNumber("GripperPitch",m_GripperPosition.getPitch());
    SmartDashboard.putNumber("GripperRoll", m_GripperPosition.getRoll());
    SmartDashboard.putNumber("Gripper Angle", m_GripperPosition.getAngle());
    SmartDashboard.putNumber("Gripper RawX", m_GripperPosition.getRawAccelX());
    SmartDashboard.putNumber("Gripper RawY", m_GripperPosition.getRawAccelY());
    SmartDashboard.putNumber("Gripper RawZ", m_GripperPosition.getRawAccelZ());
    SmartDashboard.putNumber("Gripper RawMagX", m_GripperPosition.getRawMagX());
    SmartDashboard.putNumber("Gripper RawMagY", m_GripperPosition.getRawMagY());
    SmartDashboard.putNumber("Gripper RawMagZ", m_GripperPosition.getRawMagZ());

  }

}
