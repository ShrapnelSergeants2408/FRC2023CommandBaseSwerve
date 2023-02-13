// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import static frc.robot.Constants.ArmConstants.*;


public class Gripper extends SubsystemBase {
  /** Creates a new Gripper. */
  private final VictorSPX m_WristMotor;
  private final PIDController m_WristMotorPID;
  
  private final AnalogGyro m_WristPosition;
  private final double m_WristPositionOffset;

  private final DoubleSolenoid m_Gripper;


  
  public Gripper() {

    m_WristMotor = new VictorSPX(ArmConstants.kWristMotor);
    m_WristMotor.setInverted(ArmConstants.kWristMotorInverted);
    m_WristMotorPID = new PIDController(
                    kPWristMotor,
                    kIWristMotor, 
                    kDWristMotor);

    m_WristPosition = new AnalogGyro(ArmConstants.kWristPositionPort);
    m_WristPosition.reset();
    m_WristPositionOffset = m_WristPosition.getAngle();


    m_Gripper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,kGripperOpen, kGripperClosed);
    


  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("GripperYaw", m_WristPosition.getYaw());
    //SmartDashboard.putNumber("GripperPitch",m_WristPosition.getPitch());
    //SmartDashboard.putNumber("GripperRoll", m_WristPosition.getRoll());
    //SmartDashboard.putNumber("Gripper Angle", m_WristPosition.getAngle());
    //SmartDashboard.putNumber("Gripper RawX", m_WristPosition.getRawAccelX());
    //SmartDashboard.putNumber("Gripper RawY", m_WristPosition.getRawAccelY());
    //SmartDashboard.putNumber("Gripper RawZ", m_WristPosition.getRawAccelZ());
    //SmartDashboard.putNumber("Gripper RawMagX", m_WristPosition.getRawMagX());
    //SmartDashboard.putNumber("Gripper RawMagY", m_WristPosition.getRawMagY());
    //SmartDashboard.putNumber("Gripper RawMagZ", m_WristPosition.getRawMagZ());

  }

  public void grabPiece(){

    m_Gripper.set(Value.kForward);

  }

  public void releasePiece(){

    m_Gripper.set(Value.kReverse);

  }
}
