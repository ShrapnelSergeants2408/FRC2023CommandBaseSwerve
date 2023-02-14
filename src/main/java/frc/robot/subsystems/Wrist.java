// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogGyro;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;


public class Wrist extends SubsystemBase {
  /** Creates a new Gripper. */
  private final VictorSPX m_WristMotor;
  private final PIDController m_WristMotorPID;
  
  private final AnalogGyro m_WristPosition;
  //private final double m_WristPositionOffset;

  public enum wristMode {
      STOW,
      DEPLOY,
  };  

 
  public Wrist() {

    m_WristMotor = new VictorSPX(kWristMotor);
    m_WristMotor.setInverted(kWristMotorInverted);
    m_WristMotorPID = new PIDController(
                    kPWristMotor,
                    kIWristMotor, 
                    kDWristMotor);

    m_WristPosition = new AnalogGyro(getChannelFromPin(PinType.AnalogIn, kWristPositionPort));  
    //wait 1 sec to calibrate gyro
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        m_WristPosition.reset();
      } catch (Exception e) {
      }
    }).start();

  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Angle", m_WristPosition.getAngle());
    SmartDashboard.putNumber("Wrist Offset", m_WristPosition.getOffset());


  }

  public PIDController setWristPID(){
    return m_WristMotorPID;
  }

  public double getWristPosition(){
    return m_WristPosition.getAngle();
  }

  public void setWristMotor(double speed){  //TODO: change setpoint value to variable?
    m_WristMotor.set(VictorSPXControlMode.PercentOutput, speed);
    //do something PID here
  }

  public void stopWrist() {
    m_WristMotor.set(VictorSPXControlMode.PercentOutput,0);
  }


  //navX expansion stuff
  public enum PinType { DigitalIO, PWM, AnalogIn, AnalogOut };
    
  public final int MAX_NAVX_MXP_DIGIO_PIN_NUMBER      = 9;
  public final int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER   = 3;
  public final int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER  = 1;
  public final int NUM_ROBORIO_ONBOARD_DIGIO_PINS     = 10;
  public final int NUM_ROBORIO_ONBOARD_PWM_PINS       = 10;
  public final int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS  = 4;
    
  /* getChannelFromPin( PinType, int ) - converts from a navX-MXP */
  /* Pin type and number to the corresponding RoboRIO Channel     */
  /* Number, which is used by the WPI Library functions.          */
    
  public int getChannelFromPin( PinType type, int io_pin_number ) 
             throws IllegalArgumentException {
      int roborio_channel = 0;
      if ( io_pin_number < 0 ) {
          throw new IllegalArgumentException("Error:  navX-MXP I/O Pin #");
      }
      switch ( type ) {
      case DigitalIO:
          if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {
              throw new IllegalArgumentException("Error:  Invalid navX-MXP Digital I/O Pin #");
          }
          roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_DIGIO_PINS + 
                            (io_pin_number > 3 ? 4 : 0);
          break;
      case PWM:
          if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {
              throw new IllegalArgumentException("Error:  Invalid navX-MXP Digital I/O Pin #");
          }
          roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_PWM_PINS;
          break;
      case AnalogIn:
          if ( io_pin_number > MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER ) {
              throw new IllegalArgumentException("Error:  Invalid navX-MXP Analog Input Pin #");
          }
          roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_ANALOGIN_PINS;
          break;
      case AnalogOut:
          if ( io_pin_number > MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER ) {
              throw new IllegalArgumentException("Error:  Invalid navX-MXP Analog Output Pin #");
          }
          roborio_channel = io_pin_number;            
          break;
      }
        return roborio_channel;
  }
}
