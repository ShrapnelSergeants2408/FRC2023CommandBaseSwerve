// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final CANSparkMax m_ArmLiftMotor;
  private final PIDController m_ArmLiftMotorPID;
  private final RelativeEncoder m_ArmLiftEncoder;

  private final VictorSPX m_ArmExtensionMotor;
  private final PIDController m_ArmExtensionMotorPID;
  private final AnalogInput m_ArmExtensionRangefinder;

  private double m_ArmExtensionDistance;
  private double m_ArmLiftPosition;
  private double m_ArmExtensionVoltageScaleFactor;


  
  public Arm() {
    m_ArmLiftMotor = new CANSparkMax(ArmConstants.kArmLiftMotor, MotorType.kBrushless);
    m_ArmLiftMotor.restoreFactoryDefaults();  //set initial arm position
    m_ArmLiftMotorPID = new PIDController(ArmConstants.kPArmLiftMotor,
                                          ArmConstants.kIArmLiftMotor,
                                          ArmConstants.kDArmLiftMotor);
    m_ArmLiftEncoder = m_ArmLiftMotor.getEncoder();
    m_ArmLiftEncoder.setPositionConversionFactor(ArmConstants.kArmLiftPositionConversionFactor);

    m_ArmLiftMotor.setInverted(ArmConstants.kArmExtensionMotorInverted);  

    m_ArmExtensionMotor = new VictorSPX(ArmConstants.kArmExtensionMotor);
    m_ArmExtensionMotor.setInverted(ArmConstants.kArmExtensionMotorInverted);
    m_ArmExtensionMotorPID = new PIDController(ArmConstants.kPArmExtensionMotor,
                                               ArmConstants.kIArmExtensionMotor, 
                                               ArmConstants.kDArmExtensionMotor);
    m_ArmExtensionRangefinder = new AnalogInput(getChannelFromPin(PinType.AnalogIn,ArmConstants.kArmExtensionRangefinderPort));

    SmartDashboard.putNumber("P Gain Arm Lift", ArmConstants.kPArmLiftMotor);
    SmartDashboard.putNumber("I Gain Arm Lift", ArmConstants.kIArmLiftMotor);
    SmartDashboard.putNumber("D Gain Arm Lift", ArmConstants.kDArmLiftMotor);
    //SmartDashboard.putNumber("I Zone Arm Lift", ArmConstants.kIzArmLiftMotor);
    //SmartDashboard.putNumber("Feed Forward Arm Lift", ArmConstants.kFFArmLiftMotor);
    SmartDashboard.putNumber("Min Output Arm Lift", ArmConstants.kMinOutputArmLiftMotor);
    SmartDashboard.putNumber("Max Output Arm Lift", ArmConstants.kMaxOutputArmLiftMotor);

    SmartDashboard.putNumber("P Gain Arm Extension", ArmConstants.kPArmExtensionMotor);
    SmartDashboard.putNumber("I Gain Arm Extension", ArmConstants.kIArmExtensionMotor);
    SmartDashboard.putNumber("D Gain Arm Extension", ArmConstants.kDArmExtensionMotor);
    //SmartDashboard.putNumber("I Zone Arm Extension", ArmConstants.kIzArmExtensionMotor);
    //SmartDashboard.putNumber("Feed Forward Arm Extension", ArmConstants.kFFArmExtensionMotor);
    SmartDashboard.putNumber("Min Output Arm Extension", ArmConstants.kMinOutputArmExtensionMotor);
    SmartDashboard.putNumber("Max Output Arm Extension", ArmConstants.kMaxOutputArmExtensionMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_ArmLiftPosition = m_ArmLiftEncoder.getPosition();

    m_ArmExtensionVoltageScaleFactor = RobotController.getVoltage5V(); //compensate for supply volage differences
    m_ArmExtensionDistance = m_ArmExtensionRangefinder.getValue()*m_ArmExtensionVoltageScaleFactor*0.0492;

    SmartDashboard.putNumber("Arm Lift Position ", m_ArmLiftPosition);
    SmartDashboard.putNumber("Arm Extension",m_ArmExtensionDistance);

  }

  public PIDController setArmLiftPID(){
    return m_ArmLiftMotorPID;
  }

  public PIDController setArmExtensionPID(){
    return m_ArmExtensionMotorPID;
  }

  public void armMovement(double armLift, double armExtend){
    m_ArmLiftMotor.set(armLift);
    m_ArmExtensionMotor.set(VictorSPXControlMode.PercentOutput, armExtend);
  }

  public double getArmLiftMeasurement(){
    return m_ArmLiftPosition;
  }
  
  public double getArmExtensionMeasurement(){
    return m_ArmExtensionDistance;
  }

  public void useArmLiftOutput(){
    //do something PID here
  }

  public void useArmExtensionOutput(){
    //do something PID here
  }

  public void stopArm() {
    m_ArmLiftMotor.set(0);
    m_ArmExtensionMotor.set(VictorSPXControlMode.PercentOutput,0);
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
