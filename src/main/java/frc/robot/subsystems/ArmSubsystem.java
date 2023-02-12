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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  //private final VictorSPX m_ArmExtensionMotor;
  private final AnalogInput m_ArmExtensionRangefinder;
  

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

    //rangefinder
    m_ArmExtensionRangefinder = new AnalogInput(getChannelFromPin(PinType.AnalogIn, ArmConstants.kArmExtensionRangefinderPort));  

    
      
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

  
  public Command setArmGoalCommand(double kArmOffsetRads) {

    return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
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
