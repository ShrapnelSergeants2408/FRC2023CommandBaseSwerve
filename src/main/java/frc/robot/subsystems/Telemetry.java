// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telemetry extends SubsystemBase {
  /** Creates a new Telemetry. */
  PowerDistribution m_pdp = new PowerDistribution(0, ModuleType.kCTRE);

  public Telemetry() {}

  @Override
  public void periodic() {
    // Get the voltage going into the PDP, in Volts.
    // The PDP returns the voltage in increments of 0.05 Volts.
    double voltage = m_pdp.getVoltage();
    SmartDashboard.putNumber("Voltage", voltage);

    // Get the current in Amperes.
    // The PDP returns the current in increments of 0.125A.
    // At low currents the current readings tend to be less accurate.
    double current0 = m_pdp.getCurrent(0);
    SmartDashboard.putNumber("Current Channel 0", current0);

    double current1 = m_pdp.getCurrent(1);
    SmartDashboard.putNumber("Current Channel 1", current1);

    double current2 = m_pdp.getCurrent(2);
    SmartDashboard.putNumber("Current Channel 2", current2);

    double current3 = m_pdp.getCurrent(3);
    SmartDashboard.putNumber("Current Channel 3", current3);

    double current4 = m_pdp.getCurrent(4);
    SmartDashboard.putNumber("Current Channel 4", current4);

    double current5 = m_pdp.getCurrent(5);
    SmartDashboard.putNumber("Current Channel 5", current5);

    double current6 = m_pdp.getCurrent(6);
    SmartDashboard.putNumber("Current Channel 6", current6);

    double current7 = m_pdp.getCurrent(7);
    SmartDashboard.putNumber("Current Channel 7", current7);

    double current8 = m_pdp.getCurrent(8);
    SmartDashboard.putNumber("Current Channel 8", current8);

    double current9 = m_pdp.getCurrent(9);
    SmartDashboard.putNumber("Current Channel 9", current9);

    double current10 = m_pdp.getCurrent(10);
    SmartDashboard.putNumber("Current Channel 10", current10);

    double current11 = m_pdp.getCurrent(11);
    SmartDashboard.putNumber("Current Channel 11", current11);

    double current12 = m_pdp.getCurrent(12);
    SmartDashboard.putNumber("Current Channel 12", current12);

    double current13 = m_pdp.getCurrent(13);
    SmartDashboard.putNumber("Current Channel 13", current13);

    double current14 = m_pdp.getCurrent(14);
    SmartDashboard.putNumber("Current Channel 14", current14);

    double current15 = m_pdp.getCurrent(15);
    SmartDashboard.putNumber("Current Channel 15", current15);





    
  }
}

