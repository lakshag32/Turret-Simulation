// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controls.TurretControls;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.TurretRotator;

public class RobotContainer {
  private final TurretRotator m_turret;
  private final Flywheel m_flywheel; 
  private final TurretControls m_turretControls;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_turret = new TurretRotator(); 
    m_flywheel = new Flywheel(); 
    m_turretControls = new TurretControls(m_turret, m_flywheel); 
    m_turretControls.configureControls();

  }
}
