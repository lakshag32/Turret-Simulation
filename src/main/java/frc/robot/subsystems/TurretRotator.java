// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RotatorConstants;

public class TurretRotator extends SubsystemBase {
  
  private final WPI_TalonFX m_motor;
  private final PIDController m_controller; 
  private double m_motorOutput = 0.0; 

  public TurretRotator() {
    m_motor = new WPI_TalonFX(RotatorConstants.kRotatorMotorID);
    m_controller = new PIDController(RotatorConstants.kP, RotatorConstants.kI, RotatorConstants.kD); 
    SmartDashboard.putData(m_controller); 
  }

  @Override
  public void periodic() {
    moveMotorsSafelyWithPID();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setSetpoint(double desiredSetpoint){
    m_controller.reset();
    m_controller.setSetpoint(desiredSetpoint);

  }

  public void moveMotorsSafelyWithPID(){
    m_motorOutput = m_controller.calculate(encoderCountToRotations()); 
    MathUtil.clamp(m_motorOutput, RotatorConstants.kMinMotorOutput, RotatorConstants.kMaxMotorOutput); 
    m_motor.set(m_motorOutput); 
  }

  public double encoderCountToRotations(){
    return m_motor.getSelectedSensorPosition()*RotatorConstants.kEncoderCountToRotation; 

  } 
  
}
