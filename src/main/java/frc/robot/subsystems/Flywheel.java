// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Flywheel sim code taken from: 

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FlywheelConstants;
import frc.robot.constants.RotatorConstants;

public class Flywheel extends SubsystemBase {
  
  private final WPI_TalonFX m_motor;
  private TalonFXSimCollection m_motorSim;
  private FlywheelSim m_flywheelSim; 
  private final PIDController m_controller; 
  private final BangBangController m_bangBangController; 
  private final SimpleMotorFeedforward m_ffController; 
  private double m_ffValue = 0; 


  public Flywheel(){

    m_motor = new WPI_TalonFX(RotatorConstants.kRotatorMotorID);
    m_ffController = new SimpleMotorFeedforward(FlywheelConstants.kFlywheelKs, FlywheelConstants.kFlywheelKv, FlywheelConstants.kFlywheelKa);
    m_controller = new PIDController(RotatorConstants.kP, RotatorConstants.kI, RotatorConstants.kD); 
    m_bangBangController = new BangBangController(); 

    if(RobotBase.isSimulation()){
      m_motorSim = m_motor.getSimCollection(); 
      m_flywheelSim = new FlywheelSim(
        FlywheelConstants.m_gearbox, 
        FlywheelConstants.kFlywheelGearing, 
        FlywheelConstants.kFlywheelMomentOfInertia
      );
    }
    setFlywheelSetpoint(0);


  }

  @Override
  public void periodic() {
    moveFlywheelMotor();
  }

  @Override
  public void simulationPeriodic() {
    m_flywheelSim.setInputVoltage(m_motor.get() * RobotController.getInputVoltage());
    m_flywheelSim.update(0.02);
    m_motorSim.setIntegratedSensorVelocity((int)m_flywheelSim.getAngularVelocityRadPerSec()); 
    moveFlywheelMotor();
    System.out.println(m_flywheelSim.getAngularVelocityRPM()); 
  }

  public void setFlywheelSetpoint(double setpointVelocityRPM){

    setpointVelocityRPM = MathUtil.clamp(setpointVelocityRPM, 0, FlywheelConstants.kMaxSetpointValue);
    m_ffValue = m_ffController.calculate(setpointVelocityRPM);
  } 

  public void moveFlywheelMotor(){
    m_ffValue = MathUtil.clamp(m_ffValue,-1,1); 
    m_motor.setVoltage(10*m_ffValue);
  }

}
