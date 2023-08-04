// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FlywheelConstants;
import frc.robot.constants.RotatorConstants;

public class Flywheel extends SubsystemBase {
  
  private final WPI_TalonFX m_motor;
  private TalonFXSimCollection m_motorSim;
  private FlywheelSim m_flywheelSim; 
  private final PIDController m_controller; 
  private final BangBangController m_bangBangController; 
  private final SimpleMotorFeedforward m_feedForwardController; 
  private double m_ffValue = 0; 


  public Flywheel() {
    setFlywheelSetpoint(5000);

    m_motor = new WPI_TalonFX(RotatorConstants.kRotatorMotorID);
    m_motorSim = m_motor.getSimCollection(); 
    m_feedForwardController = new SimpleMotorFeedforward(FlywheelConstants.kFlywheelKs, FlywheelConstants.kFlywheelKv, FlywheelConstants.kFlywheelKa);

    m_controller = new PIDController(RotatorConstants.kP, RotatorConstants.kI, RotatorConstants.kD); 
    m_bangBangController = new BangBangController(); 

    
    SmartDashboard.putData("PID Controller", m_controller); 

    if(RobotBase.isSimulation()){
      m_motorSim = m_motor.getSimCollection(); 
      m_flywheelSim = new FlywheelSim(
        FlywheelConstants.m_gearbox, 
        FlywheelConstants.kFlywheelGearing, 
        FlywheelConstants.kFlywheelMomentOfInertia
      );
    }

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
    //moveFlywheelMotor();
    System.out.println("hello");
    // System.out.println(m_flywheelSim.getAngularVelocityRPM()); 
  }

  public void setFlywheelSetpoint(double setpointVelocityRPM){

    setpointVelocityRPM = MathUtil.clamp(setpointVelocityRPM, 0, FlywheelConstants.kMaxSetpointValue);
    m_ffValue = m_feedForwardController.calculate(setpointVelocityRPM);
    m_bangBangController.setSetpoint(setpointVelocityRPM);

    System.out.println(m_bangBangController.getSetpoint());
  } 

  public void moveFlywheelMotor(){
    double bangOutput = m_bangBangController.calculate(m_motor.getSelectedSensorVelocity()) * 12.0;
    m_motor.setVoltage(bangOutput + 0.9 * m_ffValue);
  }

}
