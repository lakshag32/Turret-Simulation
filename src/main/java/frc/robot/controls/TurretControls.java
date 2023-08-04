package frc.robot.controls;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.TurretRotator;

public class TurretControls {

  /**
   * Below are instance variables. Think of them as the specific variables that we are going to need for THIS class
   * 
   * m_wrist is a variable of the type WristSubsystem 
   * m_button1, m_button1, and m_button3 are variables of the type Trigger
   * m_joy is a variable of the type Joystick
  */

  TurretRotator m_rotator; 
  Trigger m_button1;
  Trigger m_button2;
  Trigger m_button3;
  Trigger m_button4;
  Joystick m_joy; 

  /**
   * This is the constructor for the WristControls class. As a parameter, it requires a WristSubsystem. 
   * 
   * <p>
   * 
   * We will pass a WristSubsystem object to it in RobotContainer when we create an object of this class.  
   * 
   * <p> 
   * 
   * By assiging m_wrist to wristSubsystem, it now holds a WristSubsystem object. Now we can access the
   * various methods we made in the WristSubsystem.java file from the m_wrist instance variable. 
   * 
   * <p>
   * 
   * For the other instance variables, we can just turn them directly into objects by assigning them to an instance of the class
   * by doing m_instanceVariable = new Class();
   * 
   * <p>
   * 
   * @param wristSubsystem
   */
  public TurretControls(TurretRotator rotator){
    
    m_rotator = rotator; 
    m_joy = new Joystick(0); 
    m_button1 = new JoystickButton(m_joy, 1);
    m_button2 = new JoystickButton(m_joy, 2);
    m_button3 = new JoystickButton(m_joy,3);
    m_button4 = new JoystickButton(m_joy,4);


  }


  /**
   * The configureControls() method binds the MoveWrist() command to the pressing of different buttons
   */
  public void configureControls(){

    /**
     * When a button is pressed run the MoveWrist command. 
     * Give the command the WristSubsystem(stored in m_wrist instance variable) and a desired setpoint to move the wrist too
     */
    m_button1.onTrue(new InstantCommand(()->m_rotator.setSetpoint(0))); 
    m_button2.onTrue(new InstantCommand(()->m_rotator.setSetpoint(Math.PI/2))); 
    m_button3.onTrue(new InstantCommand(()->m_rotator.setSetpoint(Math.PI))); 
    m_button4.onTrue(new InstantCommand(()->m_rotator.setSetpoint(-Math.PI/2))); 


    /**
     * When a button is pressed an InstantCommand() is created that, on button press,
     * sets the desired setpoint in the WristSubsystem using the setSetpoint() method declared there 
     */

    // m_button1.onTrue(new InstantCommand(()-> m_wrist.setSetpoint(-50)));
    // m_button2.onTrue(new InstantCommand(()-> m_wrist.setSetpoint(75)));
    // m_button3.onTrue(new InstantCommand(()-> m_wrist.setSetpoint(200)));

    

  }

  public double getJoyVal(){
    return m_joy.getRawAxis(0); 
  }
}
