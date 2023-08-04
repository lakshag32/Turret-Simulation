package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class RotatorConstants {
  public static final int kRotatorMotorID = 0;// TODO: This needs to be changed
  public static final double kP = 1.0; 
  public static final double kI = 0.0; 
  public static final double kD = 0.0; 
  public static final double kGearRatio = 2; //TODO: This needs to be changed
  public static final double kEncoderCountToRotation = (1/2048)*kGearRatio; 
  public static final double kMinMotorOutput = -1; //TODO: This needs to be changed
  public static final double kMaxMotorOutput = 1; //TODO: This needs to be changed 
  public static final int motorPortSim = 0;
  public static final  DCMotor m_armGearbox = DCMotor.getVex775Pro(2);
  public static final double kGearing = (20.0/1.0) * (62.0/34.0) * (48.0/18.0);

  public static final double kArmLength = Units.inchesToMeters(16.1);
  
  //calculate MOI using the center of gravity distance and weight
  public static final double kCOGWeight = Units.lbsToKilograms(7.3);
  public static final double kCOGDistance = Units.inchesToMeters(8.11);
  
  /** Wrist moment of inertia represents how hard it is to angularly accelerate (ie spin) something. */
  public static final double kMomentOfInertia = kCOGWeight * kCOGDistance * kCOGDistance; // 0.1405

  /**
   * 
   */
  public static final double kMaxAngleRads = Double.POSITIVE_INFINITY;
  public static final double kMinAngleRads = Double.NEGATIVE_INFINITY;

  public static final double kEncoderTicksToRadsConversion = 2*Math.PI/2048;


}
