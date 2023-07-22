package frc.robot.constants;

public class RotatorConstants {
  public static final int kRotatorMotorID = 0;// TODO: This needs to be changed
  public static final double kP = 1.0; 
  public static final double kI = 0.0; 
  public static final double kD = 0.0; 
  public static final double kGearRatio = 2; //TODO: This needs to be changed
  public static final double kEncoderCountToRotation = (1/2048)*kGearRatio; 
  public static final double kMinMotorOutput = -0.25; //TODO: This needs to be changed
  public static final double kMaxMotorOutput = 0.25; //TODO: This needs to be changed 
}
