package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class FlywheelConstants {
  public static final int kFlywheelMotorID = 0;// TODO: This needs to be changed
  public static final DCMotor m_gearbox = DCMotor.getFalcon500(1);
  public static final double kMaxSetpointValue = 6000;
    // Gains are for example purposes only - must be determined for your own robot!
  public static final double kFlywheelKs = 0.0001; // V
  public static final double kFlywheelKv = 0.000195; // V/RPM
  public static final double kFlywheelKa = 0.0003; // V/(RPM/s)


  public static final double kFlywheelGearing = 1.0;

  // 1/2 MR²
  public static final double kFlywheelMomentOfInertia =
      0.5 * Units.lbsToKilograms(1.5) * Math.pow(Units.inchesToMeters(4), 2);

}
