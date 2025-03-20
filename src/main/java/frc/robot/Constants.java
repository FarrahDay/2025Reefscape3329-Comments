package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.05;
  }

  public static class ElevatorConstants {
    public static final int rightID = 13;
    public static final int leftID = 14;
    public static final double maxVelocity = 0.3;
    public static final double maxAcceleration = 0.3;
    public static final double L1 = 3.5;     //about 1 inch per 0.7 revolutions
    public static final double L2 = 14.45  - 0.7 * 1.5 - 1.4 - 2.1 - 0.75 * 0.7;
    public static final double L3 = 27.675 - 0.7 * 1.5 - 1.4 - 3.5;
    public static final double L4 = 43.05;
    public static final double A1 = 24.261 - 0.7 - 1.4;
    public static final double A2 = 37.404 - 0.7 - 1.4;
    public static final double P = 5.881;
    public static final double CS = 9.85 - 3.5 - 1.4;
  }

  public static class CoralConstants {
    public static final int pivotID = 15;
    public static final int intakeID = 16;
    public static final int encoderID = 0;
    public static final double intakeSpeed = 0.3;
    public static final double ejectSpeed = 0.3;
    public static final double kP = 3.2;
    public static final double kI = 0;
    public static final double kD = 0.1;
    public static final double maxVelocity = 0.5;
    public static final double maxAcceleration = 0.2;
    public static final double L1 = 0.158 - 0.05;
    public static final double L2 = 0.025;
    public static final double L3 = 0.025;
    public static final double L4 = 0;
    public static final double A1 = 0.25;
    public static final double A2 = 0.25;
    public static final double P = 0.25;
    public static final double CS = 0.184;
  }

  public static class AlgaeConstants {
    public static final int pivotID = 17;
    public static final int intakeID = 18;
    public static final int encoderID = 1;
    public static final double intakeSpeed = 4;
    public static final double ejectSpeed = 0.8;
    public static final double holdSpeed = 0.5;
    public static final double kP = 9;
    public static final double kI = 0;
    public static final double kD = 0.1 + (0.125 - 0.1) / 2;
    public static final double maxVelocity = 0.4;
    public static final double maxAcceleration = 0.25;
    public static final double L1 = 0.233;
    public static final double L2 = 0.233;
    public static final double L3 = 0.233;
    public static final double L4 = 0.233;
    public static final double A1 = 0;
    public static final double A2 = 0;
    public static final double P = 0;
    public static final double CS = 0.233;
  }

  public static class VisionConstants{
    public static final double turnP = 0.05;
    public static final double turnI = 0;
    public static final double turnD = 0.004;
  }
  
  public static final double maxSpeed = Units.feetToMeters(10);
}
