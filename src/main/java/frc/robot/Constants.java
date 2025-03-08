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
    public static final double L1 = 3.5;
    public static final double L2 = 12.7;
    public static final double L3 = 26.1;
    public static final double L4 = 43.05;
    public static final double A1 = 24.261;
    public static final double A2 = 37.404;
    public static final double P = 5.881;
    public static final double CS = 9.5
    ;
  }

  public static class CoralConstants {
    public static final int pivotID = 15;
    public static final int intakeID = 16;
    public static final int encoderID = 0;
    public static final int limitSwitchID = 6;
    public static final double testAngle = 45;
    public static final double intakeSpeed = 0.3;
    public static final double ejectSpeed = 0.3;
    public static final double kP = 4;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double maxVelocity = 0.4;
    public static final double maxAcceleration = 0.1;
    public static final double L1 = 0.158;
    public static final double L2 = 0;
    public static final double L3 = 0;
    public static final double L4 = 0.115;
    public static final double A1 = 0.25;
    public static final double A2 = 0.25;
    public static final double P = 0.25;
    public static final double CS = 0.174;
  }

  public static class AlgaeConstants {
    public static final int pivotID = 17;
    public static final int intakeID = 18;
    public static final int encoderID = 1;
    public static final double intakeSpeed = 4;
    public static final double ejectSpeed = 0.8;
    public static final double kP = 6;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double maxVelocity = 0.3;
    public static final double maxAcceleration = 0.15;
    public static final double L1 = 0.233;
    public static final double L2 = 0.233;
    public static final double L3 = 0.233;
    public static final double L4 = 0.233;
    public static final double A1 = 0;
    public static final double A2 = 0;
    public static final double P = 0;
    public static final double CS = 0.233;
  }


  
  public static final double maxSpeed = Units.feetToMeters(10);
}
