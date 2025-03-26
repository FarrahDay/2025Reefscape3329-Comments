package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.05;     // just makes controllers less sensitive
  }

  public static class ElevatorConstants {
    public static final int rightID = 13;
    public static final int leftID = 14;
    public static final double maxVelocity = 0.3;
    public static final double maxAcceleration = 0.3;
    public static final double L1 = 3.5;     //about 1 inch per 0.7 revolutions     revolutions needed to get the elevator in position to score
    public static final double L2 = 14.45  - 0.7 * 1.5 - 1.4 - 2.1 - 0.75 * 0.7 - 2.1 - 0.7 * 2.5 + 0.7 * 0.25 - 0.7;
    public static final double L3 = 27.675 - 0.7 * 1.5 - 1.4 - 3.5 - 1.4 - 2.1;
    public static final double L4 = 43.05 - 0.7 + 0.35 - 1.4 + 0.7 + 0.35;
    public static final double A1 = 24.261 - 0.7 - 1.4;     // revolutions needed for algae collection heights
    public static final double A2 = 37.404 - 0.7 - 1.4;
    public static final double P = 5.881;     
    public static final double CS = 2.75;     // defines height for coral station configuration(?)
  }

  public static class CoralConstants {
    public static final int pivotID = 15;
    public static final int intakeID = 16;
    public static final int encoderID = 0;
    public static final double intakeSpeed = 0.3;
    public static final double ejectSpeed = 0.3;
    public static final double kP = 3.5;     // controls maximum speed motors can spin
    public static final double kI = 0;     // ?
    public static final double kD = 0.1;     // changes motor acceleration
    public static final double maxVelocity = 0.5;
    public static final double maxAcceleration = 0.2;
    public static final double L1 = 0.158 - 0.05;
    public static final double L2 = 0.025;
    public static final double L3 = 0.025;
    public static final double L4 = -0.025;
    public static final double A1 = 0.25;
    public static final double A2 = 0.25;
    public static final double P = 0.25;
    public static final double CS = 0.165;
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
    public static final double forwardP = 4;
    public static final double forwardI = 0;
    public static final double forwardD = 0;
    public static final double strafeP = 0;
    public static final double strafeI = 0;
    public static final double strafeD = 0;
    public static final double turnP = 0.02;
    public static final double turnI = 0;
    public static final double turnD = 0.004 / 10;
  }
  
  public static final double maxSpeed = Units.feetToMeters(10);
}
