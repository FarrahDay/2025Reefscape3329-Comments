package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private PhotonCamera camera;
  private double forward, strafe, turn, targetYaw, targetDistance, yawDifference;
  private static PIDController forwardPID, strafePID, turnPID;
  private static boolean targetVisible;
  private static PhotonTrackedTarget tag;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    camera = new PhotonCamera("Orange Camera");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    turnPID = new PIDController(Constants.VisionConstants.turnP,
                                Constants.VisionConstants.turnI,
                                Constants.VisionConstants.turnD);
    forwardPID = new PIDController(Constants.VisionConstants.forwardP, 
                                   Constants.VisionConstants.forwardI, 
                                   Constants.VisionConstants.forwardD);
    strafePID = new PIDController(Constants.VisionConstants.strafeP, 
                                  Constants.VisionConstants.strafeI, 
                                  Constants.VisionConstants.strafeD);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    forward = m_robotContainer.m_driverController.getLeftY();
    strafe = m_robotContainer.m_driverController.getLeftX();
    turn = m_robotContainer.m_driverController.getRightX();
    getVisionData();
    if(m_robotContainer.m_driverController.povLeft().getAsBoolean() && targetVisible){
      runTurn(0);
      runForwardOffsets(runForward(0.5), runStrafe(0));
      runStrafeOffsets(runForward(0.5), runStrafe(0));
    }
    SmartDashboard.putBoolean("Visible", targetVisible);
    SmartDashboard.putNumber("Target Distance", targetDistance);
    SmartDashboard.putNumber("Target Yaw", targetYaw);
    SmartDashboard.putNumber("Yaw Difference", yawDifference);
    SmartDashboard.putBoolean("Forward Goal", forwardPID.atSetpoint());
    SmartDashboard.putBoolean("Strafe Goal", strafePID.atSetpoint());
    SmartDashboard.putBoolean("Turn Goal", turnPID.atSetpoint());
    m_robotContainer.forward = this.forward;
    m_robotContainer.strafe = this.strafe;
    m_robotContainer.turn = this.turn;
  }
  
  public void getVisionData(){
    var results = camera.getAllUnreadResults();
    if(!results.isEmpty()){
      var result = results.get(results.size() - 1);
      if(result.hasTargets()){
        targetVisible = true;
        tag = result.getBestTarget();
        targetYaw = tag.getYaw();
        targetDistance = PhotonUtils.calculateDistanceToTargetMeters(Units.inchesToMeters(8.125), Units.inchesToMeters(13), Units.degreesToRadians(20), Units.degreesToRadians(tag.getPitch()));
        yawDifference = tag.getYaw() - m_robotContainer.drivebase.getGyro().getDegrees();
      }
      else{
        targetVisible = false;
      }
    }
  }

  public double runForward(double target){
    forwardPID.setSetpoint(target);
    return forwardPID.calculate(targetDistance);
  }

  public double runStrafe(double target){
    strafePID.setSetpoint(target);
    return strafePID.calculate(yawDifference);
  }

  public void runTurn(double target){
    turnPID.setSetpoint(target);
    turn = turnPID.calculate(targetYaw);
  }

  public void runForwardOffsets(double forward, double strafe){
    int tagID = tag.getFiducialId();
    if(tagID == 17){
      this.forward = strafe * Math.sin(60) + forward * Math.cos(60);
    }
    else if(tagID == 18){
      this.forward = forward;
    }
    else if(tagID == 19){
      this.forward = strafe * Math.sin(-60) + forward * Math.cos(-60);
    }
    else if(tagID == 20){
      this.forward = strafe * Math.sin(-120) + forward * Math.cos(-60);
    }
    else if (tagID == 21){
      this.forward = -forward;
    }
    else if(tagID == 22){
      this.forward = strafe * Math.sin(120) + this.forward * Math.cos(120);
    }
  }

  public void runStrafeOffsets(double forward, double strafe){
    int tagID = tag.getFiducialId();
    if(tagID == 17){
      this.strafe = strafe * Math.cos(60) - forward * Math.sin(60);
    }
    else if(tagID == 18){
      this.strafe = strafe;
    }
    else if(tagID == 19){
      this.strafe = strafe * Math.cos(-60) - forward * Math.sin(60);
    }
    else if(tagID == 20){
      this.strafe = strafe * Math.cos(-120) - forward * Math.sin(60);
    }
    else if(tagID == 21){
      this.strafe = -strafe;
    }
    else if(tagID == 22){
      this.strafe = strafe * Math.cos(120) - forward * Math.sin(120);
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
