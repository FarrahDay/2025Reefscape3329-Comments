package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.cameraserver.CameraServer;
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
  private double forward;
  private double strafe;
  private double turn;
  private boolean targetVisible;
  private static PIDController turnPID;

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
    double turnTarget = 0;
    turnPID = new PIDController(Constants.VisionConstants.turnP,
                                  Constants.VisionConstants.turnI,
                                  Constants.VisionConstants.turnD);
    setTargetTurn(turnTarget);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  public void setTargetTurn(double target){
    turnPID.setSetpoint(target);
  }

  @Override
  public void teleopPeriodic() {
    forward = m_robotContainer.m_driverController.getLeftY();
    strafe = m_robotContainer.m_driverController.getLeftX();
    turn = m_robotContainer.m_driverController.getRightX();
    double targetYaw = 0.0;
    var results = camera.getAllUnreadResults();
    if(!results.isEmpty()){
      var result = results.get(results.size() - 1);
      if(result.hasTargets()){
        for(var target : result.getTargets()){
          if(target.getFiducialId() == 8){
            targetYaw = target.getYaw();
            targetVisible = true;
          }
        }
      }
    }
    else{
      targetVisible = false;
    }
    m_robotContainer.forward = -this.forward;
    m_robotContainer.strafe = -this.strafe;
    if(m_robotContainer.m_driverController.a().getAsBoolean() && targetVisible){
      turn = -turnPID.calculate(targetYaw);
    }
    else if(m_robotContainer.m_driverController.b().getAsBoolean() && targetVisible){
      turn = turnPID.calculate(0);
    }
    m_robotContainer.turn = this.turn;
    SmartDashboard.putBoolean("Visible", targetVisible);
    SmartDashboard.putNumber("Target Yaw", targetYaw);
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
