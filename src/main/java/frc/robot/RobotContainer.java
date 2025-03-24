package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Configs.*;
import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  public final SwerveSubsystem drivebase = new SwerveSubsystem();
  private Elevator m_Elevator = new Elevator();
  private Coral m_Coral = new Coral();
  private Algae m_Algae = new Algae(m_Elevator);
  public final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  public double forward = 0;
  public double strafe = 0;
  public double turn = 0;
  public PathPlannerAuto stay = new PathPlannerAuto("Stay");

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous Chooser", autoChooser);
    //NamedCommands.registerCommand("L1Config", new L1Config(m_Elevator, m_Coral, m_Algae));
    NamedCommands.registerCommand("L2Config", new L2Config(m_Elevator, m_Coral, m_Algae));
    //NamedCommands.registerCommand("L3Config", new L3Config(m_Elevator, m_Coral, m_Algae));
    //NamedCommands.registerCommand("L4Config", new L4Config(m_Elevator, m_Coral, m_Algae));
    //NamedCommands.registerCommand("CSConfig", new CSConfig(m_Elevator, m_Coral, m_Algae));
    //NamedCommands.registerCommand("IntakeCoral", m_Coral.intakeCoralCommand());
    NamedCommands.registerCommand("EjectCoral", m_Coral.ejectCoralCommand());
    setMotorBrake(true);
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    configureBindings();
    autoChooser.setDefaultOption("None", null);
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), () -> forward, () -> strafe)
      .withControllerRotationAxis(() -> -turn).deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
      .allianceRelativeControl(true);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  private void configureBindings() {
    m_operatorController.povLeft().onTrue(new L1Config(m_Elevator, m_Coral, m_Algae));
    m_operatorController.povDown().onTrue(new L2Config(m_Elevator, m_Coral, m_Algae));
    m_operatorController.povUp().onTrue(new L3Config(m_Elevator, m_Coral, m_Algae));
    m_operatorController.povRight().onTrue(new L4Config(m_Elevator, m_Coral, m_Algae));
    m_operatorController.a().onTrue(new CSConfig(m_Elevator, m_Coral, m_Algae));
    m_operatorController.leftBumper().whileTrue(m_Coral.intakeCoralCommand());
    m_driverController.rightTrigger().whileTrue(m_Coral.ejectCoralCommand());

    m_operatorController.b().onTrue(new A1Config(m_Elevator, m_Coral, m_Algae));
    m_operatorController.y().onTrue(new A2Config(m_Elevator, m_Coral, m_Algae));
    m_operatorController.x().onTrue(new PConfig(m_Elevator, m_Coral, m_Algae));
    m_operatorController.leftTrigger().whileTrue(m_Algae.intakeAlgaeCommand());
    m_operatorController.rightTrigger().whileTrue(m_Algae.ejectAlgaeCommand());

    m_driverController.x().onTrue(drivebase.zeroGyro());
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
