// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Commands.HangerMove;
import frc.robot.Commands.IntakeMove;
import frc.robot.Commands.ShooterMove;
import frc.robot.Commands.Auto.ShootNoteAuto;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  private final ShooterSubsystem m_robotShooter = new ShooterSubsystem();
  private final HangerSubsystem m_robotHanger = new HangerSubsystem();
  private final VisionSubsystem m_robotVision = new VisionSubsystem();
  private final LightSubsystem m_robotLight = new LightSubsystem();

  public Object drive;

  //Creates the pdh for pdh widget
  private PowerDistribution m_pdh = new PowerDistribution();

  //Creates the field for field wiget
  Field2d field = new Field2d();

  //Creates the auto chooser
  private SendableChooser<Command> pathPlannerChooser;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // The subsystem's controller
  XboxController m_subsystemController = new XboxController(OIConstants.kSubsystemsControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() { 
    
    // Configure path planner
    configurePathPlanner();

    // Configure shuffleboard widgets
    configureShuffleboardWidgets();

    // Configure the button bindings
    configureButtonBindings();
  
    // Configure defualt commands
    configureDefaultCommands();
  }

  public void periodic() {
    field.setRobotPose(m_robotDrive.getPose());
    Shuffleboard.update();
  }

  /**
   * This method is used to configure path planner with your robot
   */
  private void configurePathPlanner() {

    NamedCommands.registerCommand("ShootNoteAuto", new ShootNoteAuto(m_robotIntake, m_robotShooter));
    NamedCommands.registerCommand("IntakeOut", new InstantCommand(() -> m_robotIntake.setArmPosition(1)));
    NamedCommands.registerCommand("IntakeIn", new InstantCommand(() -> m_robotIntake.setArmPosition(0)));

    HolonomicPathFollowerConfig pathConfig = new HolonomicPathFollowerConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kDistanceToFarthestModuleMeters,
      new ReplanningConfig(false, false));

    AutoBuilder.configureHolonomic(
      m_robotDrive::getPose,
      m_robotDrive::resetOdometry,
      m_robotDrive::getCurrentspeeds,
      m_robotDrive::setCurrentspeeds,
      pathConfig,
      () ->   {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Blue;
        }
        return false;},
      m_robotDrive);

    pathPlannerChooser = AutoBuilder.buildAutoChooser();
  }

  /**
   * Use this method to difine your shuffleboard widgets.
   * Widgets can be created by geting your {@link ShuffleboardTab}
   * Then calling {@link ShuffleboardTab.add} then adding properties
   */
  private void configureShuffleboardWidgets() {

  // Creates the tabs in shuffleboard
  ShuffleboardTab PreGameTab = Shuffleboard.getTab(ShuffleboardConstants.kPreGameTabName);
  ShuffleboardTab AutoTab = Shuffleboard.getTab(ShuffleboardConstants.kAutoTabName);
  ShuffleboardTab TeleopTab = Shuffleboard.getTab(ShuffleboardConstants.kTeleopTabName);

  // Creates widget for the auto selector
  PreGameTab
    .add("Path Auto", pathPlannerChooser)
    .withWidget(BuiltInWidgets.kComboBoxChooser)
    .withSize(3, 2)
    .withPosition(0, 0); 

    // Creates widget for the rev board
  TeleopTab
    .add("Power",m_pdh)
    .withWidget(BuiltInWidgets.kPowerDistribution)
    .withPosition(10, 0);

    // Creates widget for the field
  TeleopTab
    .add("Field", field)
    .withWidget(BuiltInWidgets.kField)
    .withSize(6, 3)
    .withPosition(7, 3);

  // Creates widget for shoot note auto command
  TeleopTab
    .add("Shoot note", new ShootNoteAuto(m_robotIntake, m_robotShooter))
    .withWidget(BuiltInWidgets.kCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Create X stance button
    new JoystickButton(m_driverController, Button.kRightBumper.value)
      .whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive));

    new JoystickButton(m_driverController, 1)
      .toggleOnTrue(new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
          false,
          true),
          m_robotDrive));

    new JoystickButton(m_driverController, 1)
      .toggleOnTrue(new InstantCommand(
        () -> m_robotDrive.isFieldRelitive(false)));
    
    new JoystickButton(m_driverController, 1)
      .toggleOnFalse(new InstantCommand(
        () -> m_robotDrive.isFieldRelitive(true)));

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
      .whileTrue(new RunCommand(
        () -> m_robotDrive.zeroHeading(),
        m_robotDrive));

    new JoystickButton(m_subsystemController, Button.kA.value)
      .toggleOnTrue(new InstantCommand(() -> m_robotIntake.setArmPosition(1)));

    new JoystickButton(m_subsystemController, Button.kA.value)
      .toggleOnFalse
      (new InstantCommand(() -> m_robotIntake.setArmPosition(0)));

    new JoystickButton(m_subsystemController, Button.kX.value)
      .onTrue
        (new InstantCommand(() -> m_robotShooter.setShooterPosition(0.18)));

      new JoystickButton(m_subsystemController, Button.kB.value)
      .onTrue
        (new InstantCommand(() -> m_robotShooter.setShooterPosition(0.30)));

    new JoystickButton(m_subsystemController, Button.kY.value)
      .onTrue
        (new InstantCommand(() -> m_robotShooter.setShooterPosition(0)));
    
    new JoystickButton(m_subsystemController, Button.kStart.value)
      .whileTrue(new HangerMove(m_robotHanger, m_subsystemController));
  }

  private void configureDefaultCommands() {

    // sets drive default command
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
          true,
          true),
          m_robotDrive));
    
    // sets intake default command
    m_robotIntake.setDefaultCommand(new IntakeMove(m_robotIntake, m_subsystemController));
    
     // sets the shooters defult command
    m_robotShooter.setDefaultCommand(new ShooterMove(m_robotShooter, m_subsystemController));
    
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {  
    return pathPlannerChooser.getSelected();
  }
}
