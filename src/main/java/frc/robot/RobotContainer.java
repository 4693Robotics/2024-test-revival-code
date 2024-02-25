// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Commands.BumpIntake;
import frc.robot.Commands.IntakeMove;
import frc.robot.Commands.IntakeOut;
import frc.robot.Commands.LoadNote;
import frc.robot.Commands.MoveToTagPosition;
import frc.robot.Commands.ShootIntakeAmp;
import frc.robot.Commands.ShootNote;
import frc.robot.Commands.Auto.AutosCommands;
//import frc.robot.Commands.Drive_With_Joysticks;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final CameraSubsystem m_robotCameras = new CameraSubsystem();

  //Creates the pdh for pdh widget
  private PowerDistribution m_pdh = new PowerDistribution();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
   
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_subsystemController = new XboxController(OIConstants.kSubsystemsControllerPort);

  public Object drive;

  // Creates the tabs in shuffleboard
  ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");
  ShuffleboardTab PreGameTab = Shuffleboard.getTab("Pre Game");

  // creates widget for the rev board
  ComplexWidget PdhWidget = TeleopTab
  .add("Power",m_pdh)
  .withWidget(BuiltInWidgets.kPowerDistribution);

  //makes the widget for the auto selector
  ComplexWidget AutoSelector = PreGameTab
  .add("Auto", autoChooser)
  .withWidget(BuiltInWidgets.kComboBoxChooser)
  .withSize(3, 2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() { 

    // sets all the options for the auto chooser
    autoChooser.addOption("No auto", null);
    autoChooser.addOption("Middle 2 Note", new AutosCommands().Auto2NoteMiddle(m_robotDrive, m_robotIntake, m_robotShooter));
    autoChooser.addOption("Middle 1 Note", new AutosCommands().Auto1NoteMove(m_robotDrive, m_robotIntake, m_robotShooter));
    autoChooser.addOption("test", new AutosCommands().TestAuto(m_robotDrive));

    // Configure the button bindings
    configureButtonBindings();
  
    // sets drive default command
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    // sets intake default command
    m_robotIntake.setDefaultCommand(new IntakeMove( m_subsystemController, m_robotIntake));

    m_robotShooter.setDefaultCommand(new ShootNote(m_robotShooter, m_subsystemController));
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
               false,   true),
              m_robotDrive));

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading()
        ));
    
    new JoystickButton(m_driverController, Button.kX.value)
        .onTrue(new MoveToTagPosition(m_robotDrive, m_robotCameras, 1.5, 0, 0.2));

    new JoystickButton(m_subsystemController, Button.kA.value)
        .onTrue(new IntakeOut(m_robotIntake));

    new JoystickButton(m_subsystemController, Button.kB.value)
        .onTrue(new LoadNote(m_robotIntake));

    new JoystickButton(m_subsystemController, Button.kX.value)
        .onTrue(new ShootIntakeAmp(m_robotIntake));

    new JoystickButton(m_subsystemController, Button.kRightBumper.value)
        .onTrue(new BumpIntake(m_robotIntake));
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
