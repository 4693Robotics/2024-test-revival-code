// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Drive_With_Joysticks extends Command {
  private final DriveSubsystem m_DriveSubsystem;
  private DoubleSupplier m_xspeed;
  private DoubleSupplier m_yspeed;
  private DoubleSupplier m_rot;
  private boolean m_fieldRelative;
  private boolean m_rateLimit;

  /** Creates a new Drive_With_Joysticks. */
  public Drive_With_Joysticks(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, boolean fieldRelative, boolean rateLimit, DriveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = subsystem;
    m_xspeed = xSpeed;
    m_yspeed = ySpeed;
    m_rot = rot;
    m_fieldRelative = fieldRelative;
    m_rateLimit = rateLimit;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xspeed =   MathUtil.applyDeadband(m_xspeed.getAsDouble(), OIConstants.kDriveDeadband);
    double yspeed =   MathUtil.applyDeadband(m_yspeed.getAsDouble(), OIConstants.kDriveDeadband);
    double rot    =   MathUtil.applyDeadband(m_rot.getAsDouble(), OIConstants.kDriveDeadband);
    
    m_DriveSubsystem.drive(xspeed, yspeed, rot, m_fieldRelative, m_rateLimit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.drive(0,0,0, m_fieldRelative, m_rateLimit);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
