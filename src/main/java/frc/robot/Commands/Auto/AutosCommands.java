package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Commands.DriveTranslation;
import frc.robot.Commands.IntakeMove;
import frc.robot.Commands.IntakeOut;
import frc.robot.Commands.LoadNote;
import frc.robot.Commands.MoveToTagPosition;
import frc.robot.Commands.RotateToAngle;
import frc.robot.Commands.RotateToTagPosition;
import frc.robot.Commands.ShootNote;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutosCommands {
    
    public SequentialCommandGroup Test2Note(DriveSubsystem DriveSubsystem, IntakeSubsystem IntakeSubsystem) {
        return new SequentialCommandGroup(
           new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(-1)), //Ejects loaded piece
           new WaitCommand(0.25),
           new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(0)), //Stops the Ejection
           new IntakeOut(IntakeSubsystem), //Moves the Intake out
           new WaitCommand(0.5),
           new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(0.5)), //Turns on the roller 
           new RotateToAngle(DriveSubsystem, 0, 0), //Needs a rotate angle to not break things don't know why
           new InstantCommand(() -> DriveSubsystem.drive(-0.7, 0, 0, false, false)), //Move robot backwards for 2.4 secconds
           new WaitCommand(1),
           new InstantCommand(() -> DriveSubsystem.drive(0, 0, 0, false, false)),
           new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(0)),
           new LoadNote(IntakeSubsystem),
           new WaitCommand(0.5),
           new InstantCommand(() -> DriveSubsystem.drive(0.7, 0, 0, false, false)),
           new WaitCommand(1),
           new InstantCommand(() -> DriveSubsystem.drive(0, 0, 0, false, false)),
           new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(-1)),
           new WaitCommand(0.40),
           new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(0)),
           new InstantCommand(() -> DriveSubsystem.drive(-0.7, 0, 0, false, false)),
           new WaitCommand(1.4),
           new InstantCommand(() -> DriveSubsystem.drive(0, 0, 0, false, false))
        ); 
    }

 }
