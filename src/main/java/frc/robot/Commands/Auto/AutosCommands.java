package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Commands.BumpIntake;
import frc.robot.Commands.DriveTranslation;
import frc.robot.Commands.IntakeMove;
import frc.robot.Commands.IntakeOut;
import frc.robot.Commands.LoadNote;
import frc.robot.Commands.MoveToTagPosition;
import frc.robot.Commands.RotateToAngle;
import frc.robot.Commands.ShootNote;
import frc.robot.Commands.TrackedTranslation;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutosCommands {
    
    public SequentialCommandGroup Auto2NoteMiddle(DriveSubsystem DriveSubsystem, IntakeSubsystem IntakeSubsystem, ShooterSubsystem ShooterSubsystem) {
        return new SequentialCommandGroup(
           new InstantCommand(() -> ShooterSubsystem.setShooterSpeed(1)),
           new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(-1)), //Ejects loaded piece
           new WaitCommand(0.7),
           new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(0)),
           new WaitCommand(0.4),
           new InstantCommand(() -> ShooterSubsystem.setShooterSpeed(0)), //Stops the Ejection
           new IntakeOut(IntakeSubsystem), //Moves the Intake out
           new WaitCommand(0.7),
           new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(0.5)), //Turns on the roller 
           new RotateToAngle(DriveSubsystem, 0, 0), //Needs a rotate angle to not break things don't know why
           new InstantCommand(() -> DriveSubsystem.drive(-0.7, 0, 0, false, false)), //Move robot backwards for 0.6 secconds
           new WaitCommand(0.6),
           new InstantCommand(() -> DriveSubsystem.drive(0, 0, 0, false, false)), //Stops the robot
           new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(0)),
           new LoadNote(IntakeSubsystem), //Move the intake inwards
           new WaitCommand(0.7),
           new InstantCommand(() -> DriveSubsystem.drive(0.7, 0, 0, false, false)), //Moves Robot back to get ready to shoot 
           new WaitCommand(0.4),
           new InstantCommand(() -> DriveSubsystem.drive(0, 0, 0, false, false)), //Stops the robot
           new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(-1)),
           new InstantCommand(() -> ShooterSubsystem.setShooterSpeed(1)), //Shoots the note
           new WaitCommand(0.4),
           new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(0)), //Stops the shooter and the intake
           new InstantCommand(() -> ShooterSubsystem.setShooterSpeed(0))
        ); 
    }

    public SequentialCommandGroup Auto1NoteMove(DriveSubsystem DriveSubsystem, IntakeSubsystem IntakeSubsystem, ShooterSubsystem ShooterSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> ShooterSubsystem.setShooterSpeed(1)),
            new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(-1)), //Ejects loaded piece
            new WaitCommand(1),
            new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(0)),
            new WaitCommand(0.4),
            new InstantCommand(() -> ShooterSubsystem.setShooterSpeed(0)), //Stops the Ejection
            new WaitCommand(0.7),
            new InstantCommand(() -> IntakeSubsystem.moveIntakeRoller(0.5)), //Turns on the roller 
            new RotateToAngle(DriveSubsystem, 0, 0), //Needs a rotate angle to not break things don't know why
            new InstantCommand(() -> DriveSubsystem.drive(-0.7, 0, 0, false, false)), //Move robot backwards for 0.6 secconds
            new WaitCommand(0.6),
            new InstantCommand(() -> DriveSubsystem.drive(0, 0, 0, false, false)) //Stops the robot
        );
    }
    public SequentialCommandGroup TestAuto(DriveSubsystem DriveSubsystem) {
        return new SequentialCommandGroup(
            new RotateToAngle(DriveSubsystem, 0, 0),
            new TrackedTranslation(0.6, 0, 2, DriveSubsystem)
        );
    }

    

 }
