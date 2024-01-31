package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.RotateToAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutosCommands {
    
    public SequentialCommandGroup Auto1(DriveSubsystem drivesystem) {
     return new SequentialCommandGroup(
        new InstantCommand(() -> drivesystem.zeroHeading()),
        new InstantCommand(() -> drivesystem.drive(0.3, 0, 0, true, false)),
        new WaitCommand(1),
        new RotateToAngle(drivesystem, 90, 0.25),
        new InstantCommand(() -> drivesystem.drive(0, 0.3, 0, true, false)),
        new WaitCommand(1),
        new InstantCommand(() -> drivesystem.drive(0, 0, 0, true, false)),
        new InstantCommand(() -> drivesystem.setX())
        );
    }

    public SequentialCommandGroup Auto2(DriveSubsystem drivesystem, IntakeSubsystem intakesystem) {
      return new SequentialCommandGroup(
        new InstantCommand(() -> drivesystem.drive(0.3, 0, 0, true, false)),
        new WaitCommand(1),
        new InstantCommand(() -> drivesystem.drive(0, 0, 0, true, false)),
        new RotateToAngle(drivesystem, 180, 0.25),
        new InstantCommand(() -> intakesystem.moveIntakeRoller(1)),
        new WaitCommand(1),
        new InstantCommand(() -> drivesystem.drive(-0.3, 0, 0, true, false)),
        new WaitCommand(1),
        new InstantCommand(() -> drivesystem.drive(0, 0, 0, true, false)),
        new InstantCommand(() -> intakesystem.intakeOff()),
        new RotateToAngle(drivesystem, 0, 0.1),
        new InstantCommand(() -> drivesystem.setX())    
        );
    }
    
    public SequentialCommandGroup Auto3(DriveSubsystem drivesystem, IntakeSubsystem intakesystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> drivesystem.zeroHeading()),
            new InstantCommand(() -> intakesystem.moveIntakeRoller(1)),
            new InstantCommand(() -> drivesystem.drive(0.3, 0, 0 ,true, false)),
            new WaitCommand(1),
            new InstantCommand(() -> drivesystem.drive(0, 0, 0, true, false)),
            new InstantCommand(() -> drivesystem.setX())
        );
    }
}


