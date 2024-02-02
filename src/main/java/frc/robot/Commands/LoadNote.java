package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class LoadNote extends Command {

    private DriveSubsystem drivesystem;
    private IntakeSubsystem intakesystem;
    
    public LoadNote(DriveSubsystem DriveSubsystem, IntakeSubsystem IntakeSubsystem) {

        this.intakesystem = IntakeSubsystem;
        this.drivesystem = DriveSubsystem;

        addRequirements(DriveSubsystem, IntakeSubsystem);
    }

    public void initialize() {

    }

    public void execute() {

    }

    public void end() {

    }

    public boolean isFinished(boolean interrupted) {
        return false;
    }
}
