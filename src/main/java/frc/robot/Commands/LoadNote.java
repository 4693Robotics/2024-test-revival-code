package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class LoadNote extends Command {

    private IntakeSubsystem intakesystem;
    
    public LoadNote(IntakeSubsystem IntakeSubsystem) {

        this.intakesystem = IntakeSubsystem;
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
