package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootNote extends Command {

    private ShooterSubsystem shootersystem;
    
    public ShootNote(ShooterSubsystem ShooterSubsystem) {

        this.shootersystem = ShooterSubsystem;
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
