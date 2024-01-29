package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TestAuto extends Command {
    
    DriveSubsystem DriveSystem;

    public TestAuto(DriveSubsystem DriveSubsystem) {
        DriveSystem = DriveSubsystem;
    }

    @Override
    public void execute() {
        DriveSystem.drive(0.1, 0, 0, true, false);
        Timer.delay(0.2);
        DriveSystem.drive(0, 0, 0.2, true, false);
        Timer.delay(0.3);
        DriveSystem.drive(0, 0, 0, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        DriveSystem.drive(0, 0, 0, true, false);
        DriveSystem.setX();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
