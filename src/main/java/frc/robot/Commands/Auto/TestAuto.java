package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.RotateToAngle;
import frc.robot.subsystems.DriveSubsystem;

public class TestAuto extends Command {
    
    DriveSubsystem DriveSystem;
    double GyroAngle;

    public TestAuto(DriveSubsystem DriveSubsystem, double gyroHeading) {
        DriveSystem = DriveSubsystem;
        GyroAngle = gyroHeading;
    }

    @Override
    public void execute() {
        DriveSystem.zeroHeading();
        DriveSystem.drive(0.1, 0, 0, true, false);
        Timer.delay(0.2);
        new RotateToAngle(0.5, 90, DriveSystem);
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
