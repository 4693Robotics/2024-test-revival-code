package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToAngle extends Command {

    double rotationSpeed;
    double endDirection;
    DriveSubsystem driveSystem;

    public RotateToAngle(double speed, double finalDirection, DriveSubsystem DriveSubsystem) {
        rotationSpeed = speed;
        endDirection = finalDirection;
        driveSystem = DriveSubsystem;
    }

    public void execute() {
        driveSystem.drive(0, 0, rotationSpeed, true, false);
    }

    public void end(boolean interrupted) {
        driveSystem.drive(0, 0, 0, true, false);
    }

    public boolean isFinished() {
        return driveSystem.getHeading() >= endDirection;
    }

}