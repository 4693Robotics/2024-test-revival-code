package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TrackedTranslation extends Command {

    double xSpeed;
    double ySpeed;
    double time;
    double keptAngle;

    Timer timer;

    PIDController rotController;

    DriveSubsystem drivesystem;

    public TrackedTranslation(double xSpeed, double ySpeed, double time, DriveSubsystem DriveSubsystem) {

    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.time = time;

    this.timer = new Timer();

    this.rotController = new PIDController(0.1, 0, 0);

    this.drivesystem = DriveSubsystem;

    }

    public void initialize() {

    keptAngle = drivesystem.getHeading();
    rotController.setSetpoint(keptAngle);

    timer.reset();
    timer.start();

    }

    public void execute() {

    double rotSpeed = rotController.calculate(drivesystem.getHeading());
    drivesystem.drive(xSpeed, ySpeed, rotSpeed, false, false);

    }

    public void end(boolean interrupted) {

    drivesystem.drive(0, 0, 0, false, false);
    timer.stop();

    }

    public boolean isFinished() {
        return time < timer.get();
    }

    
}
