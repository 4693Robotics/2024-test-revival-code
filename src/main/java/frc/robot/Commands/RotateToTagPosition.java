package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToTagPosition extends Command {

    private DriveSubsystem drivesystem;
    private CameraSubsystem camerasystem;
    private PIDController yawPIDController;

    private double maxSpeed;
    
    private boolean inPosition;

    public RotateToTagPosition(DriveSubsystem DriveSubsystem, CameraSubsystem CameraSubsystem, double maxSpeed) {

        this.drivesystem = DriveSubsystem;
        this.camerasystem = CameraSubsystem;
        yawPIDController = new PIDController(5, 0, 0);

        this.maxSpeed = maxSpeed;

        addRequirements(DriveSubsystem, CameraSubsystem);
    }

    public void initialize() {
        yawPIDController.setSetpoint(0);
    }

    public void execute() {
        if (camerasystem.getBestTagYaw() != 0.0) {

        double yawOutput = yawPIDController.calculate(camerasystem.getBestTagYaw());
        double yawOutputLimit = Math.copySign(Math.min(Math.abs(yawOutput), maxSpeed), yawOutput);

        SmartDashboard.putNumber("tag Yaw", camerasystem.getBestTagYaw());

        drivesystem.drive(0, 0, yawOutputLimit, false, false);

        } else {
            drivesystem.drive(0, 0, 0, false, false);
        }

        inPosition = yawPIDController.atSetpoint();
    }

    public void end(boolean interrupted) {
        drivesystem.drive(0, 0, 0, false, false);
    }

    public boolean isFinished() {
        return inPosition;
    }
}
