package frc.robot.Commands;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class MoveToTagPosition extends Command {

    private DriveSubsystem drivesystem;
    private CameraSubsystem camerasystem;
    private List<PhotonTrackedTarget> selectedTarget;
    private PIDController xPIDController;
    private PIDController yPIDController;

    private double maxSpeed;
    private double xdistance;
    private double ydistance;

    private boolean inPosition;

    public MoveToTagPosition(DriveSubsystem DriveSubsystem, CameraSubsystem CameraSubsystem, double xdistance, double ydistance, double maxSpeed) {

        this.drivesystem = DriveSubsystem;
        this.camerasystem = CameraSubsystem;
        xPIDController = new PIDController(0.1, 0, 0);
        yPIDController = new PIDController(0.1, 0, 0);

        this.maxSpeed = maxSpeed;
        this.xdistance = xdistance;
        this.ydistance = ydistance;

        addRequirements(DriveSubsystem, CameraSubsystem);
    }

    public void initialize() {

        xPIDController.setSetpoint(xdistance);
        yPIDController.setSetpoint(ydistance);

        xPIDController.setTolerance(0.01);
        yPIDController.setTolerance(0.01);
    }

    public void execute() {
        selectedTarget = camerasystem.getTarget();

        double xOutput = xPIDController.calculate(selectedTarget.get(0).getBestCameraToTarget().getX());
        double xOutputLimit = Math.copySign(Math.min(Math.abs(xOutput), maxSpeed), xOutput);

        double yOutput = yPIDController.calculate(selectedTarget.get(0).getBestCameraToTarget().getY());
        double yOutputLimit = Math.copySign(Math.min(Math.abs(yOutput), maxSpeed), yOutput);

        drivesystem.drive(xOutputLimit, yOutputLimit, 0, false, false);

        SmartDashboard.putNumber("April tag x", xOutput);
        SmartDashboard.putNumber("April tag y", yOutput);

        inPosition = xPIDController.atSetpoint() && yPIDController.atSetpoint();
    }

    public void end(boolean interrupted) {
        drivesystem.drive(0, 0, 0, false, false);
    }

    public boolean isFinished() {
        return inPosition;
    }
}