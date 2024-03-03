package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class MoveToTagPosition extends Command {

    private DriveSubsystem drivesystem;
    private VisionSubsystem camerasystem;
    private PIDController xPIDController;
    private PIDController yPIDController;
    private PIDController rotPIDController;
    private Timer time;

    private double maxSpeed;
    private double xdistance;
    private double ydistance;
    private double rotdistance;
    private int TagId;

    private boolean inPosition;
    private boolean isTagDetected;

    public MoveToTagPosition(DriveSubsystem DriveSubsystem, VisionSubsystem CameraSubsystem, double xdistance, double ydistance, double maxSpeed, int TagId) {

        this.drivesystem = DriveSubsystem;
        this.camerasystem = CameraSubsystem;
        this.xPIDController = new PIDController(10, 0, 0);
        this.yPIDController = new PIDController(10, 0, 0);
        this.rotPIDController = new PIDController(5, 0, 0.5);
        this.time = new Timer();

        this.maxSpeed = maxSpeed;
        this.xdistance = xdistance;
        this.ydistance = ydistance;
        this.rotdistance = 3.15;
        this.TagId = TagId;

        addRequirements(DriveSubsystem, CameraSubsystem);
    }

    @Override
    public void initialize() {

        xPIDController.setSetpoint(xdistance);
        yPIDController.setSetpoint(ydistance);
        rotPIDController.setSetpoint(rotdistance);

        rotPIDController.setTolerance(0.1);
    }

    @Override
    public void execute() {
        if (camerasystem.getTagXDistance(TagId) != -1 && camerasystem.getTagYDistance(TagId) != -1) {
            double xOutput = xPIDController.calculate(camerasystem.getTagXDistance(TagId));
            double xOutputLimit = Math.copySign(Math.min(Math.abs(xOutput), maxSpeed), xOutput);

            double yOutput = yPIDController.calculate(camerasystem.getTagYDistance(TagId));
            double yOutputLimit = Math.copySign(Math.min(Math.abs(yOutput), maxSpeed), yOutput);

            double rotOutput = rotPIDController.calculate(camerasystem.getTagYaw(TagId));
            double rotOutputLimit = Math.copySign(Math.min(Math.abs(rotOutput), maxSpeed), rotOutput);

            drivesystem.drive(-xOutputLimit, -yOutputLimit, rotOutputLimit, false, true);

        } else {
            drivesystem.drive(0, 0, 0, false, false);
        }

        inPosition = xPIDController.atSetpoint() && yPIDController.atSetpoint() && rotPIDController.atSetpoint();
        isTagDetected = camerasystem.isTagDetected();
    }

    @Override
    public void end(boolean interrupted) {
        drivesystem.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return inPosition || isTagDetected;
    }
}