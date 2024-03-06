package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class FeederMove extends Command {

    ShooterSubsystem shootersystem;
    XboxController controller;

    public FeederMove(ShooterSubsystem ShooterSubsystem, XboxController controller) {

        this.shootersystem = ShooterSubsystem;
        this.controller = controller;

        addRequirements(ShooterSubsystem);
    }

    @Override
    public void initialize() {
    }
    

    @Override
    public void execute() {
        double YRightspeed = MathUtil.applyDeadband(controller.getRightY(), OIConstants.KSubsystemsDeadband);
        shootersystem.setFeederSpeed(YRightspeed);
    }

    @Override
    public void end(boolean interrupted) {
        shootersystem.setFeederSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
