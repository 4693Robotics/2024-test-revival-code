package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterMove extends Command {

    private final ShooterSubsystem shootersystem;
    private final XboxController controller;
    private final POVButton upDpad;
    private final POVButton downDpad;
    private final POVButton rightDpad;
    private final POVButton leftDpad;

    public ShooterMove(ShooterSubsystem ShooterSubsystem, XboxController XboxController) {
        this.shootersystem = ShooterSubsystem;
        this.controller = XboxController;

        // Assuming D-pad Up and D-pad Down for increase and decrease
        upDpad = new POVButton(controller, 0);
        downDpad = new POVButton(controller, 180);
        rightDpad = new POVButton(controller, 90);
        leftDpad = new POVButton(controller, 270);

        addRequirements(shootersystem);
    }

    @Override
    public void execute() {
                
        double YRightspeed = MathUtil.applyDeadband(controller.getRightY(), OIConstants.KSubsystemsDeadband);
        shootersystem.setFeederSpeed(YRightspeed);

        double YLeftspeed = MathUtil.applyDeadband(controller.getLeftY(), OIConstants.KSubsystemsDeadband);
        shootersystem.setShooterAngleSpeed(YLeftspeed / 5);

        if (upDpad.getAsBoolean()) {
         
            shootersystem.setShooterSpeed(1);
        } else if (downDpad.getAsBoolean()) {
            
            shootersystem.setShooterSpeed(0);
        } else if (rightDpad.getAsBoolean()) {
            
            shootersystem.setShooterSpeed(0.3);
        } else if (leftDpad.getAsBoolean()) {

            shootersystem.setShooterSpeed(-0.3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shootersystem.setFeederSpeed(0);
        shootersystem.setShooterSpeed(0);
        shootersystem.setShooterAngleSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
