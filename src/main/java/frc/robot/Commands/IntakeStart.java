package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStart extends Command {

    IntakeSubsystem intakeSubsystem;
    XboxController controllerXAxis;

     public IntakeStart(XboxController controller, IntakeSubsystem Intake) {
        intakeSubsystem = Intake;
        controllerXAxis = controller;

        addRequirements(Intake);
     }

     @Override
     public void execute() {
        double XLeftspeed = -MathUtil.applyDeadband(controllerXAxis.getLeftX(), OIConstants.KSubsystemsDeadband);
        double XRightspeed = -MathUtil.applyDeadband(controllerXAxis.getRightX(), OIConstants.KSubsystemsDeadband);
        intakeSubsystem.moveIntake(XLeftspeed);
        intakeSubsystem.moveIntakeMini(XRightspeed);
     }

     @Override
     public void end(boolean interrupted) {
        intakeSubsystem.moveIntake(0);
        intakeSubsystem.moveIntakeMini(0);
     }

     @Override
     public boolean isFinished() {
       return false;
     }
    
}
