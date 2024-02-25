package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMove extends Command {

    IntakeSubsystem intakeSubsystem;
    XboxController controller;

     public IntakeMove(XboxController controller, IntakeSubsystem Intake) {
        intakeSubsystem = Intake;
        this.controller = controller;

        addRequirements(Intake);
     }

     @Override
     public void execute() {
      double XLeftspeed = -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.KSubsystemsDeadband);
      double YRightspeed = -MathUtil.applyDeadband(-controller.getRightY(), OIConstants.KSubsystemsDeadband);
      intakeSubsystem.moveIntakeArm(XLeftspeed/10);
      intakeSubsystem.moveIntakeRoller(YRightspeed);
     }

     @Override
     public void end(boolean interrupted) {
        intakeSubsystem.moveIntakeArm(0);
        intakeSubsystem.moveIntakeRoller(0);
     }

     @Override
     public boolean isFinished() {
       return false;
     }
    
}
