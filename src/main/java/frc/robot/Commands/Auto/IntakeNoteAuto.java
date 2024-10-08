
package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNoteAuto extends Command {

    IntakeSubsystem intakesystem;
    DriveSubsystem drivesystem;
    Timer timer;
    
    public IntakeNoteAuto(IntakeSubsystem IntakeSubsystem, DriveSubsystem DriveSubsystem) {

        this.intakesystem = IntakeSubsystem;
        this.drivesystem = DriveSubsystem;
        this.timer = new Timer();

        addRequirements(IntakeSubsystem, DriveSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        intakesystem.setArmPosition(1);
    }

    @Override
    public void execute() {
        if (timer.get() > 0.1) {
            drivesystem.drive(-0.1, 0, 0, false, true);
            intakesystem.setIntakeRollerSpeed(0.5);
        }

        if (timer.get() > 0.6) {
            drivesystem.drive(0, 0, 0, false, true);
            intakesystem.setIntakeRollerSpeed(0);
            intakesystem.setArmPosition(0);
        }

        if (timer.get() > 0.65) {
            drivesystem.drive(0.1, 0, 0, false, true);
        }

        if (timer.get() > 1.25) {
            drivesystem.drive(0, 0, 0, false, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivesystem.drive(0, 0, 0, false, true);
    
        intakesystem.setIntakeRollerSpeed(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 2;
        
    }
}
