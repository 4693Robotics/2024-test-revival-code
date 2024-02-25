package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class BumpIntake extends Command {

    Timer time;

    IntakeSubsystem intakesystem;

    public BumpIntake(IntakeSubsystem IntakeSubsystem) {

        time = new Timer();

        this.intakesystem = IntakeSubsystem;
    }

    @Override
    public void initialize() {
        time.reset();
         time.start();
    }

    @Override
    public void execute() {
        if (time.get() < 0.05) {
            intakesystem.moveIntakeRoller(-0.4);
        } else {
            intakesystem.moveIntakeRoller(0);
        }
     
    }
    
    @Override
    public void end(boolean interrupted) {
        intakesystem.moveIntakeRoller(0);
        time.stop();
    }

    @Override
    public boolean isFinished() {
        return time.get() > 0.25;
    }
}
