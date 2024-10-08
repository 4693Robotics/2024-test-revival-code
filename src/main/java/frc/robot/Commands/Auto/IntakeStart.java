package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeStart extends Command {

    IntakeSubsystem intakesystem;
    Timer timer;
    
    public IntakeStart(IntakeSubsystem IntakeSubsystem) {

        this.intakesystem = IntakeSubsystem;
        
        this.timer = new Timer();

        addRequirements(IntakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        intakesystem.setArmPosition(1);
        intakesystem.setIntakeRollerSpeed(0.55);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 0.1;
        
    }
}
