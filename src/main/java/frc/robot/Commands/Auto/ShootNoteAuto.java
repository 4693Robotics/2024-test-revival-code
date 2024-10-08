package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootNoteAuto extends Command {

    IntakeSubsystem intakesystem;
    ShooterSubsystem shootersystem;
    Timer timer;
    
    public ShootNoteAuto(IntakeSubsystem IntakeSubsystem, ShooterSubsystem ShooterSubsystem) {

        this.intakesystem = IntakeSubsystem;
        this.shootersystem = ShooterSubsystem;
        this.timer = new Timer();

        addRequirements(IntakeSubsystem, ShooterSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        shootersystem.setShooterSpeed(-0.55);
    }

    @Override
    public void execute() {
        if (timer.get() > 0.4) {
            shootersystem.setFeederSpeed(-1);
            intakesystem.setIntakeRollerSpeed(-1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shootersystem.setShooterSpeed(0);
        shootersystem.setFeederSpeed(0);
        intakesystem.setIntakeRollerSpeed(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 1;
        
    }
}
