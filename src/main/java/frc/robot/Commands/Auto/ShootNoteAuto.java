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
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        shootersystem.setShooterSpeed(1);
        shootersystem.setFeederSpeed(1);
    }

    @Override
    public void execute() {
        if (timer.get() > 0.2) {
            intakesystem.moveIntakeRoller(-1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shootersystem.setShooterSpeed(0);
        shootersystem.setFeederSpeed(0);
        intakesystem.moveIntakeRoller(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 3;
        
    }
}
