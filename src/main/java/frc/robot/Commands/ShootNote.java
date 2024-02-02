package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootNote extends Command {

    private final ShooterSubsystem shooter;
    private final XboxController controller;
    private final POVButton increaseButton;
    private final POVButton decreaseButton;

    public ShootNote(ShooterSubsystem shooter, XboxController XboxController) {
        this.shooter = shooter;
        this.controller = XboxController;

        // Assuming D-pad Up and D-pad Down for increase and decrease
        increaseButton = new POVButton(controller, 180);
        decreaseButton = new POVButton(controller, 0);

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (increaseButton.getAsBoolean()) {
            // Increase shooter speed when D-pad Up is pressed
            shooter.setShooterSpeed(shooter.getShooterSpeed() + 0.1);
        } else if (decreaseButton.getAsBoolean()) {
            // Decrease shooter speed when D-pad Down is pressed
            shooter.setShooterSpeed(shooter.getShooterSpeed() - 0.1);
        }
    }
}
