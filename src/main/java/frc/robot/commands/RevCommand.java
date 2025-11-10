package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.ShooterState;

public class RevCommand extends Command {
    private final ShooterState state;
    public RevCommand(ShooterState state) {
        this.state = state;
    }

    @Override
    public void initialize() {
        Shooter.getInstance().setShooterMode(state);
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().setShooterMode(ShooterState.off);
    }

}
