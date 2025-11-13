package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.MechanismState;
import frc.robot.subsystem.deflector.Deflector;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeStates;
import frc.robot.subsystem.laser.Laser;
import frc.robot.subsystem.laser.LaserState;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.ShooterState;

public class IntakeCommand extends Command {
    private final IntakeStates state;
    private final boolean ignoreLaser;

    public IntakeCommand(IntakeStates state, boolean ignoreLaser) {
        this.state = state;
        this.ignoreLaser = ignoreLaser;
    }
    @Override
    public void initialize() {
        Intake.getInstance().setState(state);
        if (state == IntakeStates.rev && !ignoreLaser) {
            Shooter.getInstance().setShooterMode(ShooterState.SourceIntake);
            Deflector.getInstance().setDeflectorState(MechanismState.deployed);
        }
    }
    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().setState(IntakeStates.off);
        if (state == IntakeStates.rev && !ignoreLaser) {
            Shooter.getInstance().setShooterMode(ShooterState.off);
            Deflector.getInstance().setDeflectorState(MechanismState.stored);
        }
    }

    @Override
    public boolean isFinished() {
        return Laser.getInstance().getLaserState() == LaserState.Detected && !ignoreLaser;
    }
}
