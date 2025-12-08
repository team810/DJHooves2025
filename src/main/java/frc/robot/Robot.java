
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RevCommand;
import frc.robot.subsystem.deflector.Deflector;
import frc.robot.subsystem.drivetrain.Drivetrain;
import frc.robot.subsystem.drivetrain.DrivetrainConstants;
import frc.robot.subsystem.drivetrain.control.ManualControlFOC;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeStates;
import frc.robot.subsystem.laser.Laser;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.ShooterState;
import frc.robot.subsystem.tbone.TBone;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
    private final Trigger intakeGround;
    private final Trigger intakeSource;
    private final Trigger intakeRevs;
    private final Trigger revFar;
    private final Trigger revClose;
    private final Trigger score;
    private final Trigger resetGyro;

    private ManualControlFOC swerveControl;

    public Robot() {
        Logger.recordMetadata("ProjectName", "DJHooves");

        if (isReal()) {
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();
        DriverStation.silenceJoystickConnectionWarning(true);

        IO.Initialize(
                IO.PrimaryDriverProfiles.Parm,
                IO.SecondaryDriverProfiles.Joystick
        );

        CommandScheduler.getInstance().setPeriod(.015);

        intakeGround = new Trigger(IO.getButtonValue(Controls.intakeGround));
        intakeSource = new Trigger(IO.getButtonValue(Controls.intakeSource));
        intakeRevs = new Trigger(IO.getButtonValue(Controls.intakeRevs));
        revFar = new Trigger(IO.getButtonValue(Controls.revFar));
        revClose = new Trigger(IO.getButtonValue(Controls.revClose));
        score = new Trigger(IO.getButtonValue(Controls.score));
        resetGyro = new Trigger(IO.getButtonValue(Controls.resetGyro));

        intakeGround.whileTrue(
            new IntakeCommand(IntakeStates.fwd, false)
        );
        intakeSource.whileTrue(
                new StartEndCommand(
                        () -> Intake.getInstance().setState(IntakeStates.off),
                        () -> Intake.getInstance().setState(IntakeStates.off)
                )
        );
        intakeRevs.whileTrue(
            new IntakeCommand(IntakeStates.rev, true)
        );
        score.whileTrue(
            new IntakeCommand(IntakeStates.fwd, true)
        );
        revFar.whileTrue(
            new RevCommand(ShooterState.Tape)
        );
        revClose.whileTrue(
            new RevCommand(ShooterState.Subwoofer)
        );
        resetGyro.whileTrue(
                new InstantCommand(Drivetrain.getInstance()::resetGyro)
        );

    }

    @Override
    public void robotPeriodic() {
        readPeriodic();

        swerveControl = new ManualControlFOC(
            -MathUtil.applyDeadband(IO.getJoystickValue(Controls.xDriveVelocity).get() ,.05) * 5.2,
            -MathUtil.applyDeadband(IO.getJoystickValue(Controls.yDriveVelocity).get() , .05) * 5.2,
            -MathUtil.applyDeadband(IO.getJoystickValue(
                    Controls.thetaDriveVelocity).get(), .1)  * 5.2 * (Math.sqrt((DrivetrainConstants.WHEEL_BASE_LENGTH * DrivetrainConstants.WHEEL_BASE_LENGTH)+(DrivetrainConstants.WHEEL_BASE_WIDTH * DrivetrainConstants.WHEEL_BASE_WIDTH)) * Math.PI)
        );
        Drivetrain.getInstance().setControl(swerveControl);
        CommandScheduler.getInstance().run();
        writePeriodic();
    }

    public void readPeriodic() {
        Drivetrain.getInstance().readPeriodic();
        Deflector.getInstance().readPeriodic();
        Intake.getInstance().readPeriodic();
        Laser.getInstance().readPeriodic();
        Shooter.getInstance().readPeriodic();
        TBone.getInstance().readPeriodic();
    }

    public void writePeriodic() {
        Drivetrain.getInstance().writePeriodic();
        Deflector.getInstance().writePeriodic();
        Intake.getInstance().writePeriodic();
        Shooter.getInstance().writePeriodic();
        TBone.getInstance().writePeriodic();
    }

    
    
    @Override
    public void autonomousInit() {}
    
    
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void teleopInit() {}
    
    
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {}
    
    
    @Override
    public void testInit() {}
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void simulationInit() {}
    
    
    @Override
    public void simulationPeriodic() {}
}
