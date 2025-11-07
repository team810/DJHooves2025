// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.IO.IO;
import frc.robot.subsystem.deflector.Deflector;
import frc.robot.subsystem.drivetrain.Drivetrain;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.laser.Laser;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.tbone.TBone;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

    public Robot() {
        Logger.recordMetadata("ProjectName", "DJHooves");

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();

        IO.Initialize(
                IO.PrimaryDriverProfiles.Parm,
                IO.SecondaryDriverProfiles.Joystick
        );

        CommandScheduler.getInstance().setPeriod(.015);

    }

    @Override
    public void robotPeriodic() {
        readPeriodic();
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
