package frc.robot.IO;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public abstract class IO {
    public enum PrimaryDriverProfiles {
        Parm,
    }

    public enum SecondaryDriverProfiles {
        Joystick,
        Controller
    }

    private static final XboxController primary = new XboxController(0);
    private static final XboxController secondary = new XboxController(1);
    private static final Joystick secondaryJoystick = new Joystick(1);

    private static final HashMap<Controls, Supplier<Double>> controlsJoystick = new HashMap<>();
    private static final HashMap<Controls, BooleanSupplier> controlsButtons = new HashMap<>();

    public static void Initialize(PrimaryDriverProfiles primaryProfile, SecondaryDriverProfiles secondaryProfile) {
        controlsJoystick.clear();
        controlsButtons.clear();

        switch (primaryProfile) {
            case Parm:
                controlsJoystick.put(Controls.xDriveVelocity, primary::getLeftX);
                controlsJoystick.put(Controls.yDriveVelocity, primary::getLeftY);
                controlsJoystick.put(Controls.thetaDriveVelocity, primary::getRightX);

                controlsButtons.put(Controls.resetGyro, primary::getAButton);
                break;
        }

        switch (secondaryProfile) {
            case Joystick:
                controlsButtons.put(Controls.intakeGround, () -> secondaryJoystick.getRawButton(3));
                controlsButtons.put(Controls.intakeRevs, () -> secondaryJoystick.getRawAxis(2) > .5 || secondaryJoystick.getRawAxis(2) < -.5);
                controlsButtons.put(Controls.intakeSource, () -> secondaryJoystick.getRawButton(4));

                controlsButtons.put(Controls.score, () -> secondaryJoystick.getRawButton(1));
                controlsButtons.put(Controls.revClose, () -> secondaryJoystick.getRawAxis(1) > .6);
                controlsButtons.put(Controls.revFar, () -> secondaryJoystick.getRawAxis(1) < -.6);
                controlsButtons.put(Controls.ampScore, () -> secondaryJoystick.getX() > .5 || secondaryJoystick.getX() < -.5);

                break;
            case Controller:
                controlsButtons.put(Controls.intakeGround, secondary::getAButton);
                controlsButtons.put(Controls.intakeSource, secondary::getXButton);
                controlsButtons.put(Controls.intakeRevs, secondary::getYButton);


                controlsButtons.put(Controls.score, () -> secondary.getRightTriggerAxis() > .75);
                controlsButtons.put(Controls.revClose, secondary::getLeftBumperButton);
                controlsButtons.put(Controls.revFar, () -> secondary.getLeftTriggerAxis() > .75);

                controlsButtons.put(Controls.ampScore, secondary::getRightBumperButton);

                break;
        }
    }

    public static Supplier<Double> getJoystickValue(Controls control) {
        return controlsJoystick.get(control);
    }

    public static BooleanSupplier getButtonValue(Controls control) {
        return controlsButtons.get(control);
    }
}

