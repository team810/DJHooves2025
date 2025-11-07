package frc.robot.subsystem.shooter;


import org.littletonrobotics.junction.Logger;

public class Shooter {
    private final static Shooter INSTANCE = new Shooter();
    public static Shooter getInstance() {return INSTANCE;}

    private final ShooterIO shooter;

    private double topTargetSpeed;
    private double bottomTargetSpeed;

    private ShooterMode shooterMode;

    private double targetTopTestRPM;
    private double targetBottomTestRPM;

    private Shooter()
    {
        shooter = new ShooterReal();

        topTargetSpeed = 0;
        bottomTargetSpeed = 0;

        shooterMode = ShooterMode.off;

        targetTopTestRPM = 2000;
        targetBottomTestRPM = 2000;
    }

    public void readPeriodic() {
        shooter.readPeriodic();
    }

    public void writePeriodic() {
        Logger.recordOutput("Shooter/Top/TargetSpeedSub", topTargetSpeed);
        Logger.recordOutput("Shooter/Bottom/TargetSpeedSub", bottomTargetSpeed);
        Logger.recordOutput("Shooter/Mode/ShooterMode", shooterMode);

        shooter.writePeriodic();
    }

    public void setShooterMode(ShooterMode shooterMode) {
        this.shooterMode = shooterMode;

        switch (shooterMode)
        {
            case SourceIntake -> {
                topTargetSpeed = -2000;
                bottomTargetSpeed = -2000;
            }
            case Amp -> {
                topTargetSpeed = 2500;
                bottomTargetSpeed = 2500;
            }
            case Tape -> {
                topTargetSpeed = 2300;
                bottomTargetSpeed = 2300;
            }
            case Subwoofer -> {
                topTargetSpeed = 2300;
                bottomTargetSpeed = 2300;
            }
            case test -> {
                topTargetSpeed = targetTopTestRPM;
                bottomTargetSpeed = targetBottomTestRPM;
            }
            case off -> {
                topTargetSpeed = 0;
                bottomTargetSpeed = 0;
            }
        }

        shooter.setTopTargetRPM(topTargetSpeed);
        shooter.setBottomTargetRPM(bottomTargetSpeed);
    }
}


