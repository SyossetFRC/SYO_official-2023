package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class IdleDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private long m_waitTime;
    private Timer timer;

    public IdleDriveCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this(drivetrainSubsystem, (long) Double.POSITIVE_INFINITY);
    }

    public IdleDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                                long msecs) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_waitTime = msecs;
        timer = new Timer();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        if (timer.get() < (m_waitTime / 1000)) {
            return false;
        }
        return true;
    }
}
