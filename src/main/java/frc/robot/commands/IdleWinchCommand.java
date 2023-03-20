package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WinchSubsystem;

public class IdleWinchCommand extends CommandBase {
    private final WinchSubsystem m_winchSubsystem;

    private long m_waitTime;
    private Timer timer;

    public IdleWinchCommand(WinchSubsystem winchSubsystem) {
        this(winchSubsystem, (long) Double.POSITIVE_INFINITY);
    }

    public IdleWinchCommand(WinchSubsystem winchSubsystem,
                                long msecs) {
        this.m_winchSubsystem = winchSubsystem;
        this.m_waitTime = msecs;
        timer = new Timer();

        addRequirements(winchSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        m_winchSubsystem.rotate(0);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() < (m_waitTime / 1000.0)) {
            return false;
        }
        return true;
    }
}