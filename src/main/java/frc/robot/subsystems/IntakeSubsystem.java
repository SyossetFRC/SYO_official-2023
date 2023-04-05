package frc.robot.subsystems;

import static frc.robot.Constants.INTAKE_MOTOR_1;
import static frc.robot.Constants.INTAKE_MOTOR_2;
import static frc.robot.Constants.CLAW_SOLENOID_FORWARD;
import static frc.robot.Constants.CLAW_SOLENOID_REVERSE;
import static frc.robot.Constants.RELEASE_SOLENOID_1;
import static frc.robot.Constants.RELEASE_SOLENOID_2;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private DoubleSolenoid m_claw;

    private CANSparkMax m_intake1;
    private CANSparkMax m_intake2;
    private MotorControllerGroup m_intake;
    private Solenoid m_release1;
    private Solenoid m_release2;

    public IntakeSubsystem() {
        m_claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, CLAW_SOLENOID_FORWARD, CLAW_SOLENOID_REVERSE);
        m_release1 = new Solenoid(PneumaticsModuleType.REVPH, RELEASE_SOLENOID_1);
        m_release2 = new Solenoid(PneumaticsModuleType.REVPH, RELEASE_SOLENOID_2);

        m_intake1 = new CANSparkMax(INTAKE_MOTOR_1, MotorType.kBrushed);
        m_intake2 = new CANSparkMax(INTAKE_MOTOR_2, MotorType.kBrushed);
        m_intake = new MotorControllerGroup(m_intake1, m_intake2);
    }

    public void close() {
        m_intake.set(0.6);
        m_release1.set(false);
        m_release2.set(false);
        m_claw.set(Value.kForward);
    }


    public void open() {
        m_intake.set(0.0);
        m_release1.set(true);
        m_release2.set(true);
        Timer.delay(0.2);
        m_claw.set(Value.kReverse);
        Timer.delay(0.5);
        m_release1.set(false);
        m_release2.set(false);

    }

    public void releaseSuction() {
        m_intake.set(0);
        m_release1.set(true);
        m_release2.set(true);
    }
}
