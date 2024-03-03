package frc.robot.subsystems;
import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;

/*
 * TODO: prevent feeding; unless lift system is above/below encoder value
 * 
 * 
 */

public class IntakeSubsystem extends SubsystemBase {
      private CANSparkMax m_intake1 = new CANSparkMax(9, MotorType.kBrushless);
      private CANSparkMax m_intake2 = new CANSparkMax(10, MotorType.kBrushless);
      private AnalogInput feedSensor;
      

      //CONSTANTS
      private double intakeSpeed = 0.3;
      private double feedSpeed = 1;
      private double intakeReverse = -0.4;
      private int sensorThreshold = 800;
      private boolean intakeOn = false;

    public IntakeSubsystem() {
        m_intake1.setIdleMode(IdleMode.kBrake);
        m_intake2.setIdleMode(IdleMode.kBrake);
        intakeSpeed = SmartDashboard.getNumber("Intake Speed", intakeSpeed);
        intakeReverse = SmartDashboard.getNumber("Reverse Intake", intakeReverse); 
        intakeOn = SmartDashboard.getBoolean("Intake On", intakeOn);
        feedSensor = new AnalogInput(0);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Sensor Value", feedSensor.getValue());
        RelativeEncoder intake1Encoder = m_intake1.getEncoder();
        RelativeEncoder intake2Encoder = m_intake2.getEncoder();
        intakeOn = SmartDashboard.putBoolean("Intake On", intakeOn);
    }

    public void noteIntake() {
        //TODO: set dependent on boolean evaluation of sensor value.
        if (feedSensor.getValue() < sensorThreshold) {
            m_intake1.set(intakeSpeed);
            m_intake2.set(intakeSpeed);
        }
        else {
            m_intake1.stopMotor();
            m_intake2.stopMotor();
        }
    }

    public void noteUntake(){
        m_intake1.set(intakeReverse);
        m_intake2.set(intakeReverse);
    }
    

    public void noteFeed() {
        //TODO: run intake to feed into shooter, ignoring sensor value.
        m_intake1.set(feedSpeed);
        m_intake2.set(feedSpeed);
    }
    public void intakeStop() {
        m_intake1.stopMotor();
        m_intake2.stopMotor();
    }
    public void noteFeedStop() {

        m_intake1.stopMotor();
        m_intake2.stopMotor();
    }

    public boolean intakeSensor() {
        return feedSensor.getValue() > sensorThreshold;
    }

}
