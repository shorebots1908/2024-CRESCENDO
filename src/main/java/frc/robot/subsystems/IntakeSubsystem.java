package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;

public class IntakeSubsystem extends SubsystemBase {
      private CANSparkMax m_intake1 = new CANSparkMax(9, MotorType.kBrushless);
      //private CANSparkMax m_intake2 = new CANSparkMax(10, MotorType.kBrushless);
      private AnalogInput feedSensor;

      //CONSTANTS
      private double intakeSpeed = 0;
      private double intakeReverse = 0;
      private int sensorThreshold = 500;
      private boolean intakeOn = false;

    public IntakeSubsystem() {
        m_intake1.setIdleMode(IdleMode.kBrake);
        intakeSpeed = SmartDashboard.getNumber("Intake Speed", intakeSpeed);
        intakeReverse = SmartDashboard.getNumber("Reverse Intake", intakeReverse);
        intakeOn = SmartDashboard.getBoolean("Intake On", intakeOn);
        feedSensor = new AnalogInput(1);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Sensor Value", feedSensor.getValue());
    }

    public void noteIntake() {
        //TODO: set dependent on boolean evaluation of sensor value.
        intakeSpeed = 0.7;
        intakeReverse = 0;
        intakeOn = true;
    }

    public void noteFeed() {
        //TODO: run inbtake to feed into shooter, ignoring sensor value.
    }

}
