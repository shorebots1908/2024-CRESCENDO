package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * Encoder bottom value: 4000
 * Encoder top value: 3498
 * TODO: reset encoder values
 * TODO: sync the motors based on encoder values
 * TODO: add lift + lower functions
 * TODO: functions set to fixed encoder values (lift + lower)
 * TODO: function to read encoder values (very important)
 */

public class LiftSubsystem extends SubsystemBase {
    // private CANSparkMax lifter1 = new CANSparkMax(13, MotorType.kBrushless);
    // private CANSparkMax lifter2 = new CANSparkMax(14, MotorType.kBrushless);
    private DoubleSolenoid solenoid1 = new DoubleSolenoid(18, PneumaticsModuleType.CTREPCM, 0, 1);
    // the below variables are no longer being used and were the old design which was slow
    // switched to pneumatics due to speed and efficiency (wont be able to hang with pneumatics BUT)
    // faster amp <-> down time
    //
    // private RelativeEncoder lifter2Encoder, lifter1Encoder;
    // private double encoder1Position = 0, encoder2Position = 0;
    // private double difference;
    // private double encoder1Scale = 1;
    // private double encoder2Scale = 1;
    // private double maxDeviation = 4;
    // private double speed = -0.80;
    // private double slowThreshold = 40;
    // private double liftBottom = 4000;
    // private double liftTop = 3492;
    private Timer timer = new Timer();
    //3498; original liftTop

    // public void control(double direction) {
    //     syncMotors(speed * direction, lifter1Encoder.getPosition(), lifter2Encoder.getPosition());
    //     if(direction != 0)
    //     {
    //         //syncMotors(speed * direction, lifter1Encoder.getPosition(), lifter2Encoder.getPosition());
    //         lifter1.set(speed * direction*encoder1Scale);
    //         lifter2.set(speed * direction*encoder2Scale);
    //     }
    //     else {
    //         liftersStop();
    //     }
    // }

    // public void measuredControl(double direction) {
    //         if((encoder1Position >= liftBottom || encoder2Position >= liftBottom) && direction < 0){
    //             liftersStop();
    //         }
    //         else if((encoder1Position <= liftTop || encoder2Position <= liftTop) && direction > 0){
    //             liftersStop();
    //         }
    //         else if((encoder1Position > (liftBottom - slowThreshold + 2) || encoder2Position > (liftBottom - slowThreshold + 2)) && direction < 0){
    //             lifter1.set(-1 * direction * encoder1Scale * ((liftBottom - encoder1Position)/slowThreshold));
    //             lifter2.set(-1 * direction * encoder2Scale * ((liftBottom - encoder2Position)/slowThreshold));
    //         }
    //         else if ((encoder1Position < (liftTop + slowThreshold - 2) || encoder2Position < (liftTop + slowThreshold - 2 )) && direction > 0) {
    //             lifter1.set(-1 * direction * encoder1Scale * ((encoder1Position - liftTop)/slowThreshold));
    //             lifter2.set(-1 * direction * encoder2Scale * ((encoder2Position - liftTop)/slowThreshold));
    //         }
    //         else if (direction != 0) {
    //             lifter1.set(-1 * direction*encoder1Scale);
    //             lifter2.set(-1 * direction*encoder2Scale);
    //         }
    //         else {
    //             liftersStop();
    //         }
    // }

    // public void lift() {
    //     syncMotors(speed, lifter1Encoder.getPosition(), lifter2Encoder.getPosition());
    //     lifter1.set(speed*encoder1Scale);
    //     lifter2.set(speed*encoder2Scale);
    // }

    // public void lower() {
    //     syncMotors(-speed, lifter1Encoder.getPosition(), lifter2Encoder.getPosition());
    //     lifter1.set(-speed*encoder1Scale);
    //     lifter2.set(-speed*encoder2Scale);
    // }
    // public void liftersStop() {
    //     lifter1.stopMotor();
    //     lifter2.stopMotor();
    // }
    // public void liftersReset() {
    //     lifter1Encoder.setPosition(4000);
    //     lifter2Encoder.setPosition(4000);

    // }


    public LiftSubsystem() {
        // lifter2Encoder = lifter2.getEncoder();
        // lifter1Encoder = lifter1.getEncoder();
        solenoid1.set(Value.kForward);
    }
 

    // public void syncMotors(double velocity, double Encoder1, double Encoder2) {
    //     difference = Encoder1 - Encoder2;
    //     if (velocity > 0) {
    //         if (difference > 0) {
    //             encoder1Scale = Math.max((maxDeviation - Math.abs(difference))/(maxDeviation), 0);
    //             encoder2Scale = 1;
    //         }
    //         else if(difference < 0) {
    //             encoder1Scale = 1;
    //             encoder2Scale = Math.max((maxDeviation - Math.abs(difference))/(maxDeviation), 0);
    //         }
    //         else
    //         {
    //             encoder1Scale = 1;
    //             encoder2Scale = 1;
    //         }
    //     }
    //     else if(velocity < 0) {
    //         if (difference > 0) {
    //             encoder1Scale = 1;
    //             encoder2Scale = Math.max((maxDeviation - Math.abs(difference))/(maxDeviation), 0);
    //         }
    //         else if(difference < 0) {
    //             encoder1Scale = Math.max((maxDeviation - Math.abs(difference))/(maxDeviation), 0);
    //             encoder2Scale = 1;
    //         }
    //         else
    //         {
    //             encoder1Scale = 1;
    //             encoder2Scale = 1;
    //         }
    //     }
    // }

    public void timerInit(){
        timer.reset();
        timer.start();
    }

    public void timerStop(){
        timer.stop();
    }

    public double getTime(){
        return timer.get();
    }
    public boolean liftToAmp() {
        solenoid1.set(Value.kReverse);
        return true;
    }
    public boolean liftToNormalHeight(){
        solenoid1.set(Value.kForward);
        return true;
    }


    @Override
    public void periodic() {
        // encoder1Position = lifter1Encoder.getPosition();
        // encoder2Position = lifter2Encoder.getPosition();
        // SmartDashboard.putNumber("Lifter 1 Encoder", encoder1Position);
        // SmartDashboard.putNumber("Lifter 2 Encoder", encoder2Position);
        // SmartDashboard.putNumber("Encoder1", encoder1Scale);
        // SmartDashboard.putNumber("Encoder2", encoder2Scale);
        // SmartDashboard.putNumber("Lift Difference", (maxDeviation - (encoder1Position - encoder2Position))/maxDeviation);
        SmartDashboard.putString("Current Lift Position", solenoid1.get().toString());
    }


}
