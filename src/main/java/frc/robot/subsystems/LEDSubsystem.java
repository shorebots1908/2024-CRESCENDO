package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3.LEDCurrent;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {
    private Spark ledStrip = new Spark(0);
    private double yellow = 0.69; 
    private double purple = 0.91; 
    private double green = 0.77; 
    private double red = 0.61;
    private double blue = 0.87;
    private double teamColor;
    private String color = "";
    // private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    public LEDSubsystem(){
    // if (m_IntakeSubsystem.intakeSensor() == true) {
    //         teamColor = yellow;
    //         yellow();
    //     }
    // }
    }

    public void setLEDColor(double pwmColorCode) {
        ledStrip.set(pwmColorCode);
    }
    public void currentLEDColor() {
        ledStrip.get();
    }
    public void periodic() {
        SmartDashboard.putString("Current LED Color", color);
    }
    public void yellow() {
        setLEDColor(yellow);
        color = "yellow";
    }

    public void green() {
        setLEDColor(green);
        color = "green";
    }
    public void blue() {
        setLEDColor(blue);
        color = "blue";
    }
    public void red() {
        setLEDColor(red);
        color = "red";

    }
    public void purple() {
        setLEDColor(purple);
        color = "purple";
    }
    public void teamColor() {
        setLEDColor(teamColor);
        if(teamColor == red){
            color = "red";
        }
        else {
            color = "blue";
        }
    }
}
 