package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
public class CommandsContainer {

    private double driveSpeed = 0.40;
    public boolean fieldRelative = true;
    public Command defaultDriveCommand(DriveSubsystem drive, Joystick controller) {
        return new RunCommand(
            () -> {
                fieldRelative = controller.getRawAxis(5) > 0;
                driveSpeed = 0.7 + (0.3 * controller.getRawAxis(7));
                drive.drive(
                -driveSpeed *  modifyAxis(controller.getRawAxis(2)),
                driveSpeed * modifyAxis(controller.getRawAxis(3)),
                driveSpeed *  modifyAxis(controller.getRawAxis(0)),
                fieldRelative, true);},
            drive);
        
    }

    public Command defaultLiftCommand(LiftSubsystem lift, Joystick controller)  {
        return new RunCommand(
            () -> lift.control(controller.getRawAxis(6)));
    }

    public Command defaultAutonomousCommand(RobotContainer robot, DriveSubsystem drive, Constants constants) {
        return null;
        
    }
    

    
    private static double modifyAxis(double value) {
    // Deadband
    value = -MathUtil.applyDeadband(value, OIConstants.kDriveDeadband);
    
    // Square the axis, keep its sign.
    value = Math.copySign(value * value, value);
    
    return value;

  }
}
