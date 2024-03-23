package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
public class CommandsContainer {

    private double driveSpeed = 0.50;
    public boolean fieldRelative = true;
    public Command defaultDriveCommand(DriveSubsystem drive, Joystick controller, XboxController controller2) {
        return new RunCommand(
            () -> {
                fieldRelative = controller.getRawAxis(5) > 0;
                driveSpeed = Math.max(0.75 + (0.25 * controller.getRawAxis(7)), (0.5 + (controller2.getRightTriggerAxis() * 0.5)));
                drive.drive(
                -driveSpeed *  modifyAxis(controller.getRawAxis(2)),
                driveSpeed * modifyAxis(controller.getRawAxis(3)),
                driveSpeed *  modifyAxis(controller.getRawAxis(0)),
                fieldRelative, true);},
            drive);
        
    }

    // public Command defaultLiftCommand(LiftSubsystem lift, Joystick controller)  {
    //     return new RunCommand(
    //         () -> lift.control(controller.getRawAxis(6)));
    // }

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
