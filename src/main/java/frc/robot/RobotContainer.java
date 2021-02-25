package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.*;
import frc.robot.valuetuner.ValueTuner;
import org.techfire225.webapp.Webserver;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class RobotContainer {
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public SwerveDrive swerveDrive = new SwerveDrive(true, false);

    public RobotContainer(){
        configureButtonBindings();

        Shuffleboard.getTab("Autonomous").add(m_chooser);
        if (Robot.debug) {
            startValueTuner();
            startFireLog();
        }
    }

    private void configureButtonBindings() {
        // Grab the hatch when the 'A' button is pressed.
        swerveDrive.setDefaultCommand(new HolonomicDrive(swerveDrive));
        OI.c.whenPressed(new InstantCommand(swerveDrive::resetAllEncoders, swerveDrive));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomous() {
        return m_chooser.getSelected();
    }

    public Command getAutonomousCommand() {
        return new HolonomicDrive(swerveDrive);
    }


    /**
     * Initiates the value tuner.
     */
    private void startValueTuner() {
        new ValueTuner().start();
    }

    /**
     * Initiates the port of team 225s Fire-Logger.
     */
    private void startFireLog() {
        try {
            new Webserver();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
