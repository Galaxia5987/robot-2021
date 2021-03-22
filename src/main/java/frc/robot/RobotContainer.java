/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commandgroups.PickupBalls;
import frc.robot.subsystems.PTO.PTO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.StartIntake;
import frc.robot.subsystems.intake.commands.ToggleIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedShooter;
import frc.robot.subsystems.conveyor.commands.LoadConveyor;
import frc.robot.valuetuner.ValueTuner;
import webapp.Webserver;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.commands.StartFunnel;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public Funnel funnel = new Funnel();
    public static final PTO pto = new PTO();
    private final Shooter shooter = new Shooter();
    public XboxController Xbox = new XboxController(1);
    public JoystickButton a = new JoystickButton(Xbox, XboxController.Button.kA.value);
    public JoystickButton b = new JoystickButton(Xbox, XboxController.Button.kB.value);
    // The robot's subsystems and commands are defined here...
    public Intake intake = new Intake();
    public JoystickButton BL = new JoystickButton(Xbox, XboxController.Button.kBumperLeft.value);
    public JoystickButton BR = new JoystickButton(Xbox, XboxController.Button.kBumperRight.value);
    public JoystickButton y = new JoystickButton(Xbox, XboxController.Button.kY.value);
    public Conveyor conveyor = new Conveyor();


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        if (Robot.debug) {
            startValueTuner();
            startFireLog();
        }
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
//        a.whenPressed(new ToggleIntake(intake));
//        BR.whileHeld(new StartIntake(intake, true));//transfers the balls to the Funnel
//        BL.whileHeld(new StartFunnel(funnel,true));
//
//        b.whileHeld(new FeedShooter(conveyor, 0.7));
//        y.whileHeld(new LoadConveyor(conveyor, 0.7));
        a.whileHeld(new PickupBalls(intake, funnel, conveyor));
  }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // An ExampleCommand will run in autonomous
        return null;
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
