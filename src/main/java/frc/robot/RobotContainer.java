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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.PTO.PTO;
import frc.robot.subsystems.PTO.commands.SwitchSubsystems;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.commands.ManageClimb;
import frc.robot.subsystems.climber.commands.SetDrum;
import frc.robot.subsystems.climber.commands.SetStopper;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.VisionModule;
import frc.robot.utils.vision_commands.TurnOffLeds;
import frc.robot.valuetuner.ValueTuner;
import frc.robot.valuetuner.WebConstant;
import org.photonvision.LEDMode;
import webapp.Webserver;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static final PTO pto = new PTO();
    public static final WebConstant velocity = new WebConstant("velocity", 0);
    public Funnel funnel = new Funnel();
    public Shooter shooter = new Shooter();
    public Climber climber = new Climber();
    public XboxController Xbox = new XboxController(1);
    public JoystickButton a = new JoystickButton(Xbox, XboxController.Button.kA.value);
    public JoystickButton b = new JoystickButton(Xbox, XboxController.Button.kB.value);
    public JoystickButton x = new JoystickButton(Xbox, XboxController.Button.kX.value);
    // The robot's subsystems and commands are defined here...
    public Intake intake = new Intake();
    public JoystickButton BL = new JoystickButton(Xbox, XboxController.Button.kBumperLeft.value);
    public JoystickButton BR = new JoystickButton(Xbox, XboxController.Button.kBumperRight.value);
    public JoystickButton y = new JoystickButton(Xbox, XboxController.Button.kY.value);
    public Conveyor conveyor = new Conveyor();
    public VisionModule visionModule = new VisionModule();
    public JoystickButton LT = new JoystickButton(Xbox, XboxController.Button.kStickLeft.value);


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
//        a.whileHeld(new PickupBalls(intake, funnel, conveyor));
//        b.whenPressed(new SwitchSubsystems(pto, false));
//        y.whileHeld(new FeedAndShoot(conveyor, shooter, 0.4));
//        a.whenPressed(new InstantCommand(() -> {
//            VisionModule.camera.setLED(LEDMode.kOff);
//            System.out.println("Hey");
//        }));
//        BR.whileHeld(new FeedShooter(conveyor, 0.7));
//        a.whenPressed(new SetStopper(climber, Climber.PistonMode.CLOSED));
//        b.whenPressed(new SetStopper(climber, Climber.PistonMode.OPEN));
//        x.whenPressed(new ManageClimb(climber, 0));
//        y.whenPressed(new ManageClimb(climber, 0.2));
        BR.whenPressed(new SwitchSubsystems(pto, true));
        BL.whenPressed(new SwitchSubsystems(pto, false));
//        a.whenPressed(new InstantCommand(() -> VisionModule.camera.setLED(LEDMode.kOff)));

        a.whenPressed(new SetStopper(climber, Climber.PistonMode.OPEN));
        b.whenPressed(new SetStopper(climber, Climber.PistonMode.CLOSED));
        x.whenPressed(new SetDrum(climber, Climber.PistonMode.OPEN));
        y.whenPressed(new SetDrum(climber, Climber.PistonMode.CLOSED));
        LT.whenPressed(new TurnOffLeds());

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
