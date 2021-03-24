package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.*;
import frc.robot.valuetuner.ValueTuner;
import org.techfire225.webapp.Webserver;
import frc.robot.valuetuner.WebConstant;
import org.photonvision.LEDMode;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
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

    public SwerveDrive swerveDrive = new SwerveDrive(false, false);

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

  }

    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

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
