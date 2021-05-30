package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commandgroups.ShootAndAdjust;
import frc.robot.subsystems.PTO.PTO;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedShooter;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.autonomous.FollowPath;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.AdjustHood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.*;
import frc.robot.utils.VisionModule;
import frc.robot.valuetuner.ValueTuner;
import frc.robot.valuetuner.WebConstant;
import org.photonvision.LEDMode;
import org.techfire225.webapp.Webserver;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class RobotContainer {
    public static final PTO pto = new PTO();
//    public static final WebConstant velocity = new WebConstant("velocity", 69);
    public static final VisionModule vision = new VisionModule();
    //    public VisionModule visionModule = new VisionModule();
    public static XboxController Xbox = new XboxController(2);
    public static XboxController XboxDriver = new XboxController(3);
    public Funnel funnel = new Funnel();
    public Shooter shooter = new Shooter();
    public Climber climber = new Climber();
    public Hood hood = new Hood();
    public JoystickButton a = new JoystickButton(Xbox, XboxController.Button.kA.value);
    public JoystickButton b = new JoystickButton(Xbox, XboxController.Button.kB.value);
    public JoystickButton x = new JoystickButton(Xbox, XboxController.Button.kX.value);
    public JoystickButton xDriver = new JoystickButton(XboxDriver, XboxController.Button.kX.value);
    // The robot's subsystems and commands are defined here...
    public Intake intake = new Intake();
    public JoystickButton BL = new JoystickButton(Xbox, XboxController.Button.kBumperLeft.value);
    public JoystickButton BR = new JoystickButton(Xbox, XboxController.Button.kBumperRight.value);
    public JoystickButton y = new JoystickButton(Xbox, XboxController.Button.kY.value);
    public Conveyor conveyor = new Conveyor();
    public JoystickButton LT = new JoystickButton(Xbox, XboxController.Button.kStickLeft.value);
    public JoystickButton start = new JoystickButton(Xbox, XboxController.Button.kStart.value);
    public JoystickButton stop = new JoystickButton(Xbox, XboxController.Button.kBack.value);

    public SwerveDrive swerveDrive = new SwerveDrive(false, false);

    public WebConstant visionOutput = new WebConstant("vision-output", 0);

    private Trajectory trajectory = new Trajectory();


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureDefaultCommands();
        configureButtonBindings();
        if (Robot.debug) {
            startValueTuner();
            startFireLog();
        }

    }


    private void configureDefaultCommands() {

//        swerveDrive.setDefaultCommand(new HolonomicDrive(swerveDrive));
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        a.whenPressed(new AdjustHood(hood, Hood.State.HIGH));
        b.whenPressed(new AdjustHood(hood, Hood.State.LOW));
        x.whenPressed(new AdjustHood(hood, Hood.State.MIDDLE));
        y.whileHeld(new FeedShooter(conveyor, Constants.Conveyor.CONVEYOR_MOTOR_POWER));
        BR.whileHeld(new ShootAndAdjust(shooter, hood));
        start.whenPressed(() -> VisionModule.setLEDs(LEDMode.kOff));
        stop.whenPressed(() -> VisionModule.setLEDs(LEDMode.kOn));
        BL.whenPressed(new ToggleVisionPiston(shooter));
//        a.whileHeld(new PickupBalls(intake, funnel, conveyor, visionOutput::get, true));
//        b.whileHeld(new ShootWithoutVision(shooter));
//        x.whileHeld(new FeedShooter(conveyor, Constants.Conveyor.CONVEYOR_MOTOR_POWER));
//        y.whileHeld(new Outtake(funnel, conveyor));
//        BR.whenPressed(new ToggleIntake(intake));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new FollowPath(swerveDrive, trajectory);
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
