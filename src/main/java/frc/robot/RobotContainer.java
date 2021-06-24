package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commandgroups.ConveyorShooter;
import frc.robot.commandgroups.Outtake;
import frc.robot.commandgroups.PickupBalls;
import frc.robot.commandgroups.PickupBallsFeeder;
import frc.robot.subsystems.PTO.PTO;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.autonomous.MoveForward;
import frc.robot.subsystems.drivetrain.commands.MoveToPosition;
import frc.robot.subsystems.drivetrain.commands.Rotate;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.AdjustHood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.ToggleIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.ToggleVisionPiston;
import frc.robot.utils.VisionModule;
import frc.robot.valuetuner.ValueTuner;
import org.photonvision.LEDMode;
import org.techfire225.webapp.Webserver;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class RobotContainer {
    public static final PTO pto = new PTO();
    public static final VisionModule vision = new VisionModule();
    public static final SwerveDrive swerveDrive = new SwerveDrive(false);
    public static XboxController XboxDriver = new XboxController(3);
    public static XboxController Xbox = new XboxController(2);
    public final Funnel funnel = new Funnel();
    public final Shooter shooter = new Shooter();
    public final Climber climber = new Climber();
    public final Intake intake = new Intake();
    public final Hood hood = new Hood();
    public final Conveyor conveyor = new Conveyor();
    public JoystickButton a = new JoystickButton(Xbox, XboxController.Button.kA.value);
    public JoystickButton b = new JoystickButton(Xbox, XboxController.Button.kB.value);
    public JoystickButton y = new JoystickButton(Xbox, XboxController.Button.kY.value);
    public JoystickButton x = new JoystickButton(Xbox, XboxController.Button.kX.value);
    public JoystickButton RB = new JoystickButton(Xbox, XboxController.Button.kBumperRight.value);
    public JoystickButton LB = new JoystickButton(Xbox, XboxController.Button.kBumperLeft.value);
    public JoystickButton RT = new JoystickButton(Xbox, XboxController.Axis.kRightTrigger.value);
    public Joystick LT = new Joystick(XboxController.Axis.kLeftTrigger.value);
    public JoystickButton R = new JoystickButton(Xbox, XboxController.Button.kStickRight.value);
    public JoystickButton L = new JoystickButton(Xbox, XboxController.Button.kStickLeft.value);
    public JoystickButton start = new JoystickButton(Xbox, XboxController.Button.kStart.value);
    public JoystickButton back = new JoystickButton(Xbox, XboxController.Button.kBack.value);
    public JoystickButton xDriver = new JoystickButton(XboxDriver, XboxController.Button.kX.value);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureDefaultCommands();
        configureButtonBindings();
        vision.setLEDs(LEDMode.kOn);

        if (Robot.debug) {
            startValueTuner();
            startFireLog();
        }

    }


    private void configureDefaultCommands() {
//        swerveDrive.setDefaultCommand(new HolonomicDrive(swerveDrive));
        swerveDrive.setDefaultCommand(new Rotate(swerveDrive));
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        a.whileHeld(new PickupBalls(intake, funnel, conveyor, Constants.Intake.POWER::get, true));
        b.whileHeld(new MoveToPosition(swerveDrive, vision));
        x.whileHeld(new ConveyorShooter(shooter, hood, conveyor, funnel, vision, Constants.Conveyor.CONVEYOR_MOTOR_POWER));
        y.whileHeld(new PickupBallsFeeder(intake, funnel, conveyor, Constants.Intake.POWER::get));
        RB.whileHeld(new Outtake(funnel, conveyor, shooter));
        LB.whenPressed(new ToggleIntake(intake));
        R.whileHeld(new AdjustHood(hood, Hood.State.HIGH));
        back.whenPressed(new ToggleVisionPiston(vision));
        start.whenPressed(() -> vision.setLEDs(LEDMode.kOff));
//        L.whenPressed(() -> vision.setLEDs(LEDMode.kOn));
        xDriver.whenPressed(swerveDrive::lock);
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
//        return new ShootAndSafeTrench(swerveDrive, vision, shooter, hood, intake, funnel, conveyor);
//        return new ShootFromInitiation(swerveDrive, vision, funnel, conveyor, shooter, hood);
//        return new FiveNineEightSeven(swerveDrive);
        return new MoveForward(swerveDrive, 1).withTimeout(1);
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
