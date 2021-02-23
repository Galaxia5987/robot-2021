package frc.robot.subsystems.color_wheel.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.color_wheel.ColorWheel;

/**
 * Spin the wheel 3 times.
 */
public class Spin extends CommandBase {

    private final ColorWheel colorWheel;
    private final double power;
    private String previousColor; //the first color the sensor sees.
    private String currentColor; //the first color the sensor sees.
    private int differentColorCounter = 0;

    public Spin(ColorWheel colorWheel, double power) {
        this.colorWheel = colorWheel;
        this.power = power;
        addRequirements(colorWheel);
    }

    @Override
    public void initialize() {
        colorWheel.updateSensor();
        previousColor = colorWheel.getColorString();
        colorWheel.setPower(power);
    }

    @Override
    public void execute() {
        colorWheel.updateSensor();
        currentColor = colorWheel.getColorString();
        updateDifferentColorCount();
        moderatePower();
    }

    @Override
    public boolean isFinished() {
        return differentColorCounter >= Constants.ColorWheel.REQUIRED_SPINS * Constants.ColorWheel.COLOR_WHEEL_SLOTS;
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.setPower(0);
    }

    private void updateDifferentColorCount() {
        if (!currentColor.equals(previousColor)) {
            previousColor = currentColor;
            differentColorCounter++;
        }
    }

    private void moderatePower() {
        if (differentColorCounter == Constants.ColorWheel.REQUIRED_SPINS * Constants.ColorWheel.COLOR_WHEEL_SLOTS - 1)
            colorWheel.setPower(Constants.ColorWheel.REDUCE_POWER_BY * power);
        else
            colorWheel.setPower(power);
    }
}
