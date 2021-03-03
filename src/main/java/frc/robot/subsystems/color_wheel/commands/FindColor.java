package frc.robot.subsystems.color_wheel.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.color_wheel.ColorWheel;

/**
 * Spin the wheel until you reach the desired color.
 */

public class FindColor extends CommandBase {
    private final ColorWheel colorWheel;
    private final String targetColor;
    private final double power;
    private String prevColor;
    private int cutoff = 10;

    public FindColor(ColorWheel colorWheel, String targetColor, double power) {
        this.colorWheel = colorWheel;
        this.targetColor = targetColor;
        this.power = power;
        addRequirements();
    }

    @Override
    public void initialize() {
        colorWheel.updateSensor();
        String initColor = colorWheel.getColorString();
        switch (initColor) {
            case "RED":
                switch (targetColor) {
                    case "GREEN":
                    case "BLUE":
                        colorWheel.setPower(power);
                        break;
                    case "YELLOW":
                        colorWheel.setPower(-power);
                }
                break;
            case "GREEN":
                switch (targetColor) {
                    case "RED":
                        colorWheel.setPower(-power);
                        break;
                    case "BLUE":
                    case "YELLOW":
                        colorWheel.setPower(power);
                }
                break;
            case "BLUE":
                switch (targetColor) {
                    case "RED":
                    case "YELLOW":
                        colorWheel.setPower(power);
                        break;
                    case "GREEN":
                        colorWheel.setPower(-power);
                }
                break;
            case "YELLOW":
                switch (targetColor) {
                    case "RED":
                    case "GREEN":
                        colorWheel.setPower(power);
                        break;
                    case "BLUE":
                        colorWheel.setPower(-power);
                }
                break;
        }
        prevColor = initColor;
    }

    @Override
    public void execute() {
        colorWheel.updateSensor();
        if (!colorWheel.getColorString().equals(prevColor)) {
            colorWheel.setPower(power * --cutoff / 10.);
            prevColor = colorWheel.getColorString();
        }
    }

    @Override
    public boolean isFinished() {
        return colorWheel.getColorString().equals(targetColor);
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.setPower(0);
    }
}
