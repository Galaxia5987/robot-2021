package frc.robot.subsystems.color_wheel.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.color_wheel.ColorWheel;

/**
 * Spin the wheel until you reach the desired color.
 */
public class FindColor extends CommandBase {
    private final ColorWheel colorWheel;
    private final String color;
    private final double power;
    private String tempColor = "";
    private int initColorIndex;
    private int targetColorIndex;
    private int cutoff = 10;

    public FindColor(ColorWheel colorWheel, String color, double power) {
        this.colorWheel = colorWheel;
        this.color = color;
        this.power = power;
        addRequirements(colorWheel);
    }

    @Override
    public void initialize() {
        colorWheel.updateSensor();
        findInitialAndTargetColorPosition();
        findPathAndSetPower();
    }

    @Override
    public void execute() {
        colorWheel.updateSensor();
        if (!colorWheel.getColorString().equals(tempColor)) {
            tempColor = colorWheel.getColorString();
            colorWheel.setPower(power * cutoff-- / 10.);
        }
    }

    @Override
    public boolean isFinished() {
        return colorWheel.getColorString().equals(color);
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.setPower(0);
    }

    public void findPathAndSetPower() {
        int clockDistance, antiDistance;
        if (targetColorIndex < initColorIndex) {
            clockDistance = targetColorIndex + Constants.ColorWheel.COLORS.length - initColorIndex;
            antiDistance = initColorIndex - targetColorIndex;
        } else {
            clockDistance = targetColorIndex - initColorIndex;
            antiDistance = Constants.ColorWheel.COLORS.length - targetColorIndex + initColorIndex;
        }
        if (clockDistance < antiDistance) {
            colorWheel.setPower(power);
        } else {
            colorWheel.setPower(-power);
        }
    }

    private void findInitialAndTargetColorPosition() {
        tempColor = colorWheel.getColorString();
        initColorIndex = 0;
        targetColorIndex = 0;
        for (int i = 0; i < Constants.ColorWheel.COLORS.length; i++) {
            if (Constants.ColorWheel.COLORS[i].equals(tempColor))
                initColorIndex = i;
            if (Constants.ColorWheel.COLORS[i].equals(color))
                targetColorIndex = i;
        }
    }
}
