package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class VerticalSlidesPrep extends CommandBase {
	private final VerticalSlides vSlides;
	private final Arm arm;

	public VerticalSlidesPrep(VerticalSlides vSlides_p, Arm arm_p) {
		vSlides = vSlides_p;
		arm = arm_p;
		addRequirements(vSlides, arm);
	}

	@Override
	public void initialize() {
		vSlides.moveToPos(2030);
		arm.moveArm(0.5);
		arm.moveHinge(0);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
