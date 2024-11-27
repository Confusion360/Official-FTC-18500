package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class Prep extends CommandBase {
	private final HorizontalSlides hSlides;
	private final Arm arm;

	public Prep(HorizontalSlides hSlides_p, Arm arm_p) {
		hSlides = hSlides_p;
		arm = arm_p;
		addRequirements(hSlides, arm);
	}

	@Override
	public void initialize() {
		hSlides.setHingePos(0.2);
		arm.moveArm(0.6);
		hSlides.move(0.5);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
