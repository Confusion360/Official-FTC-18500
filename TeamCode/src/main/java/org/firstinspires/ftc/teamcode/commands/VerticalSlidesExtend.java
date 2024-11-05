package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class VerticalSlidesExtend extends CommandBase {

	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final VerticalSlides vSlides;
	private final Arm arm;

	public VerticalSlidesExtend(Arm arm_p, VerticalSlides vSlides_p) {
		vSlides = vSlides_p;
		arm = arm_p;
		addRequirements(arm, vSlides);
	}

	@Override
	public void initialize() {
		vSlides.moveToPos(2300);
		arm.moveArm(0.8);
		arm.moveHinge(0.4);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
