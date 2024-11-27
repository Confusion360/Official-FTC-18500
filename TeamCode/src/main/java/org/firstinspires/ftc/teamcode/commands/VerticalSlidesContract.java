package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class VerticalSlidesContract extends CommandBase {

	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final VerticalSlides vSlides;
	private final Arm arm;

	public VerticalSlidesContract(Arm arm_p, VerticalSlides vSlides_p) {
		vSlides = vSlides_p;
		arm = arm_p;
		addRequirements(arm, vSlides);
	}

	@Override
	public void initialize() {
		vSlides.moveToPos(0);
		arm.moveArm(1);
		arm.moveHinge(0.45);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
