package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;

// command for moving the sample from the intake to the arm, getting it ready for elevation to the basket
public class IntakeToArmTransfer extends CommandBase{
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final HorizontalSlides hSlides;
	private final Arm arm;
	private final VerticalSlides vSlides;

	public IntakeToArmTransfer(Arm arm_p, HorizontalSlides hSlides_p, VerticalSlides vSlides_p) {
		hSlides = hSlides_p;
		arm = arm_p;
		vSlides = vSlides_p;
		addRequirements(hSlides, arm, vSlides);
	}

	@Override
	public void initialize() {
		vSlides.moveToPos(0);
		hSlides.intakeOff();
		hSlides.setHingePos(0.17);
		sleep(100);
		arm.moveArm(0.43);
		arm.moveHinge(0.43);
		sleep(450);
		hSlides.move(0.55);
		sleep(150);
		hSlides.move(0.5);
		sleep(150);
		arm.grab(1);
		sleep(500);
		arm.moveHinge(0.6);
		hSlides.move(0.6);
		sleep(500);
		arm.moveArm(0.7);
		hSlides.move(0.55);
	}

	public static void sleep(long milliseconds) {
		try {
			Thread.sleep(milliseconds);
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
		}
	}

	@Override
	public boolean isFinished() {return true;}
}
