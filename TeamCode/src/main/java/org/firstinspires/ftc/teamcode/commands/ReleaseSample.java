package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class ReleaseSample extends CommandBase{
	private final Arm arm;
	private final VerticalSlides vSlides;

	public ReleaseSample(Arm arm_p, VerticalSlides vSlides_p){
		arm = arm_p;
		vSlides = vSlides_p;
		addRequirements(arm, vSlides);
	}

	@Override
	public void initialize() {
		arm.release();
		sleep(1000);
		arm.moveArm(0.7);
		vSlides.moveToPos(0);
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	public static void sleep(long milliseconds) {
		try {
			Thread.sleep(milliseconds);
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
		}
	}
}
