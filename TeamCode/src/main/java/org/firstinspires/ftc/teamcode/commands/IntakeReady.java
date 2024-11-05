package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class IntakeReady extends CommandBase{
	private final HorizontalSlides hSlides;

	public IntakeReady(HorizontalSlides hSlides_p) {
		hSlides = hSlides_p;
		addRequirements(hSlides);
	}

	@Override
	public void initialize() {
		hSlides.move(0.67);
		sleep(150);
		hSlides.setHingePos(0.49);
		hSlides.intakeOn();
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
