package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides;

public class IntakeEject extends CommandBase {
	public HorizontalSlides hSlides;

	public IntakeEject (HorizontalSlides hSlides_p) {
		hSlides = hSlides_p;
		addRequirements(hSlides);
	}

	@Override
	public void initialize() {
		hSlides.intakeEject();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
