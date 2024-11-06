package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides;

public class HorizontalSlidesOtherLength extends CommandBase {
	private HorizontalSlides hSlides;
	private boolean direction;	//false - in, true- out

	public HorizontalSlidesOtherLength (HorizontalSlides hSlides_p, boolean out) {
		hSlides = hSlides_p;
		direction = out;
		addRequirements(hSlides);
	}

	@Override
	public void initialize() {
		//direction will be true if its extension
		if (direction) hSlides.extendFurther();
		else hSlides.extendFurther();
	}
}
