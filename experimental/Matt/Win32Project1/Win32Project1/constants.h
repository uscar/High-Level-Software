#pragma once

namespace simvals {
	//-----------------------HARD CONSTANTS-----------------------------------------
	//math constants
	const long double pi = atan2(0, -1);
	const int randomMax = 64;

	//roomba software behavior parameters

	//roomba speed
	const double robotSpeed = (double)330 / (double)1000; // m/s
	// Time between trajectory noise injections
	const double noiseInterval = (double)5000 / (double)1000;
	// Time between auto-reverse
	const double reverseInterval = (double)20000 / (double)1000;
	// Time needed to affect trajectory
	const double noiseLength = (double)850 / (double)1000;
	// Time needed to reverse trajectory
	const double reverseLength = (double)2456 / (double)1000; // .33/2 m/s * pi * wheelbase / 2
	// Time needed to spin 45 degrees
	const double topTouchTime = reverseLength / 4;

	//physical system constants
	const int numroombas = 10;
	const double roombaDiameter = 0.34;
	const double wheelbase_width = 0.25798379655423864; //worked backwards to this

	//const double FOV; //we need this value to find the vield of view as a function of height

	//-----------------------PARAMETERS---------------------------------------------

	//debug stuff
	const bool bestmove_test = false;

	//prediction parameters
	const int branching = 5;
	double predictUntil = 15.0; //10s predictive capacity?

	//simulation parameters
	const int num_sims = 10;//we'll find a better way to manage the simulations later
	const double runendtime = 600; //default state end time

	//quad interaction parameters
	const double quadSpeedEst = 3.0; //m/s
	const double quadvSpeedEst = 1;
	const double quadvSpeedEstInteracting = -quadSpeedEst / 3;
	const double topTouchTimereq = 100;
	const double bumperTimereq = 100;
	const double roombaTapHeight = 0.05; //just a guess, i believe
	const double roombaBumpHeight = 0.0;

	//quad behavior parameters
	const double cruiseHeight = 1.0;

	//calculation parameters
	const double newton_accuracy = 0.1; //within 10 cm for now
	const int newton_iterations = 0;
}
