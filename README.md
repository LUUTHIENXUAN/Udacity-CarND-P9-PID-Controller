# Project 8: PID Controls Project

## 1. Introduction
PID(Proportional-Integral-Derivative) controllers implemented in C++ to maneuver the vehicle around the track. This project was done in the following steps:

+ Implement PID Controller for Steering
+ Optimize for each PID coefficient parameters for Steering by `twiddle` as follow:
  *  Get the average error (CTE) in 3(by default) steps in `run`.
  *  Feed this error to `twiddle` to adjust the PID coefficient
  *  Repeat those steps until meeting the tolerance requirement.

> **NOTE**: This approach is slightly different with the lecture. Instead of rerun the `twiddle` by reset the simulator, I use `twiddle` to optimize the current PID coefficients but while the car running.

>*I believe this approach is more practical for real vehicle and more beneficial if you already have your own PID coefficient.*

+ Implement PID Controller for Speed
  + Set up `reference_speed` to keep the car reach that speed
  + Set up cost function to control speed over `CTE`, vehicle 's current `angle`, next `steering_angle` so that the vehicle could slow down at tight turns, oscillations and overshooting  

## 2. Results & Discussion


#### Results:
A video of the simulated car driving around the track:

Youtube video recorded at 100 (mph). Click to view!
<div align="center">
  <a href="https://www.youtube.com/watch?v=U9JQCG4BG0U"><img src="https://img.youtube.com/vi/U9JQCG4BG0U/0.jpg" alt="100(mph)"></a>
</div>

#### Discussion:
In the end, implementation of code for a basic PID controller is fairly straightforward, and `twiddle` is like a magic for tunning parameters. Here is what I leaned from "PID" for stearing:

1.  `P(proportional)` component: the car will steer to the cross-track error, or CTE which how far the car away from the reference path in the road. The higher CTE then the higher steering angle should be taken into.

 - If this coefficient is set **too high**, the car will constantly overcorrect and overshoot the middle.

 - If this coefficient is set **too low**, the car will react slowly to heavy curves or high CTE.

2. `I(integral)` component:  sums up all CTEs up to that point. In case the car is one side of the reference path for the whole time, this component will encourage the car to turn back toward the reference path. In fact, this component should be small enough to prevent the affect of accumulation of CTE.

 - If this coefficient is set **too high**, the car tends to have quick oscillations.

 - If this coefficient is set **too low**, the car may stay in one side of the reference path for long periods of time.

3. `D(derivate)` component:  change in CTE from current value to the previous one. This component helps the car more stable in terms of  curve and moving outward the reference path (like zigzag) , because it would contribute higher steering angle to make the car comming back faster to reference path.

 - If this coefficient is set **too high**, the car may steer in large angle.

 - If this coefficient is set **too low**, the car tends to have quick oscillations and behave overshooting.

4. Set up `twiddle`:

 Although I have used `twiddle` for optimisation, the car needs to run stable in a few first frames. This requires a "so-so"(or good) initialization of PID coefficient parameters.
 This setup also depends on car's speed ofcourse. I have used PID for Speed but just for keeping the car up to the reference speed. This is a real challenge for `twiddle` for adjusting coefficient in case of high speed because the car will reach the reference speed soon.

 Below is `run`code and `twiddle` code used in this project.


```C++
double PID::run(double cte, int current_time_step){

	err += cte*cte;

	if (current_time_step == max_time_step) {

		double average_err = err/max_time_step; //average cross track error
		err = 0;

		return average_err;
	}
	else return err;
}
```

```C++
void PID::twiddle(double tol, double err){

  if (!twiddle_intialized) {

    best_error = err;
    std::cout << "Initializing best_error..." << best_error <<std::endl;
    // done initializing, no need to update uptate Twiddle
    twiddle_intialized = true;

    return;
	}
	if ((dparams[0] + dparams[1] + dparams[2]) < tol){

     std::cout << "Twiddle completed! "
               << (dparams[0] + dparams[1] + dparams[2])
               << " < " << tol
               << std::endl;
     std::cout << " Final Result:"
               << " Kp = " << params[0]
               << " Ki = " << params[1]
               << " Kd = " << params[2]
               << std::endl;

     twiddle_completed = true;
	}
	switch (current_stage) {

		case 0: {

			params[index]  += dparams[index];
			current_stage  += 1 ;

		}break;

		case 1: {

			double error_1 = err;
			if (error_1 < best_error){
				best_error      = error_1;
				dparams[index] *= 1.1;
			}
			else{
				params[index]  -= 2.0*dparams[index];
			}

			current_stage  += 1;

		}break;

		case 2: {

			double error_2 = err;
			if (error_2 < best_error){
				best_error      = error_2;
				dparams[index] *= 1.1;
			}
			else{
				params[index]  += dparams[index];
				dparams[index] *= 0.9;
			}

			//return to case 0
			current_stage = 0;
			//increase index
			index = (index < 2) ? index += 1 : 0;
		}break;

	}

	///* Update coefficients
	Kp = params[0];
	Ki = params[1];
	Kd = params[2];

}
```

## 3. Set up environment
### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

### Basic Build Instructions
```
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.
```
Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

### Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

### Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

### Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

### Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

### Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

### How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
