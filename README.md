# QuadTargetFSM

![What a landing target looks like](https://raw.githubusercontent.com/erinaceous/QuadTargetFSM/master/Target.png)

Owain Jones <contact@odj.me>, MSc Dissertation project

C++ and OpenCV implementation of quadcopter target tracking algorithm.
This is my cleaned up version that uses a finite state machine to detect the
marker patterns. Working slightly better (AKA can actually track targets)
and more efficiently and less buggily than the original implementation.

It has a number of variables that can be tweaked -- see `cfg/defaults.ini` for
detailed comments on all of the algorithms' parameters. By default it is tuned
for detecting targets in outdoors lighting using a Raspberry Pi NoIR camera.
