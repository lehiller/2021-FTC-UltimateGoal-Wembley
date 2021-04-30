package org.firstinspires.ftc.teamcode.autos;


/**
 * The purpose of this interface is to allow the robot to do multiple things without having to rewrite loops.
 * Simply have a loop use a function, and have the function be whatever action is required.
 */

public interface MultiTask {

    void execute();
    void stop();
}

