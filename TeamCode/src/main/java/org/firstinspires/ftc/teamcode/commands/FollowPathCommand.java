package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;


// Credit 21802 bc im lazy
public class FollowPathCommand extends CommandBase {

    private final Follower follower;
    private final PathChain path;
    private boolean holdEnd = true;
    Tuning tuning;

    public FollowPathCommand(Follower follower, PathChain path) {
        this.follower = follower;
        this.path = path;
    }

    public FollowPathCommand(Follower follower, Path path) {
        this(follower, new PathChain(path));
    }


    public FollowPathCommand setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    @Override
    public void initialize() {
        follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void execute() {
        follower.update();
        Tuning.drawCurrent();




    }
}