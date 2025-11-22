package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Vision.Vision;
import org.firstinspires.ftc.teamcode.commands.StaticTeleopDriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.function.Supplier;

public class Robot {
    private HardwareMap h;
    private Telemetry t;
    private Gamepad g1a, g2a, g1, g2;
    private final Turret tU;
    private final Shooter s;
    private final Intake i;
    private final Indexer iN;
    private final Vision v;
    private final MecanumDrive d;
    private final Follower f;
    private final StaticTeleopDriveCommand staticDriveCommand;
    private boolean automatedDrive = false;
    public static Pose startPose = new Pose(56,8,0, PedroCoordinates.INSTANCE);


    public Robot(HardwareMap h, Telemetry t, Gamepad g1a, Gamepad g2a) {

        this.h = h;
        this.t = t;
        this.g1a = g1a;
        this.g2a = g2a;

        tU = new Turret(this.h, this.t);
        s = new Shooter(this.h,this.t);
        i = new Intake(this.h,this.t);
        iN = new Indexer(this.h,this.t);
        v = new Vision(this.h,this.t);
        d = new MecanumDrive(this.h,this.t);

        f = Constants.createFollower(this.h);
        f.setStartingPose(startPose);

        this.g1 = new Gamepad();
        this.g2 = new Gamepad();

        // Initialize StaticTeleopDriveCommand with heading lock on right stick
        staticDriveCommand = new StaticTeleopDriveCommand(
                d,
                f,
                () -> -g1.left_stick_y,
                () -> g1.left_stick_x,
                () -> g1.left_stick_x,
                () -> -g1.right_stick_x,
                () -> 1.0
        );


    }

    public void Controls(){
        g1.copy(g1a);
        g2.copy(g2a);

        f.setPose(v.mt2(f.getHeading()));

        tU.setTurret(f.getPose().getX(),f.getPose().getY(),f.getPose().getHeading());

        Supplier<PathChain> pathChain = () -> f.pathBuilder()
                .addPath(new Path(new BezierLine(f::getPose, new Pose(38.4, 33))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(f::getHeading, Math.toRadians(0), 0.8))
                .build();


        if (g1.dpad_left && !automatedDrive) {
            f.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && !f.isBusy()) {
            f.startTeleopDrive();
            automatedDrive = false;
        }

        if (!automatedDrive) {
            // Use StaticTeleopDriveCommand with heading lock
            staticDriveCommand.execute();
        }


        if(g1.a){
            s.setVelocity();
            i.setPower(1);
        } else {
            i.setPower(0);
        }
        if (g1.x){
            iN.setPower(1);
        } else {
            iN.setPower(0);
        }
    }
    public void tPeriodic() {
        tU.periodic();
        iN.periodic();
        v.periodic();
        i.periodic();
        s.periodic();
        d.periodic();
        f.update();
        t.update();
    }
    public void tStart() {
        f.startTeleopDrive();
    }
}


