
package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;


@Config
@Autonomous
public class AutoRed extends LinearOpMode {

    public static Pose startingPose = new Pose(105.5579,132.711, Math.toRadians(0));
    double heading = 0;
    // 88.5, 7.3        105.5579,132.711
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;


    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();


        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);

        Follower f = Constants.createFollower(bot.hMap);
        f.setStartingPose(startingPose);
        f.update();

        Intake i = new Intake(hardwareMap, telem);
        i.register();

        Shooter s = new Shooter(hardwareMap, telem);
        s.register();

        Turret t = new Turret(hardwareMap,telem);
        t.resetEncoders();
        t.register();

        Indexer iN = new Indexer(hardwareMap, telem);
        iN.register();


        SequentialCommandGroup auto = new SequentialCommandGroup(

                new SequentialCommandGroup(


                                new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                         .addPath(
                                                 new BezierLine(new Pose(105.5579,132.711), new Pose(97.382, 100.4597))

                                         )
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),

                                new InstantCommand(()->{i.setPower(1);}),
                                new InstantCommand(()->{iN.setPower(1);}),
                                new WaitCommand(4500),
                                new InstantCommand(()->{iN.setPower(0);})

                                ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(new Pose(97.382, 100.4597), new Pose(97.382, 83.000))

                                        )
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                               new InstantCommand(()->{i.setPower(1);}),
                                new InstantCommand(()->{f.setMaxPower(0.75);})

                                ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(new Pose(97.382, 83.000), new Pose(125.000, 83.000))

                                        )
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new InstantCommand(()->{f.setMaxPower(1);})



                        ),

                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(new Pose(125.000, 83.000),new Pose(97.382, 100.4597))

                                        )
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(750),

                                new InstantCommand(()->{i.setPower(1);}),
                                new InstantCommand(()->{iN.setPower(1);}),
                                new WaitCommand(4500),
                                new InstantCommand(()->{iN.setPower(0);})

                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(new Pose(97.382, 100.4597), new Pose(97.382, 58.500))
                                        )
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(()->{i.setPower(1);}),
                                new InstantCommand(()->{f.setMaxPower(0.75);})


                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(new Pose(97.382, 58.500), new Pose(133.000, 58.500))

                                        )
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new InstantCommand(()->{f.setMaxPower(1);})


                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(new Pose(133.000, 58.500), new Pose(97.382, 58.09090909090909))

                                        )
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(new Pose(97.382, 58.09090909090909),new Pose(97.382, 100.4597))

                                        )
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),

                                new InstantCommand(()->{i.setPower(1);}),
                                new InstantCommand(()->{iN.setPower(1);}),
                                new WaitCommand(4500),
                                new InstantCommand(()->{iN.setPower(0);})

                        )
                )
        );

        waitForStart();
        CommandScheduler.getInstance().schedule(auto);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            f.update();

            PoseStorage.currentPose = f.getPose();

            t.setTurretRED(f.getPose().getX(),f.getPose().getY(),f.getPose().getHeading());
            s.setVelocityRED(f.getPose().getX(),f.getPose().getY());
            telem.addData("POSE", f.getPose());
            telem.update();


        }
    }
}