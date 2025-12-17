
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
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;


@Config
@Autonomous
public class AutoExample extends LinearOpMode {

    public static Pose startingPose = new Pose(55.75, 7.3, Math.toRadians(0));
    public static Pose heading = new Pose(55.75, 7.3, Math.toRadians(180));

    private Follower follower;
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




        SequentialCommandGroup auto = new SequentialCommandGroup(

                new SequentialCommandGroup(

                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                         .addPath(
                                                new BezierLine(
                                                        startingPose, new Pose(55.75, 106.18698347107438)
                                        )
                                        )
                                        .setConstantHeadingInterpolation(0)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(3000)
                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Pose(55.75, 106.18698347107438), new Pose(39.7422520661157, 85.73708677685951)
                                                )
                                        )
                                        .setConstantHeadingInterpolation(heading.getHeading())
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                               new InstantCommand(()->{i.setPower(1);})
                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Pose(39.7422520661157, 85.73708677685951), new Pose(21.5 , 85.73708677685951)
                                                )
                                        )
                                        .setConstantHeadingInterpolation(heading.getHeading())
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Pose(21.5, 85.73708677685951),  new Pose(55.75, 106.18698347107438)
                                                )
                                        )
                                        .setConstantHeadingInterpolation(heading.getHeading())
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(3000)
                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Pose(55.75, 106.18698347107438),  new Pose(55.75, 63.4)
                                                )
                                        )
                                        .setConstantHeadingInterpolation(heading.getHeading())
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(()->{i.setPower(1);})
                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Pose(55.75, 63.4), new Pose(14, 63.4)
                                                )
                                        )
                                        .setConstantHeadingInterpolation(heading.getHeading())
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Pose(14, 63.4),new Pose(60, 63.4)
                                                )
                                        )
                                        .setConstantHeadingInterpolation(heading.getHeading())
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Pose(60, 63.4), new Pose(55.75, 106.18698347107438)
                                                )
                                        )
                                        .setConstantHeadingInterpolation(heading.getHeading())
                                        .build()
                                )
                        )


                )
        );

        waitForStart();
        CommandScheduler.getInstance().schedule(auto);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            f.update();

            telem.addData("POSE", f.getPose());
            telem.update();


        }
    }
}
