
package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ReadWriteFile;

import lombok.Getter;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

import java.io.File;

@Config
public class Vision extends SubsystemBase {
    private final PIDFController pid = new PIDFController(0.2, 0, 0.0, 0.0);

    private final Limelight3A camera;
    @Getter private boolean isDataOld = false;
    @Getter private LLResult result;

    public static double TURN_P = 0.2;



    private Bot bot;


    public Vision(Bot bot) {
        this.bot = bot;
        camera = bot.hMap.get(Limelight3A.class, "limelight");

        camera.pipelineSwitch(0);
    }

    public void initializeCamera() {
        camera.setPollRateHz(100);
        camera.start();
    }

    public double getTurnPower() {
        if (!isTargetVisible()) {
            return 0.0;
        }
        double tx = getTx(0.0);

        double turnPower = tx * TURN_P;

        return turnPower;
    }

    public double getTx(double defaultValue) {
        if (result == null) {
            return defaultValue;
        }
        return result.getTx();
    }

    public double getTy(double defaultValue) {
        if (result == null) {
            return defaultValue;
        }
        return result.getTy();
    }

    public boolean isTargetVisible() {
        if (result == null) {
            return false;
        }
        return !MathUtils.isNear(0, result.getTa(), 0.0001);
    }



    @Override
    public void periodic() {
        /*result = camera.getLatestResult();

        if (result != null) {

            long staleness = result.getStaleness();
            isDataOld = staleness >= 100;
            bot.telem.addData("Distance", getDistance());
            bot.telem.addData("Angle", getTurnServoDegree());

        }*/
    }
}
