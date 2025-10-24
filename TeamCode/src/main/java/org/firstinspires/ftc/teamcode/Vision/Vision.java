
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

    private final Limelight3A camera;
    @Getter private LLResult result;

    public static double TURN_P = 0.05;



    private Bot bot;


    public Vision(Bot bot) {
        this.bot = bot;
        camera = bot.hMap.get(Limelight3A.class, "limelight");

        camera.pipelineSwitch(0);
        initializeCamera();

    }

    public void initializeCamera() {
        camera.setPollRateHz(100);
        camera.start();
    }

    public double getTurnPower() {
        if (!result.isValid()) {
            return 0.0;
        }
        double tx = getTx();

        return -tx * TURN_P;
    }

    public double getTx() {
        if (result == null) {
            return 0;
        }
        return result.getTx();
    }

    public double getTy() {
        if (result == null) {
            return 0;
        }
        return result.getTy();
    }





    @Override
    public void periodic() {
        result = camera.getLatestResult();

        if (result != null) {

            bot.telem.addData("Turn Power", getTurnPower());
            bot.telem.addData("Ty", getTy());
            bot.telem.addData("Tx", getTx());
        }



    }


}
