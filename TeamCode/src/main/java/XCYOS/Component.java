package XCYOS;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Component {
    void setUp(HardwareMap hardwareMap);
    void update();
}
