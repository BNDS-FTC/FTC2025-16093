package org.firstinspires.ftc.teamcode.references;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ColorIdentification {

    // 可调试优化的权重因子
    private final float RED_WEIGHT;
    private final float BLUE_WEIGHT;
    private final float GREEN_WEIGHT;
    private final float TOTAL_WEIGHT;
    public static double error = -1;

     // 定义目标颜色的 RGB 分量（红、蓝、黄）
    public static float[] targetRed = {0.0011f, 0.0009f, 0.00052f}; // 红色
    public static float[] targetBlue = {0.00037f, 0.00077f, 0.00135f}; // 蓝色
    public static float[] targetYellow = {0.0017f, 0.002f, 0.00055f} ; // 黄色

    /**
     * 判断颜色最接近红色、蓝色还是黄色
     * 各个权重代表当前通道在计算距离时的重要性，仅和权重的比值有关，权重的绝对值不影响结果
     */
    public ColorIdentification(float red_weight, float green_weight, float blue_weight) {
        this.RED_WEIGHT = red_weight;
        this.GREEN_WEIGHT = green_weight;
        this.BLUE_WEIGHT = blue_weight;
        this.TOTAL_WEIGHT = red_weight + green_weight + blue_weight;
    }
    /**
     * 判断颜色最接近红色、蓝色还是黄色
     * @param red 红色分量，范围 [0,1)
     * @param green 绿色分量，范围 [0,1)
     * @param blue 蓝色分量，范围 [0,1)
     * @return 最接近的颜色名称（red, blue, 或 yellow）
     */
    public String getClosestColor(float red, float green, float blue, float alpha) {


        // 计算与各目标颜色之间的欧几里得距离
        double distRed = calculateDistance(red, green, blue, targetRed);
        double distBlue = calculateDistance(red, green, blue, targetBlue);
        double distYellow = calculateDistance(red, green, blue , targetYellow);

        // 找到最短距离对应的颜色
        if (distRed < distBlue && distRed < distYellow) {
            return "red";
        } else if (distBlue < distRed && distBlue < distYellow) {
            return "blue";
        } else {
            return "yellow";
        }
    }

    // 计算颜色之间的欧几里得距离
    private double calculateDistance(float r, float g, float b, float[] target) {
        double diffR = Math.abs(r - target[0]);
        double diffG = Math.abs(g - target[1]);
        double diffB = Math.abs(b - target[2]);
        this.error = (diffR * RED_WEIGHT + diffG * GREEN_WEIGHT + diffB * BLUE_WEIGHT)
                    /TOTAL_WEIGHT;
        return error;
    }
}