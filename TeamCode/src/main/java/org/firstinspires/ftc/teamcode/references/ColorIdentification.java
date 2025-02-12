package org.firstinspires.ftc.teamcode.references;

public class ColorIdentification {

    // 可调试优化的权重因子
    private final float RED_WEIGHT;
    private final float BLUE_WEIGHT;
    private final float YELLOW_WEIGHT;

    /**
     * 判断颜色最接近红色、蓝色还是黄色
     * @param red_weight 红色的权重,权重越大越容易识别为红色
     * @param green_weight 绿色的权重,权重越大越容易识别为绿色
     * @param blue_weight 蓝色的权重,权重越大越容易识别为蓝色
     */
    public ColorIdentification(float red_weight, float green_weight, float blue_weight) {
        this.RED_WEIGHT = red_weight;
        this.BLUE_WEIGHT = green_weight;
        this.YELLOW_WEIGHT = blue_weight;
    }
    /**
     * 判断颜色最接近红色、蓝色还是黄色
     * @param red 红色分量，范围 [0,1)
     * @param green 绿色分量，范围 [0,1)
     * @param blue 蓝色分量，范围 [0,1)
     * @return 最接近的颜色名称（red, blue, 或 yellow）
     */
    public String getClosestColor(float red, float green, float blue) {
        // 定义目标颜色的 RGB 分量（红、蓝、黄）
        float[] targetRed = {1.0f, 0.0f, 0.0f}; // 红色
        float[] targetBlue = {0.0f, 0.0f, 1.0f}; // 蓝色
        float[] targetYellow = {1.0f, 1.0f, 0.0f}; // 黄色

        // 计算与各目标颜色之间的欧几里得距离
        double distRed = calculateDistance(red, green, blue, targetRed) / RED_WEIGHT;
        double distBlue = calculateDistance(red, green, blue, targetBlue) / BLUE_WEIGHT;
        double distYellow = calculateDistance(red, green, blue, targetYellow) / YELLOW_WEIGHT;

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
        double diffR = r - target[0];
        double diffG = g - target[1];
        double diffB = b - target[2];
        return Math.sqrt(diffR * diffR + diffG * diffG + diffB * diffB);
    }
}