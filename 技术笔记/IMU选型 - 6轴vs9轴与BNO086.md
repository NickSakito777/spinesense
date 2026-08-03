脊柱侧弯检测对漂移很敏感，此外要接入多个传感器，涉及到实时通信和算力分配
  
1. 传感器相关
    
    1. 选择6还是9轴？
        
        **6轴 (Accel + Gyro):** 只能测倾斜角 (Roll/Pitch)。
        
        **Yaw (航向角/垂直轴旋转) 会随时间漂移**，无法测量绝对方向。
        
          
        
        **9轴 (Accel + Gyro + Mag):** 磁力计可以修正 Yaw 轴的漂移。
        
        脊柱侧弯（Scoliosis）不仅是侧弯，还伴随着**椎体旋转 (Vertebral Rotation)**。Schroth 疗法非常看重 "Derotation" (去旋转) 呼吸。如果只用 6轴，几分钟后 Avatar 的身体就会莫名其妙地“转圈”，演示效果会崩。
        
        **在无外部航向参考的情况下，6轴 IMU 的 yaw 只能靠陀螺积分，存在不可避免漂移；9轴通过磁力计提供航向参考可抑制漂移，因此对“稳定显示躯干旋转/朝向”的 avatar 演示更稳**
        
          
        
        但是！
        
        如果两个相邻的IMU经历相似的Yaw漂移（因为它们在相同的磁场环境中），那么计算相对角度时，漂移可能**部分抵消**。（**Estimating Relative Angles Using Two Inertial Measurement Units Without Magnetometers）**
        
        所以我们可以6和9轴都同时购买，比对效果
        
          
        
    
    1. 数据处理  
        **硬解（On-chip fusion）vs 软解（Raw data）****软解 (Raw Data -> MCU):** 传感器只吐出原始加速度/角速度，ESP32 负责跑卡尔曼/互补滤波。_风险：_ 有 4-6 个节点。ESP32 能不能同时跑 6 路高频滤波且不卡顿？
    
      
    
    **硬解 (On-chip DMP/Sensor Hub):** 传感器内部有 MCU，直接吐出 **四元数 (Quaternions)**。
    
    - _优势：_ ESP32 只负责搬运数据，CPU 占用极低，且厂家调教好的算法通常比自己写稳。
    
      
    
    而有个芯片可以同时满足6轴和9轴的测试：
    
    BNO086 并且能硬解。
    
    Tom给的LSM6HG256X