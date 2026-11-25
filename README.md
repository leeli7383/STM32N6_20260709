# STM32N6 工业视觉缺陷检测与剔除系统

面向瓶盖表面瑕疵检测的嵌入式视觉竞赛项目，基于 STM32N6、Neural-ART NPU、ThreadX、摄像头、LCD 和 PCA9685 舵机控制模块。

本仓库保存从竞赛提交网站下载的 **重点代码快照**及设计报告，并补充代码阅读记录、验证方法和后续交接信息。原始 `test/` 文件保持不变。当前快照不是可独立重编译的完整 STM32CubeIDE 工程。

## 系统流程

```text
摄像头 / DCMIPP ──┬── 显示图像流 ──────────────────── LCD
                 └── 双缓冲 → NN Thread → INT8 输入转换
                                           ↓
                               Neural-ART / LL_ATON 推理
                                           ↓
                               阈值筛选 → 分类 NMS
                                  ├── 显示结果快照 → DP Thread → LCD 叠框
                                  └── 剔除事件队列 → Scheduler → Servo Thread
                                                              ↓
                                                   软件 I²C / PCA9685 / 舵机
```

另有 ISP Thread 处理摄像头 ISP 更新。显示结果通过互斥锁和信号量传递；舵机等待使用 `tx_thread_sleep`，让出 CPU，不在推理线程中同步等待机械动作。

## 当前代码中可以确认的实现

| 模块 | 实现与参数 | 代码入口 |
| --- | --- | --- |
| 运行平台 | 正点原子 N647 示例基础，STM32N6，ThreadX | `test/Appli/Core/Src/main.c` |
| 摄像头 | 配置选择 IMX335，DCMIPP PIPE1 显示、PIPE2 推理采集 | `test/Appli/STM32_MW_CAMERA/cmw_camera_conf.h`、`app_camera.c` |
| 图像缓存 | 两块外部 RAM 缓冲，32 字节对齐，free/ready 信号量交接 | `app_config.h`、`app.c`、`app_bqueue.c` |
| 模型输入 | 320×320×3；实际采集区 320×192，垂直居中填充；像素加 -128 转为有符号 INT8 | `app_config.h`、`app_model_prepare_input()` |
| 推理调用 | `LL_ATON_RT_Main`；输入清理/失效缓存、输出失效缓存 | `app_model_run()` |
| 后处理 | defect1/defect2；2100 个候选位置；置信度 0.35；同类别 NMS IoU 0.45；候选上限 40，输出上限 8 | `app_yolo_postprocess()` |
| 显示 | 800×480 LCD，图像、检测结果和控制面板绘制 | `app.c`、`app_lcd.c` |
| 剔除控制 | 事件队列深度 8、动作队列深度 2；调度线程与舵机线程分离 | `reject_servo.c` |
| 舵机驱动 | PB6=SCL、PB7=SDA；地址 0x40；60 Hz PWM；通道索引 1 | `pca9685.c`、`pca9685.h`、`reject_servo.c` |
| 启动 | FSBL 初始化外部存储及 HyperRAM，再进入应用启动流程 | `test/FSBL/Core/Src/main.c` |

舵机默认参数：检测后等待 1400 ms，重复事件窗口 250 ms，动作冷却 1200 ms，推出保持和归位保持各 1500 ms。它们是源码常量，**不代表已经通过输送带实测标定**。

## 设计报告中的算法结果

以下均为报告第 16–17 页记录的 **PC 端验证结果**，本次没有重新训练或复现实验：

| 项目 | 报告值 |
| --- | --- |
| 模型 | YOLO11n，训练输入 512×512 |
| 验证集 | 96 张图像、96 个目标实例，两类缺陷 |
| Precision / Recall | 0.940 / 0.971 |
| mAP@50 / mAP@50–95 | 0.9695 / 0.5933 |
| 单张推理耗时 | 约 18.2 ms，PC 端日志值 |

板端代码输入为 320×320，与报告训练输入不同；缺少模型导出配置、生成源码及训练日志，无法确认当前 HEX 与报告模型的版本对应关系。mAP 不是分类准确率，PC 耗时也不是板端性能。

## 文件结构

```text
test/
  Appli/                 应用代码、ThreadX 配置、摄像头/ISP 配置
  FSBL/                  启动与外部存储初始化代码
  Drivers/BSP/           部分 BUS 与 LCD 支持代码
  Binary/                原提交包中的四个 Intel HEX 文件
docs/
  design-report.pdf      原始设计报告
  SOURCE_AUDIT.md        代码与报告对照、缺失依赖和已知限制
  RESUME.md              根据当前证据整理的简历素材
  snapshot-manifest.json 原始 59 个文件的 SHA-256 清单
tools/
  verify_snapshot.py     不依赖第三方库的快照及 HEX 检查
PROGRESS.md              阶段进度与接续记录
THIRD_PARTY_NOTICES.md   原始第三方版权说明与使用边界
```

## 验证与恢复完整工程

使用 Python 3 运行：

```powershell
python tools/verify_snapshot.py
```

检查原始文件数量、字节数和 SHA-256，校验 HEX 记录长度、逐行校验和及 EOF，并列出有效数据地址范围。该检查不执行固件，也不证明烧录或运行正确。

完整重编译前需要从原开发环境补齐：

1. STM32CubeIDE `.project` / `.cproject`、链接脚本、启动汇编及相应工程配置。
2. STM32N6 CMSIS/HAL、ThreadX 内核、摄像头与 ISP 中间件实现、完整板级驱动。
3. ST Edge AI / LL_ATON 运行时，以及匹配模型的 `network.h`、网络生成源码和内存配置。
4. 训练/导出/量化配置、PT/ONNX 模型与训练验证日志，用于核对 512 与 320 输入尺寸差异。

补齐后，应先分别构建 FSBL、Appli，再按照原工程的外部 Flash 布局和下载算法进行烧录。仓库未提供经过验证的烧录地址或命令；不要仅依据 HEX 文件名推断烧录方案。

## 当前边界

- 报告记载原工程 FSBL / Appli 编译 0 错误 0 警告；本次仅验证快照文件和 HEX 格式，未重新编译固件。
- 当前剔除实现是 ThreadX 队列加两个工作线程，不是报告中列举的完整 IDLE/DELAY/PUSH/RETURN/FAULT 显式状态机。
- 连续物料调度、队列溢出、I²C 异常处理和板端性能仍需核验，详见 [代码对照记录](docs/SOURCE_AUDIT.md)。
- 原始第三方文件保留原版权头；本仓库没有为第三方代码另行授予开源许可，参见 [版权说明](THIRD_PARTY_NOTICES.md)。

进度与下一步见 [PROGRESS.md](PROGRESS.md)。
