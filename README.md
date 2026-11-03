# Sustech-s-Ultimate-weapon
Used to write some ARTINX`s visual code
//起名为南科大最终兵器，创建时间为2026年十月25日
//用于储存我最初在视觉组学习和工作时写的代码。
//希望明年能和大家一起打进春茧。

---

## 仿真联调：在 Godot（rm_aim_sim）里跑 to-see-armor

> **记录时间：2026-11-03**
> 环境：WSL2 Ubuntu 22.04 / OpenCV 4.5.4 / Godot 4.7.2（`~/rm_aim_sim/.tools/godot`）/ WSLg 图形会话
> 本机没有 `/dev/dri`，Godot 走软件渲染，5–15 fps 属正常现象。

### 1. 启动顺序（必须先仿真，后视觉）

```bash
# 终端 1：启动仿真
cd ~/rm_aim_sim
./run.sh -- --remote          # 看到 Aim Lab ready | remote=true 即就绪

# 终端 2：跑检测
cd ~/ARTINX-visual-work
./build/to-see-armor --sim --team=blue --scale=2 --frames=60
```

> 仿真相机与视觉端通过 `/dev/shm/rm_aim_sim_<通道>/` 交换数据，**不监听 TCP/UDP**。
> 同一通道只允许一个仿真 + 一个视觉客户端（`client.lock` 非阻塞独占），不能同时开两个视觉程序。

### 2. 参数详解

| 参数 | 默认 | 具体用处 |
|---|---|---|
| 无参数 | — | 打开 0 号摄像头（`VideoCapture(0)`），红方，双窗口，ESC 退出。**与加入仿真支持之前完全一致** |
| `--sim` | 关 | 改为从共享内存取图，不打开摄像头 |
| `--sim=team_a` / `--channel=team_a` | `default` | 指定通道名，必须与 `rm_aim_sim/configs/sim.conf` 里 `[ipc].channel` 一致 |
| `--team=red` / `--team=blue` | `red` | 识别哪一方。**必须与仿真场景阵营一致**：仿真默认是蓝方（`appearance.team=blue`），若用 `red` 则红蓝相减后全黑，0 命中——这是算法的正确行为，不是 bug |
| `--scale=N` | 1 | 取到帧后先放大 N 倍（1–4）再走原算法，输出坐标自动除回原图。**1440×1080 下灯条只有 2–4 px 宽**，会被 `area>50` 全部滤掉；`--scale=2` 后灯条约 5×23 px，可稳定通过阈值。默认 1 = 原始行为 |
| `--frames=N` | 0（不限） | 收满 N 帧后退出，方便脚本化跑测试 |
| `--duration=S` | 0（不限） | 跑 S 秒后退出 |
| `--no-display` | 关 | 不建窗口，只计算（无图形会话或卡顿时用） |
| `--dump=DIR` | 空 | 保存最多 20 组 `raw_ / mask_ / draw_` PNG，用于离线核图 |
| `--help` | — | 打印完整用法 |

### 3. 实测结果（2026-11-03）

| 命令 | 结果 |
|---|---|
| `--sim --team=blue --scale=2 --frames=60` | **60/60 帧命中**，每帧 2 根灯条、1 块装甲板，中心稳定在 `(719.75, 691.75)`，平均 10–11 fps |
| 精度对比（仿真真值中心 `(720.00, 692.55)`） | 中心误差 **0.84 px** |
| `--team=red --scale=2` | 0 命中（阵营不符，符合预期） |
| `--team=blue --scale=1` | 0 命中（灯条太小被阈值滤除，符合预期） |
| 无参数（无摄像头） | 与改动前一致：打印 `不是哥们摄像头都不开啊`，返回 -1 |

### 4. 2026-11-03 修复记录：灯条四角自交

`findLightBars` 把 `minAreaRect` 的角度从 `(0,90]` 归一到 `(-45,45]` 时，**减 90° 必须同时交换 `size.width` / `size.height`**：

```cpp
float a = rect.angle;
if(a > 45.0f){
    a -= 90.0f;
    swap(rect.size.width, rect.size.height);   // 少了这一句，竖条会被描述成横条
}
rect.angle = a;
```

**不交换的后果**：灯条矩形从“竖”变“横”，`barEnds` 求出的上下端点跑到左右两端，装甲板四角连成自交的蝴蝶结；中心点和外接矩形仍正确，但四角无法用于 PnP 姿态解算。

**修复后**：四角顺序为标准 左上 → 右上 → 右下 → 左下，实测四角横向误差 ≤0.5 px（上沿差 2.15 px 是灯条本身高过板框的几何差异）。

