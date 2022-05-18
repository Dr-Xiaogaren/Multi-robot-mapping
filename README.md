# Multi-robot-mapping
gibson environment for multi-robot situation

### igibson环境简单至扩展多机器人
目前已经完成了仿真世界基本功能的实现，可以在场景中放置多个机器人，然后控制机器人移动，并返回RGBD类型的观测以及部分奖励。
根据任务不同，需要自己设计task functoin,reward dunction,terminatoin function。
可参照examples/robots/robot_control_example.py以及examples/environments/env_nonint_selector.py理解。
代码中可能对于一些iGibson底层的函数和python文件进行了修改后放到了项目文件中，名字没有修改，留意区分import进来的是库里的类还是项目里的类。

### 环境依赖
iGibson 2.2.0 

安装指引见 https://stanfordvl.github.io/iGibson/installation.html
安装完成后记得下载Rs场景（demo场景）以及相应数据集。

### 核心API
#### 场景部分
##### 1. iGibsonEnv
| Params | Example Value | Explanation |
| ----------| ------------- | ----------- |
|config_file | | |
|scene_id | | |
| mode  | 'headless'，'headless_tensor', 'gui_interative', 'gui_non_interative'  | headless无gui界面，headless_tensor返回观测值是torch tensor |
|action_timestep |同下 |同下 |
|physics_timestep |同下 |同下 |
|device_idx|0|仿真器所跑的显卡id|
|use_pb_gui|False|是否使用pybullet的可视化界面|


##### 2. Simulator
| Params | Example Value | Explanation |
| ----------| ------------- | ----------- |
| mode  | 'headless'，'headless_tensor', 'gui_interative', 'gui_non_interative'  | headless无gui界面，headless_tensor返回观测值是torch tensor |
| physics_timestep  | 1/40  | 类似于单片机时钟周期 ，仿真器每隔一个时间间隔更新一次 |
| render_timestep| 1/10 |类似于单片机机器周期，可以理解为机器人执行一个step的时间长度，**需要是physics_timestep的整数倍**，该参数会从config文件中的action_timestep得到|
|image_width, image_height, vertical_fov||摄像头获取的视野大小和摄像头垂直角度|
|device_idx|0|仿真器所跑的显卡id|
|use_pb_gui|False|是否使用pybullet的可视化界面|

#### task部分
在场景部分建立iGibsonEnv后，所有的对象都已经导入到了场景中，根据任务的不同，需要在task部分来设置、存储、改变场景中的对象或者与之相关的变量。
另外，reward function和temination function也是在导入到task中发挥作用的。（目前考虑感知部分的代码也会放到task中）
##### 1. MappingTask
 `reset_agent`, `reset_variables`, `get_termination`, `get_reward`, `get_task_obs`, `step`需要根据任务的不同进行相应修改。
 留意get_task_obs取出的是task-specific的观测，一些通用的观测（比如RGBD,Scan,Seg）在config文件中可以指定，不用在这单独取。

#### 运行实例
库根目录下：
```bash
python -m envs.igibson_env --config /home/vsis/Documents/Multi-robot-mapping/config/multi_robot_mapping.yaml --mode gui_interactive
```
成功后可以看到两个界面，一个旁观者界面，一个机器人视角,机器人随机移动。


