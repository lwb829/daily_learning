# nuPlan

> 基于深度学习的车辆预测规划方法，结合深蓝学院课程进行学习

- [官网](https://www.nuscenes.org/nuplan#planning)
- [nuPlan-devkit环境源码地址](https://github.com/motional/nuplan-devkit)



## 一、环境配置

**本人的环境配置主要参考了弘毅师兄的仓库笔记**

全程参考nuplan-devkit/docs/installation.md，**但需要修改一些requirements中的库版本**，具体如下：

- 在`nuplan-devkit/requirements_torch.txt`中，更新torch(以及torchvision)的版本，否则后续的训练会报错

  ```
  --find-links https://download.pytorch.org/whl/torch_stable.html
  --find-links https://data.pyg.org/whl/torch-1.9.0+cu111.html
  future==0.18.1
  pytorch-lightning==1.3.8    # Used widely
  setuptools==59.5.0
  timm  # Used in model_raster
  
  # torch==1.9.0+cu111;      platform_system == "Linux"
  torch==1.12.0+cu116;      platform_system == "Linux"  # torch的版本不能是1.9.0，否则会报错“module 'torch' has no attribute 'frombuffer'”
  torch==1.9.0;            platform_system == "Darwin"
  torch_scatter==2.0.9; platform_system == "Linux"
  torchmetrics==0.7.2
  # torchvision==0.10.0+cu111
  torchvision==0.13.0+cu116
  ```

- 在`nuplan-devkit/requirements.txt`中，降低Pillow的版本，否则会在后续的训练中报错PIL

  ```
  Pillow==9.5.0    # 避免是10.0.0的版本，否则报错： module 'PIL.Image' has no attribute 'ANTIALIAS'
  ```

其余环境配置，按照如下方法即可

```
conda env create -f environment.yml
conda activate nuplan
cd naplan-devkit

#最好用以下命令配置环境，可以避免因依赖包数量大，从而出现终端卡顿
pip install -r requirements_torch.txt
pip install -r requirements.txt
```



- 官方文档中的关于“数据集路径”的说明，必须严格遵守。**即如果数据集不放在`~/nuplan/`下，就要在`~/.bashrc`中，添加环境变量，指明数据集路径**。

  通常这是必须的，因为Ubuntu的用户目录下，往往没有足够的空间来存放数据集。例如，本人将数据集放在电脑的机械盘，而ubuntu的用户目录是在另一块固态，那么就需要在`~/.bashrc`中添加参考路径

  - **注意：**本人一开始尝试将数据集放在其它路径中，但是经过测试发现，`nuplan_scenario_tutorial.ipynb`和`nuplan_framework.ipynb`两个notebook无法跑通，**原因是无法找到`NUPLAN_MAPS_ROOT`这个环境变量（尽管我已经按照要求将`NUPLAN_MAPS_ROOT`,`NUPLAN_DATA_ROOT`,`NUPLAN_DB_ROOT`三者的路径在`.bashrc`文件中正确表达），经过网上查找后试了几个方法均不太行**
  
  最后我将数据集放在home目录下，能够跑通那几个ipynb文件，即需要在bashrc文件中写入如下内容

  ```
  # 在.bashrc文件中写入
  export NUPLAN_DATA_ROOT="$HOME/nuplan/dataset"
  export NUPLAN_MAPS_ROOT="$HOME/nuplan/dataset/maps"
  export NUPLAN_DB_FILES="$HOME/nuplan/dataset/nuplan-v1.1/splits/mini"
  
  # 在终端中刷新环境变量
  source ~/.bashrc
  ```

- 数据集的结构，参考`nuplan-devkit/docs/dataset_setup.md`，即： **sensor_blobs下的传感器原始数据，暂时没有get到使用方法，如果只是按照`nuplan-devkit/tutorials`中notebook的训练方式，可以不用管sensor_blobs下的数据**

  ```
  ~/nuplan
  ├── exp
  │   └── ${USER}
  │       ├── cache
  │       │   └── <cached_tokens>
  │       └── exp
  │           └── my_nuplan_experiment
  └── dataset
      ├── maps
      │   ├── nuplan-maps-v1.0.json
      │   ├── sg-one-north
      │   │   └── 9.17.1964
      │   │       └── map.gpkg
      │   ├── us-ma-boston
      │   │   └── 9.12.1817
      │   │       └── map.gpkg
      │   ├── us-nv-las-vegas-strip
      │   │   └── 9.15.1915
      │   │       └── map.gpkg
      │   └── us-pa-pittsburgh-hazelwood
      │       └── 9.17.1937
      │           └── map.gpkg
      └── nuplan-v1.1
         ├── splits 
         │     ├── mini 
         │     │    ├── 2021.05.12.22.00.38_veh-35_01008_01518.db
         │     │    ├── 2021.06.09.17.23.18_veh-38_00773_01140.db
         │     │    ├── ...
         │     │    └── 2021.10.11.08.31.07_veh-50_01750_01948.db
         │     └── trainval
         │          ├── 2021.05.12.22.00.38_veh-35_01008_01518.db
         │          ├── 2021.06.09.17.23.18_veh-38_00773_01140.db
         │          ├── ...
         │          └── 2021.10.11.08.31.07_veh-50_01750_01948.db
         └── sensor_blobs   
               ├── 2021.05.12.22.00.38_veh-35_01008_01518                                           
               │    ├── CAM_F0
               │    │     ├── c082c104b7ac5a71.jpg
               │    │     ├── af380db4b4ca5d63.jpg
               │    │     ├── ...
               │    │     └── 2270fccfb44858b3.jpg
               │    ├── CAM_B0
               │    ├── CAM_L0
               │    ├── CAM_L1
               │    ├── CAM_L2
               │    ├── CAM_R0
               │    ├── CAM_R1
               │    ├── CAM_R2
               │    └──MergedPointCloud 
               │         ├── 03fafcf2c0865668.pcd  
               │         ├── 5aee37ce29665f1b.pcd  
               │         ├── ...                   
               │         └── 5fe65ef6a97f5caf.pcd  
               │
               ├── 2021.06.09.17.23.18_veh-38_00773_01140 
               ├── ...                                                                            
               └── 2021.10.11.08.31.07_veh-50_01750_01948
  ```

  - 只是为了测试，可以只用mini数据集，即`nuplan-v1.1/splits/mini`下的数据，并且不用配置sensor_blobs下的数据



## 二、ipynb文件运行

**Vscode可以直接打开`ipynb`文件, 并且安装对应插件后，选择好 conda 环境就可以直接运行，这里全部运行可以帮助判断是否跑通**

- 注意：最好用vscode而不是pycharm，这里我的pycharm下会告诉我这个`nest_asyncio`依赖包不符合条件，但是在

**除了可以在 notebook 中运行，还可以直接在命令行中运行，对应的命令都在`nuplan_framework.ipynb`有提供，举例如下：**

```
conda activate nuplan
cd ~/nuplan-devkit  # or wherever you have the nuplan-devkit
python nuplan/planning/script/run_training.py \
  experiment_name=raster_experiment \
  py_func=train \
  +training=training_raster_model \
  scenario_builder=nuplan_mini \
  scenario_filter.limit_total_scenarios=500 \
  lightning.trainer.params.max_epochs=10 \
  data_loader.params.batch_size=8 \
  data_loader.params.num_workers=8
```

**经过本人实际测试，以上 notebook 的运行都是没有问题的，并且终端运行训练也是没有问题的。不过暂时是在 mini 数据集上测试的，因为完整数据集太大了........**

- 在终端中的训练，结果会保存在 `~/nuplan/exp/exp` 下
- Simulation 的结果保存也同理

































