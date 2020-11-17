# mr_nav

所有的导航配置和启动文件

## 目录说明

```
├── cfg                   ------ 配置文件夹
│   ├── dwa               ------ DWA算法的配置
│   ├── eband             ------ eband算法的配置
│   └── teb               ------ teb算法的配置
├── launch                ------ 启动文件夹
│   ├── amcl.launch       ------ amcl配置文件
│   ├── nav_dwa.launch    ------ 基于dwa导航启动
│   ├── nav_eband.launch  ------ 基于eband导航启动
│   └── nav_teb.launch    ------ 基于teb导航启动
└──
```

**注意此包中的teb_local_planner中的local_costmap层和官方的并不太一致，直接使用了DWA的参数，目前为加快项目进度，打算后期再做修改**