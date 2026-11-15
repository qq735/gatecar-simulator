# 控制核心快照

`CarCore.c`、`CarProfiles.c`、`CarCore.h`是本项目整理的可移植控制核心快照。下载包只读取此目录，不会自动覆盖或读取下载者的其他工程。

原工程改造基于 https://github.com/Lscyzq/HFK_STC32G144K ，提交 `6825bf1126439091bc6a39d0c59c1e01aaba79e3`。原作者Lscyzq，原工程版权和许可声明保持。该上游说明不表示本仿真来自上游作者或已由其验证。

主机wrapper、场模型、闭环赛道与网页在本仓库中独立实现。界面“重新编译参考C核心”只编译本目录；导入自己的控制算法请使用`sdk/gatecar_api.h`和`examples`，详见`docs/CODE_IMPORT.md`。

请在README对应边界内使用：没有编译STC外设代码，未运行完整Image/Vision元素识别链。
