# ROS2 学习

[![ROS2 version](https://img.shields.io/badge/ROS2-humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Ubuntu version](https://img.shields.io/static/v1?label=Ubuntu&message=22.04&color=e95420)](https://releases.ubuntu.com/22.04/)

## 说明

对应课程：[赵虚左 - ROS2理论与实践核心篇](https://space.bilibili.com/1101432368/channel/collectiondetail?sid=700208)

仅使用 Git 管理了 src 目录下的源代码，与本地配置相关的开发配置文件、install 目录下的编译结果不做同步。

## 目录

本地使用时，需以某个 **wsXX_XXX** 目录作为工作空间根目录。

- [ws00_helloworld](ws00_helloworld/) - ROS2 "Hello World" 初体验
  - [pkg01_helloworld_cpp](ws00_helloworld/src/pkg01_helloworld_cpp/) - C++ 版 "Hello world"
  - [pkg02_helloworld_py](ws00_helloworld/src/pkg02_helloworld_py/) - Python 版 "Hello world"
- [ws01_plumbing](ws01_plumbing/) - 通信机制
  - [base_interfaces_demo](ws01_plumbing/src/base_interfaces_demo/) - 存储各种通信接口定义文件的辅助功能包
  - [cpp01_topic](ws01_plumbing/src/cpp01_topic/) - 话题通信（C++ 实现）
  - [py01_topic](ws01_plumbing/src/py01_topic/) - 话题通信（Python 实现）
  - [cpp02_service](ws01_plumbing/src/cpp02_service/) - 服务通信（C++ 实现）
  - [py02_service](ws01_plumbing/src/py02_service/) - 服务通信（Python 实现）
  - [cpp03_action](ws01_plumbing/src/cpp03_action/) - 动作通信（C++ 实现）
  - [py03_action](ws01_plumbing/src/py03_action) - 动作通信（Python 实现）
