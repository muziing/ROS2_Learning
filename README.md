# ROS2 学习

[![ROS2 version](https://img.shields.io/badge/ROS2-humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Ubuntu version](https://img.shields.io/static/v1?label=Ubuntu&message=22.04&color=e95420)](https://releases.ubuntu.com/22.04/)
[![Python code style](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

## 说明

对应课程：[赵虚左 - ROS2理论与实践核心篇](https://space.bilibili.com/1101432368/channel/collectiondetail?sid=700208)

![课程封面](resources/images/course_cover.jpg)

仅使用 Git 管理了 src 目录下的源代码，与本地配置相关的开发配置文件、install 目录下的编译结果不做同步。

## 目录

每个 **wsXX_XXX** 为工作空间级目录，**cpp00_xxx** 或 **py00_xxx** 为功能包级目录。

- [ws00_helloworld](ws00_helloworld/) - ROS2 "Hello World" 初体验
  - [pkg01_helloworld_cpp](ws00_helloworld/src/pkg01_helloworld_cpp/) - C++ 版 "Hello world"
  - [pkg02_helloworld_py](ws00_helloworld/src/pkg02_helloworld_py/) - Python 版 "Hello world"
- [ws01_plumbing](ws01_plumbing/) - 通信机制
  - [base_interfaces_demo](ws01_plumbing/src/base_interfaces_demo/) - 存储各种通信接口定义文件的辅助功能包
  - [tutorails_plumbing](ws01_plumbing/src/tutorails_plumbing/) - 元功能包
  - [cpp01_topic](ws01_plumbing/src/cpp01_topic/) - 话题通信（C++ 实现）
  - [py01_topic](ws01_plumbing/src/py01_topic/) - 话题通信（Python 实现）
  - [cpp02_service](ws01_plumbing/src/cpp02_service/) - 服务通信（C++ 实现）
  - [py02_service](ws01_plumbing/src/py02_service/) - 服务通信（Python 实现）
  - [cpp03_action](ws01_plumbing/src/cpp03_action/) - 动作通信（C++ 实现）
  - [py03_action](ws01_plumbing/src/py03_action) - 动作通信（Python 实现）
  - [cpp04_param](ws01_plumbing/src/cpp04_param/) - 参数服务（C++ 实现）
  - [py04_param](ws01_plumbing/src/py04_param/) - 参数服务（Python 实现）
  - [cpp05_names](ws01_plumbing/src/cpp05_names/) - 节点命名空间、话题命名空间、话题重命名等（C++）
  - [cpp05_names_launch](ws01_plumbing/src/cpp05_names/launch/) - 通过launch文件修改节点、话题名称
  - [py05_names](ws01_plumbing/src/py05_names/) - 节点命名空间、话题命名空间、话题重命名等（Python）
- [ws02_time](ws02_time/) - 时间相关 API 的使用
- [ws03_exercise](ws03_exercise/) - 期中大作业
