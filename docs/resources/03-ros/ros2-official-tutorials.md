# ROS 2 Lyrical 官方教程索引

> - 状态：Source Checked
> - 语言：English
> - 费用：Free
> - 适用版本：ROS 2 Lyrical Luth
> - 最后检查：2026-08-31
> - 官方文档源版本：[`ros2/ros2_documentation@0d8366f`](https://github.com/ros2/ros2_documentation/tree/0d8366f4f89878e277a1e62daf6d15fd7eb6a4dc)

本页依据 ROS 2 官方文档仓库 `lyrical` 分支整理，覆盖官方入门学习路径、命令行基础、客户端库、tf2、URDF、Launch、rosbag 和若干进阶教程。它不是官方文档镜像；标题和链接均指向官方页面。

ROS 2 文档结构会调整。若链接与页面内容发生变化，请先查看 [Lyrical 文档首页](https://docs.ros.org/en/lyrical/) 和 [官方文档源仓库](https://github.com/ros2/ros2_documentation/tree/lyrical)。

## 1. Start here

- [ROS 2 Lyrical Documentation](https://docs.ros.org/en/lyrical/)
- [First steps with ROS — learning path](https://docs.ros.org/en/lyrical/First-Steps.html)
- [Installation options](https://docs.ros.org/en/lyrical/Get-Started/Installation.html)
- [Ubuntu (deb packages)](https://docs.ros.org/en/lyrical/Get-Started/Installation/Ubuntu-Install-Debs.html)
- [Windows binary installation](https://docs.ros.org/en/lyrical/Get-Started/Installation/Windows-Install-Binary.html)
- [Installing on Raspberry Pi](https://docs.ros.org/en/lyrical/Get-Started/Installation/Installing-on-Raspberry-Pi.html)
- [Installation troubleshooting](https://docs.ros.org/en/lyrical/Get-Started/Installation/Installation-Troubleshooting.html)
- [Setting up the ROS 2 environment](https://docs.ros.org/en/lyrical/Get-Started/Configuring-ROS2-Environment.html)
- [Using turtlesim, ros2, and rqt](https://docs.ros.org/en/lyrical/Get-Started/Introducing-Turtlesim/Introducing-Turtlesim.html)

## 2. Core concepts and command-line tutorials

- [About ROS](https://docs.ros.org/en/lyrical/About-ROS.html)
- [About nodes](https://docs.ros.org/en/lyrical/ROS-Framework/About-Nodes.html)
- [About parameters](https://docs.ros.org/en/lyrical/ROS-Framework/About-Parameters.html)
- [Interfaces, topics, services, and actions](https://docs.ros.org/en/lyrical/ROS-Framework/Interfaces-Topics-Services-Actions.html)
- [Learning about nodes](https://docs.ros.org/en/lyrical/ROS-Framework/nodes/Working-with-nodes/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [Learning about topics](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/topics/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [Learning about services](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/services/Working-with-services/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [Learning about parameters](https://docs.ros.org/en/lyrical/ROS-Framework/parameters/Working-with-parameters/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [Learning about actions](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/actions/Working-with-actions/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- [Using rqt_console to view logs](https://docs.ros.org/en/lyrical/ROS-Framework/nodes/Working-with-nodes/Using-Rqt-Console/Using-Rqt-Console.html)
- [Launching multiple nodes](https://docs.ros.org/en/lyrical/ROS-Framework/nodes/Working-with-nodes/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html)
- [Recording and playing back data](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/Working-with-interfaces/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

## 3. Workspace, packages, and client libraries

- [About client libraries](https://docs.ros.org/en/lyrical/ROS-Framework/About-Client-Libraries.html)
- [Working with client libraries](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries.html)
- [Using colcon to build packages](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Colcon-Tutorial.html)
- [Creating a workspace](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- [Creating a package](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [Writing a publisher and subscriber — C++](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [Writing a publisher and subscriber — Python](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Writing a service and client — C++](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)
- [Writing a service and client — Python](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Creating custom msg and srv files](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Custom-ROS2-Interfaces.html)
- [Implementing custom interfaces in one package](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Single-Package-Define-And-Use-Interface.html)
- [Using parameters in a class — C++](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
- [Using parameters in a class — Python](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [Managing dependencies with rosdep](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Rosdep.html)
- [Using ros2doctor to identify issues](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Getting-Started-With-Ros2doctor.html)
- [Creating and using plugins — C++](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Pluginlib.html)

## 4. Topics, services, actions, and parameters

### Topics and QoS

- [About Quality of Service settings](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/topics/About-Quality-of-Service-Settings.html)
- [Quality of Service tutorial](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/topics/Working-with-topics/Quality-of-Service.html)
- [Content-filtered subscriptions](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/topics/Working-with-topics/Content-Filtering-Subscription.html)
- [Wait for acknowledgment](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/topics/Working-with-topics/Wait-for-Acknowledgment.html)
- [Topic keys tutorial](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/topics/Working-with-topics/Topic-Keys/Topic-Keys-Tutorial.html)
- [Filtered topic keys tutorial](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/topics/Working-with-topics/Topic-Keys/Filtered-Topic-Keys-Tutorial.html)

### Services

- [Synchronous versus asynchronous service clients](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/services/Working-with-services/Sync-Vs-Async.html)

### Actions

- [Creating an action definition](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/actions/Working-with-actions/Creating-an-Action.html)
- [Writing an action server and client — C++](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/actions/Working-with-actions/Writing-an-Action-Server-Client/Cpp.html)
- [Writing an action server and client — Python](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/actions/Working-with-actions/Writing-an-Action-Server-Client/Py.html)
- [Configuring action introspection](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/actions/Working-with-actions/Action-Introspection.html)

### Parameters

- [Using the ros2 param command-line tool](https://docs.ros.org/en/lyrical/ROS-Framework/parameters/Working-with-parameters/Using-ros2-param.html)
- [Monitoring parameter changes — C++](https://docs.ros.org/en/lyrical/ROS-Framework/parameters/Working-with-parameters/Monitoring-For-Parameter-Changes-CPP.html)
- [Monitoring parameter changes — Python](https://docs.ros.org/en/lyrical/ROS-Framework/parameters/Working-with-parameters/Monitoring-For-Parameter-Changes-Python.html)

## 5. tf2

- [Introducing tf2](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/Working-with-interfaces/Introduction-To-Tf2/Introduction-To-Tf2.html)
- [tf2 tutorial index](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Tf2-Main.html)
- [Writing a static broadcaster — C++](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html)
- [Writing a static broadcaster — Python](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html)
- [Writing a broadcaster — C++](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html)
- [Writing a broadcaster — Python](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Writing-A-Tf2-Broadcaster-Py.html)
- [Writing a listener — C++](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Writing-A-Tf2-Listener-Cpp.html)
- [Writing a listener — Python](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Writing-A-Tf2-Listener-Py.html)
- [Adding a frame — C++](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Adding-A-Frame-Cpp.html)
- [Adding a frame — Python](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Adding-A-Frame-Py.html)
- [Using time — C++](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Learning-About-Tf2-And-Time-Cpp.html)
- [Traveling in time — C++](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Time-Travel-With-Tf2-Cpp.html)
- [Using stamped datatypes with tf2_ros::MessageFilter](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter.html)
- [Quaternion fundamentals](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/Working-with-interfaces/Quaternion-Fundamentals.html)

## 6. URDF and robot models

- [URDF tutorial index](https://docs.ros.org/en/lyrical/Capabilities/Simulation/URDF/URDF-Main.html)
- [Building a visual robot model from scratch](https://docs.ros.org/en/lyrical/Capabilities/Simulation/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html)
- [Building a movable robot model](https://docs.ros.org/en/lyrical/Capabilities/Simulation/URDF/Building-a-Movable-Robot-Model-with-URDF.html)
- [Adding physical and collision properties](https://docs.ros.org/en/lyrical/Capabilities/Simulation/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html)
- [Using URDF with robot_state_publisher — C++](https://docs.ros.org/en/lyrical/Capabilities/Simulation/URDF/Using-URDF-with-Robot-State-Publisher-cpp.html)
- [Using URDF with robot_state_publisher — Python](https://docs.ros.org/en/lyrical/Capabilities/Simulation/URDF/Using-URDF-with-Robot-State-Publisher-py.html)
- [Generating a URDF file](https://docs.ros.org/en/lyrical/Capabilities/Simulation/URDF/Exporting-an-URDF-File.html)

## 7. Launch

- [Launch tutorial index](https://docs.ros.org/en/lyrical/Developer-Tools/Launch/Launch-Main.html)
- [Creating a launch file](https://docs.ros.org/en/lyrical/Developer-Tools/Launch/Creating-Launch-Files.html)
- [Integrating launch files into ROS 2 packages](https://docs.ros.org/en/lyrical/Developer-Tools/Launch/Launch-system.html)
- [Using substitutions](https://docs.ros.org/en/lyrical/Developer-Tools/Launch/Using-Substitutions.html)
- [Using event handlers](https://docs.ros.org/en/lyrical/Developer-Tools/Launch/Using-Event-Handlers.html)
- [Managing large projects](https://docs.ros.org/en/lyrical/Developer-Tools/Launch/Using-ROS2-Launch-For-Large-Projects.html)
- [Using XML, YAML, and Python launch files](https://docs.ros.org/en/lyrical/Developer-Tools/Launch/Launch-file-different-formats.html)
- [Launching composable nodes](https://docs.ros.org/en/lyrical/Developer-Tools/Launch/Launching-composable-nodes.html)

## 8. rosbag2 and data workflows

- [Recording a bag from a node — C++](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Recording-A-Bag-From-Your-Own-Node-CPP.html)
- [Recording a bag from a node — Python](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Recording-A-Bag-From-Your-Own-Node-Py.html)
- [Reading from a bag file — C++](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Reading-From-A-Bag-File-CPP.html)
- [Reading from a bag file — Python](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Reading-From-A-Bag-File-Python.html)

## 9. Nodes, composition, and execution

- [Writing a composable node — C++](https://docs.ros.org/en/lyrical/ROS-Framework/nodes/Working-with-nodes/Writing-a-Composable-Node.html)
- [Composing multiple nodes in one process](https://docs.ros.org/en/lyrical/ROS-Framework/nodes/Working-with-nodes/Composition.html)
- [Efficient intra-process communication](https://docs.ros.org/en/lyrical/ROS-Framework/nodes/Working-with-nodes/intra-process/Intra-Process-Communication.html)
- [Using callback groups](https://docs.ros.org/en/lyrical/ROS-Framework/nodes/Working-with-nodes/Using-callback-groups.html)
- [Managing node lifecycles](https://docs.ros.org/en/lyrical/ROS-Framework/nodes/Working-with-nodes/Managed-Nodes.html)
- [Using the Node Interfaces Template Class — C++](https://docs.ros.org/en/lyrical/ROS-Framework/nodes/Working-with-nodes/Using-Node-Interfaces-Template-Class.html)
- [Writing an async node with asyncio — Python](https://docs.ros.org/en/lyrical/ROS-Framework/nodes/Working-with-nodes/Writing-An-Async-Node-With-Asyncio-Python.html)

## 10. Advanced client-library topics

- [Implementing a custom memory allocator](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Allocator-Template-Tutorial.html)
- [Configuring zero-copy loaned messages](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Configure-ZeroCopy-loaned-messages.html)
- [Creating an rmw implementation](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Creating-An-RMW-Implementation.html)

## Maintenance notes

- 本页只收录 ROS 2 官方文档；YouTube、Bilibili 和第三方课程应放在其他资源页。
- 官方源文件存在但标记为 `Coming Soon` 的集合页，不作为内容已完成的证明。
- 自动化访问官方文档站可能遇到 Anubis 防护，因此本轮使用官方 GitHub 源仓库核对路径。
- 发现失效链接时，应同时检查 `lyrical` 分支是否移动或重命名对应源文件。

[返回 ROS 资源目录](README.md) · [返回机器人资源总目录](../README.md)
