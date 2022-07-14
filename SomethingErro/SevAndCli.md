## Service and Client

这个文件记录了在编写ROS Service & Client 模式下常见的集中错误以及解决方案。

为了方便后面的描述，这边作出以下几个规定：

```cpp
ros::NodeHandle hd;  // 句柄变量名

#include "demo/count.h" // 当前ROS package名为demo，服务的srv名为count
// 这里的服务请求名可能会出现变动，但在请求前都会使用命名空间如 demo::count, demo::calc
```



-----------

* **error: no matching function for call to ‘ros::AdvertiseServiceOptions::initBySpecType\<demo::count&\>(const string&, const boost::function\<bool(demo::count&)\>&)’**

这个问题往往出现在Server.cpp中的 hd.advertiseService函数，这个函数的定义方式有两种：

1.  hd.advertiseService\<demo::countRequest, demo::countResponse\>("count_server", DealCountRequirest);
2.  hd.advertiseService("count_server", DealCountRequirest);

在初学阶段很容易出现下面这种定义方式：

*  hd.advertiseService\<demo::count\>("count_server", DealCountRequirest);

如果出现了上述报错将hd.advertiseService修改即可。

牢记一句话：ROS C++是要求严格语法的，定义的地方要么让他自动推导，要么自己实现完全定义。

---------

