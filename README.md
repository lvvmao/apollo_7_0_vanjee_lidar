# apollo_7_0_vanjee_lidar
apollo7.0平台下 vanjee lidar驱动
部署方法：
1、将lidar文件夹替换apollo平台下modules/drivers/lidar文件夹。
2、将vanjee_driver文件夹拷贝到/usr/local/include文件夹。


改动涉及到的文件包括：
ldiar/common/driver_factory：修改BUILD、lidar_driver_factory.cc和lidar_driver_fectory.hpp
lidar/proto：新增vanjee.proto vanjee_config.proto，修改BUILD文件
lidar/vanjee：新增该文件夹下所有内容
