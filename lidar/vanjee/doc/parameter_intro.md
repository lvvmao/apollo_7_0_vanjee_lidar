# Parameter Introduction



This document will give you a introduction to each parameter in the configuration file.  



## 1 driver parameter

For convenient, we take vanjee_720_16.pb.txt as an example.

```yaml
model: "vanjee_720_16"
connect_type: 1
host_msop_port: 3001
lidar_msop_port: 3333
host_address: "192.168.2.88"
lidar_address: "192.168.2.86"
publish_mode: 2
start_angle: 0
end_angle: 360
min_distance: 0.3
max_distance: 120
point_cloud_masking:""
use_lidar_clock: false
ts_first_point: true
use_offset_timestamp: true
dense_points: false
wait_for_difop: true
config_from_file: false
angle_path_hor: ""
angle_path_ver: "/usr/local/include/vanjee_driver/param/Vanjee_720_16_VA.csv"
pointcloud_channel: "/apollo/sensor/vanjee_720_16/PointCloud2"
scan_channel: "/apollo/sensor/vanjee_720_16/Scan"
frame_id: "vanjee_lidar_720_16"
```

**model**

This is the lidar model, since this configuration file is for vanjee_720_16, we set the model="vanjee_720_16".The parameter are as followed below:vanjee_720_16,vanjee_720_32,vanjee_722.

**connect_type**

This is the net connection model,which is tcp or udp.Set the connect_type="1" if the net connection mode is udp,otherwise set it as "2".

**host_msop_port**

This is the port number of host. The default value is 3001.Lidar would send the raw packet to this port.

**lidar_msop_port**

This is the port number of lidar. The default value is 3333.Lidar would send the raw packet from this port.

**host_address**

This is the host ip address. The default value is "192.168.2.88".Lidar would send the raw packet to this ip address.

**lidar_address**

This is the lidar ip address. The default value is "192.168.2.86".Lidar would send the raw packet from this ip address.

**publish_mode**

This is the publish mode of lidar point cloud. "0": publish the 1st echo ,"1":publish the 2nd echo ,"2":publish both the 1st and 2nd echos.

**start_angle**

The start angle of point cloud,unit:degree.

**end_angle**

The end angle of point cloud,unit:degree.The point cloud published would only include the azimuth between start_angle and end_angle.

**min_distance**

The minimum distance of point cloud,unit:m.

**max_distance**

The maximum distance of point cloud,unit:m.The point cloud published would only include the distance between min_distance and max_distance.

**point_cloud_masking**
masking range,
  line id range: C1-C2，angle range: A1-A2，distance range: L1-L2；(Range is set in ascending order)
  Group 1：1-3,0-0.5/100-105,0-1.5/10-20;(masking line id: 1 2 3，masking angle: 0-0.5°+100-105°，masking distance: 0-1.5m+10-20m);
  Group 2：5,0/90/180/270,10;(masking line id: 5，masking angle: 0+90+180+270°，masking distance: 10m);
  Group 3：1-3,0-0.5/100-105,0-1.5/10-20;5,0/90/180/270,10;(masking range = Group 1 + Group 2);

masking Rules,
  Separate group elements with commas (,), separate line id, angles, and distances with slashes (/) when they are not continuous, and separate two groups with semicolons (;), all using English characters as separators;

Note: The angle/distance ranges of Shielding Group 1 and Shielding Group 2 are independent of each other, as shown below. Shielding Group 4 and     Shielding Group 5 have different ranges, while Shielding Group 4 and Shielding Group 6 have equal ranges;

  Group 4：5,10-20/50-60,1.5-2.5/11-13;
  Group 5：5,10-20,1.5-2.5;5,50-60,11-13;
  Group 6：5,10-20,1.5-2.5;5,10-20,11-13;5,50-60,1.5-2.5;5,50-60,11-13;

**use_lidar_clock**

  - true: The timestamp of lidar messages is the lidar clock.

  - false: The timestamp of lidar messages is the PC system clock.

**ts_first_point**

  - true: The header timestamp of pointcloud is the first point timestamp.

  - false: The header timestamp of pointcloud is the last point timestamp.

**use_offset_timestamp**

  - true: The timestamp of pointcloud is timestamp offset to the header timestamp of pointcloud.

  - false: The timestamp of pointcloud is utc timestamp.

**dense_point**

The flag whether mark the invald point cloud as 'nan'. If you would not mark the invalid point as 'nan',set the dense_point=true,otherwise set it as false.

**wait_for_difop**

The flag whether need to request the angle information from the lidar firstly. Usually set wait_for_difop=true in ONLINE_LIDAR mode,otherwise set it as false.

**config_from_file**

The flag whether read the angle information from the configuration file .Usually set config_from_file=true in RAW_PACKET mode,otherwise set it as false.

**angle_path_hor**

The path of horizontal angle configuration file.

**angle_path_ver**

The path of vertical angle configuration file.

**pointcloud_channel**

The channel name of point cloud.

**scan_channel**

The channel name of scan message.

**frame_id**

This is the frame_id value in the lidar messages.