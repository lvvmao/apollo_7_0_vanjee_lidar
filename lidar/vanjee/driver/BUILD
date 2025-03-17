load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

VANJEE_COPTS = ['-DMODULE_NAME=\\"vanjee\\"']

cc_library(
    name = "vanjee_driver_component_lib",
    srcs = [
        "vanjee_driver_component.cc",
        # "driver.cc",
    ],
    hdrs = [
        "vanjee_driver_component.h",
        # "driver.h",
    ],
    copts = VANJEE_COPTS,
    deps = [
        ":driver",
        "//cyber",       
    ],
)

cc_library(
    name = "driver",
    srcs = [
        "driver.cc",
    ],
    hdrs = [
        "driver.h",
    ],
    copts = VANJEE_COPTS,
    deps = [
        "//cyber",       
        "//modules/common/util",
        "//modules/drivers/lidar/common/driver_factory:driver_base",
        "//modules/drivers/lidar/proto:config_proto",
        "//modules/drivers/lidar/proto:vanjee_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
    ],
)

cpplint()
