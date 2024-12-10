cc_library(
    name = "base",
    srcs = [
        "camera_3d.cpp",
        "image.cpp",
        # pcl:visualization is currently not available in Bazel - disabled for now
        #"visualization_3d.cpp",
    ],
    hdrs = [
        "camera_3d.h",
        "image.h",
        #"visualization_3d.h",
    ],
    deps = [
        "//base/misc:misc",
        "//:opencv",
        "@eigen//:eigen",
        "@pcl//:io",
    ],
    visibility = ["//visibility:public"],
)
