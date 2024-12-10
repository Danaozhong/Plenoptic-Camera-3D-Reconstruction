cc_library(
    name = "main_3d",
    srcs = [
        "contour_3d.cpp",
        "fit_line_3d.cpp",
        "transformation_3d.cpp",
        "fit_line_3d/fit_line_3d_begin_end_point_averaging.cpp",
        "fit_line_3d/fit_line_3d_least_squares.cpp",
        "fit_line_3d/fit_line_3d_minimum_variance.cpp",
        
    ],
    hdrs = [
        "contour_3d.h",
        "fit_line_3d.h",
        "transformation_3d.h",
        "fit_line_3d/fit_line_3d_begin_end_point_averaging.h",
        "fit_line_3d/fit_line_3d_least_squares.h",
        "fit_line_3d/fit_line_3d_minimum_variance.h",
    ],
    deps = [
        "//base/geometrics:geometrics",
        "@pcl//:common",
        "@pcl//:segmentation",
        "//:opencv"
    ],
    visibility = ["//visibility:public"],
)
