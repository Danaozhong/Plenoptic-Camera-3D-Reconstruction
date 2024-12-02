cc_library(
    name = "app",
    srcs = [
        "contour_fitting_3d.cpp",
        "find_contours_2d.cpp",
        "image_preprocessing_2d.cpp",
        "segmentation_3d.cpp",
        "ransac.cpp",
    ],
    hdrs = [
        "contour_fitting_3d.h",
        "find_contours_2d.h",
        "image_preprocessing_2d.h",
        "segmentation_3d.h",
        "ransac.h",
    ],
    deps = [
        "//main/2d:main_2d",
        "//main/3d:main_3d",
        "//base:base",
        "//base/misc:misc",
        "@eigen//:eigen"
    ],
    visibility = ["//visibility:public"],
)


cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [
        "//app:app"
    ],
)
