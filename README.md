# 3D Reconstruction Using a Plenoptic Field Camera


![build](https://github.com/Danaozhong/Plenoptic-Camera-3D-Reconstruction/actions/workflows/build.yml/badge.svg)

## Introduction
[Plenoptic cameras](https://en.wikipedia.org/wiki/Light_field_camera) aka light field camera are cameras that use an array of microlenses. Due to the slight positional difference of the lenses, it is possible to reconstruct depth (3D) information from the camera image.


I created this program in 2014 as a research project while doing my masters at the Karlsruhe University of Applied Sciences. We were using a Raytrix light field camera.

The outcome of this research has been published in a paper ["Edge Segmentation in Images of a Focused Plenoptic Camera"](https://www.niclas-zeller.de/media/publications/zeller2014isetc.pdf).

## The Algorithm

This program tries to reconstruct 3D scenes using the following technique:

1) The firmware of the camera pre-computes a (very noisy) 3D point cloud.
2) Using the 2D image, a 2D edge detector (canny) is applied to find contours of surfaces.
3) for each surface, a [RANSAC](https://en.wikipedia.org/wiki/Random_sample_consensus) algorithm is executed within the found surfaces from step (2).
4) If a consent is found, the 2D contour is re-projected to 3D space using a least squares fitting algorithm.
5) The found 3D surfaces are rendered in 3D space.

This algorithm is designed for environments with many planar surfaces, such as offices, staircases, or roads.


## How to Build

This project uses Bazel to build. Type `bazel build ...` to build. All dependecies should be automatically installed.
Due to the dependency to OpenCV, builds currently only work under Linux. Once OpenCV is available on the bazel registry, this code should be platform-independent.

## Disclaimer

This code is very old. It is published mainly for historic reasons. I believe by now, there are much better approaches to solve the problem of 3D reconstruction, such as [SLAM](https://en.wikipedia.org/wiki/Robotic_mapping), commonly used in mapping.
