#include "app/segmentation_3d.h"


int main(int argc, char** argv)
{
	sgExecution::Segmentation3D segmentation("data\\camera_parameters\\default_camera_parameters.xml", "data\\scene1\\color.png", "data\\scene1\\depth.png");
	segmentation.Execute();
}