#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

cv::Affine3d pose;

void KeyboardCallback(const cv::viz::KeyboardEvent &event, void *viewer_void)
{
    cv::viz::Viz3d *viewer = static_cast<cv::viz::Viz3d *>(viewer_void);

    if (event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN)
    {
        if (event.symbol == "a")
        {
            pose.translation(cv::Vec3d(pose.translation()[0] - 0.1, pose.translation()[1], pose.translation()[2]));
        }
        else if (event.symbol == "d")
        {
            pose.translation(cv::Vec3d(pose.translation()[0] + 0.1, pose.translation()[1], pose.translation()[2]));
        }
        else if (event.symbol == "w")
        {
            pose.translation(cv::Vec3d(pose.translation()[0], pose.translation()[1] + 0.1, pose.translation()[2]));
        }
        else if (event.symbol == "s")
        {
            pose.translation(cv::Vec3d(pose.translation()[0], pose.translation()[1] - 0.1, pose.translation()[2]));
        }

        viewer->updateWidgetPose("Cube", pose);
    }
}

int main()
{
    cv::viz::Viz3d myWindow("Viz Demo");

    cv::viz::WCube cube_widget(cv::Point3d(0, 0, 0), cv::Point3d(1, 1, 1), true, cv::viz::Color::blue());
    myWindow.showWidget("Cube", cube_widget);

    pose = cv::Affine3d::Identity();
    myWindow.registerKeyboardCallback(KeyboardCallback, &myWindow);

    while (!myWindow.wasStopped())
    {
        myWindow.spinOnce(1, true);
    }

    return 0;
}
