#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;
std::vector<cv::Point2f> intermediate_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left btn of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

std::vector<cv::Point2f> compute_intermediate_points(const std::vector<cv::Point2f> &control_points, float t) 
{
    std::vector<cv::Point2f> int_points;
    for (auto it = control_points.begin(); it != control_points.end() - 1; ++it) {
        cv::Point2f point = (1 - t) * (*it) + t * (*(it + 1));
        int_points.push_back(point);
    }
    return int_points;
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    if (control_points.size() == 1) {
        return control_points[0];
    } 
    else 
    {
        intermediate_points = control_points;
        while (intermediate_points.size() > 1) {
            intermediate_points = compute_intermediate_points(intermediate_points, t);
        }
        return intermediate_points[0];
    }
    intermediate_points.clear();
}
bool isPointInsideWindow(const cv::Point2f &point, const cv::Mat &window) {
    cv::Rect windowRect(0, 0, window.cols, window.rows);
    return windowRect.contains(point);
}
void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window, float t = 0.0, float step = 0.001) 
{
    // Base case: If t exceeds 1.0, stop recursion
    if (t > 1.0) {
        return;
    }
    
    // Call recursive_bezier to get the point at current t
    cv::Point2f point = recursive_bezier(control_points, t);
    
    // Check if point is within window boundaries
    if (isPointInsideWindow(point, window)) {
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255; // Set green channel to 255
    }
    
    // Recursively call bezier function with incremented t
    bezier(control_points, window, t + step, step);
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
