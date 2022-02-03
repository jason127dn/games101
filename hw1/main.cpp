#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
Eigen::Matrix4f get_rotation(Eigen::Vector3f axis,float angle)
{

    angle = angle*MY_PI/180.;
    
    Eigen::Matrix4f rotation;
    float cost = std::cos(angle);
    float sint = std::sin(angle);
    float n = axis.norm();
    float x = axis[0]/n;
    float y = axis[1]/n;
    float z = axis[2]/n;
    rotation << cost+(1-cost)*x*x,(1-cost)*x*y-sint*z,(1-cost)*x*z+sint*y,0.,
                (1-cost)*y*x+sint*z,cost+(1-cost)*y*y,(1-cost)*y*z-sint*x,0.,
                (1-cost)*z*x-sint*y, (1-cost)*z*y+sint*x, cost+(1-cost)*z*z,0.,
                0.,0.,0.,1.;
    return rotation;
}
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model;
    rotation_angle=rotation_angle*MY_PI/180.;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    model << std::cos(rotation_angle), -std::sin(rotation_angle), 0., 0.,
             std::sin(rotation_angle), std::cos(rotation_angle), 0., 0.,
             0., 0., 1., 0.,
             0., 0., 0., 1.;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f prespective;
    Eigen::Matrix4f orthogonal;
    prespective << zNear, 0., 0., 0.,
                    0., zNear, 0., 0.,
                    0., 0., zNear+zFar, - zNear*zFar,
                    0., 0., 1., 0.;
    float l,r,t,b;
    t = std::tan(eye_fov/2.)*std::abs(zNear);
    b = -t;
    r = t * aspect_ratio;
    l = -r;
    orthogonal << 1./r, 0., 0., 0.,
                    0., 1./t, 0., 0.,
                    0., 0., -2./(zFar-zNear), -(zFar+zNear)/2.,
                    0., 0., 0., 1.;
    projection = orthogonal*prespective;
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    Eigen::Vector3f raxis = {0., 0., 1.1};
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_rotation(raxis,angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_rotation(raxis,angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        else if (key == 'i') {
            raxis[0]+=1;
        }
        
        else if (key == 'k') {
            raxis[0]+=-1;
        }
        else if (key == 'j') {
            raxis[1]+=1;
        }
        else if (key == 'l') {
            raxis[1]+=-1;
        }
        else if (key == 'u') {
            raxis[2]+=1;
        }
        else if (key == 'o') {
            raxis[2]+=-1;
        }
    }

    return 0;
}
