#include <rclcpp/rclcpp.hpp>
#include <liblas/liblas.hpp>
#include <fstream>

class PointCloudReader : public rclcpp::Node {
public:
    PointCloudReader() : Node("pointcloud_reader") {
        RCLCPP_INFO(this->get_logger(), "Starting PointCloud Reader Node");

        // Specify the path to your LAS file
        std::string filename = "path/to/your/file.las";
        std::ifstream ifs;
        ifs.open(filename.c_str(), std::ios::in | std::ios::binary);

        if (ifs.is_open()) {
            liblas::ReaderFactory f;
            liblas::Reader reader = f.CreateWithStream(ifs);

            while (reader.ReadNextPoint()) {
                const liblas::Point& p = reader.GetPoint();
                // Process your point here
                RCLCPP_INFO(this->get_logger(), "Point: (%f, %f, %f)", p.GetX(), p.GetY(), p.GetZ());
            }
            ifs.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open LAS file");
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudReader>());
    rclcpp::shutdown();
    return 0;
}
