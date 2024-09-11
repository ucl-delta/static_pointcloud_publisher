#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <liblas/liblas.hpp>
#include <fstream>

class PointCloudReader : public rclcpp::Node {
public:
    PointCloudReader() : Node("pointcloud_reader") {
        // Declare and get the parameter for LAS file path
        this->declare_parameter<std::string>("frame_id", "base_frame");
        this->declare_parameter<std::string>("las_file_path", "");
        
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("las_file_path", las_file_path_);
        this->read_in_file(las_file_path_);

        // Initialize the publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

        // Initialize the timer to periodically check for subscribers
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PointCloudReader::check_and_publish, this)
        );        
    }

private:
    void read_in_file(std::string las_file_path_) {
        std::ifstream ifs(las_file_path_, std::ios::in | std::ios::binary);
        if (!ifs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open LAS file: %s", las_file_path_.c_str());
            return;
        }

        liblas::ReaderFactory readerFactory;
        liblas::Reader reader = readerFactory.CreateWithStream(ifs);

        // Prepare PointCloud2 message
        // sensor_msgs::msg::PointCloud2 pointcloud_msg;
        this->pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        this->pointcloud_msg->header.stamp = this->now();
        this->pointcloud_msg->header.frame_id = frame_id; // Adjust the frame as necessary
        this->pointcloud_msg->height = 1;
        this->pointcloud_msg->is_dense = false;

        // Define fields (x, y, z, rgb)
        sensor_msgs::msg::PointField field;
        field.name = "x";
        field.offset = 0;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        this->pointcloud_msg->fields.push_back(field);

        field.name = "y";
        field.offset = 4;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        this->pointcloud_msg->fields.push_back(field);

        field.name = "z";
        field.offset = 8;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        this->pointcloud_msg->fields.push_back(field);

        field.name = "rgb";
        field.offset = 12;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32; // RGB packed in a float
        field.count = 1;
        this->pointcloud_msg->fields.push_back(field);

        this->pointcloud_msg->point_step = 16; // 4 * 4 bytes (x, y, z, rgb)
        this->pointcloud_msg->row_step = 0; // Will be set after reading points

        std::vector<uint8_t> data;

        while (reader.ReadNextPoint()) {
            const liblas::Point& p = reader.GetPoint();
            
            float x = p.GetX();
            float y = p.GetY();
            float z = p.GetZ();
            uint8_t r = p.GetColor().GetRed() >> 8;
            uint8_t g = p.GetColor().GetGreen() >> 8;
            uint8_t b = p.GetColor().GetBlue() >> 8;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
                           (static_cast<uint32_t>(g) << 8) |
                           (static_cast<uint32_t>(b));
            float rgb_float;
            std::memcpy(&rgb_float, &rgb, sizeof(uint32_t));
            
            // Append the point data
            data.insert(data.end(), reinterpret_cast<const uint8_t*>(&x), reinterpret_cast<const uint8_t*>(&x) + sizeof(float));
            data.insert(data.end(), reinterpret_cast<const uint8_t*>(&y), reinterpret_cast<const uint8_t*>(&y) + sizeof(float));
            data.insert(data.end(), reinterpret_cast<const uint8_t*>(&z), reinterpret_cast<const uint8_t*>(&z) + sizeof(float));
            data.insert(data.end(), reinterpret_cast<const uint8_t*>(&rgb_float), reinterpret_cast<const uint8_t*>(&rgb_float) + sizeof(float));
        }

        this->pointcloud_msg->width = data.size() / this->pointcloud_msg->point_step;
        this->pointcloud_msg->row_step = this->pointcloud_msg->point_step * this->pointcloud_msg->width;
        this->pointcloud_msg->data.resize(data.size());
        std::copy(data.begin(), data.end(), this->pointcloud_msg->data.begin());
    }

    void check_and_publish() {
        // Check the number of subscribers
        uint8_t curr_num_publishers = publisher_->get_subscription_count();
        if (curr_num_publishers > 0 && curr_num_publishers > num_publishers) {
            RCLCPP_INFO(this->get_logger(), "Publishing pointcloud...");
            publisher_->publish(*this->pointcloud_msg);
        }
        num_publishers = curr_num_publishers;
    }

    std::string frame_id;
    uint8_t num_publishers = 0;
    std::string las_file_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> pointcloud_msg;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
