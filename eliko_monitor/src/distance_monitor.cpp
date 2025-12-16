#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <deque>  
#include <regex>
#include <iomanip>
#include <iostream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

// ANSI color codes
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BOLD    "\033[1m"
#define CLEAR   "\033[2J\033[1;1H" 

struct TopicData {
    int32_t last_distance_cm;
    // We use a deque to store timestamps of received messages.
    // This allows exact calculation of messages per second (Sliding Window).
    std::deque<rclcpp::Time> history_buffer; 
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub;
    std::string anchor_id;
    std::string tag_id;
    bool active;
};

class DistanceMonitor : public rclcpp::Node
{
public:
    DistanceMonitor() : Node("distance_monitor")
    {
        // Parameters
        this->declare_parameter("target_frequency", 100.0);
        target_freq_ = this->get_parameter("target_frequency").as_double();

        this->declare_parameter("report_frequency", 1.0);
        double report_freq = this->get_parameter("report_frequency").as_double();
        if (report_freq <= 0.0) report_freq = 1.0;

        // Timer 1: Discovery (2s)
        discovery_timer_ = this->create_wall_timer(
            2000ms, std::bind(&DistanceMonitor::discover_topics, this));

        // Timer 2: UI Report
        discovery_timer_ = this->create_wall_timer(
            2000ms, std::bind(&DistanceMonitor::discover_topics, this));

        auto report_period = std::chrono::duration<double>(1.0 / report_freq);
        report_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(report_period),
            std::bind(&DistanceMonitor::print_stats, this));

        std::cout << CLEAR;
        RCLCPP_INFO(this->get_logger(), "Monitor Started. Target: %.1f Hz | UI: %.1f Hz", target_freq_, report_freq);
    }

private:
    void discover_topics()
    {
        auto topic_names_and_types = this->get_topic_names_and_types();
        std::regex pattern(R"(/Distance/A(.*?)/T(.*))");
        std::smatch matches;

        for (const auto& topic : topic_names_and_types) {
            std::string name = topic.first;
            if (monitored_topics_.find(name) != monitored_topics_.end()) continue;

            if (std::regex_search(name, matches, pattern)) {
                for (const auto& type : topic.second) {
                    if (type == "std_msgs/msg/Int32") {
                        subscribe_to_topic(name, matches[1].str(), matches[2].str());
                        break;
                    }
                }
            }
        }
    }

    void subscribe_to_topic(const std::string& topic_name, const std::string& aid, const std::string& tid)
    {
        TopicData data;
        data.anchor_id = aid;
        data.tag_id = tid;
        data.last_distance_cm = 0;
        data.active = false;

        data.sub = this->create_subscription<std_msgs::msg::Int32>(
            topic_name, 
            rclcpp::SensorDataQoS(),
            [this, topic_name](const std_msgs::msg::Int32::SharedPtr msg) {
                this->topic_callback(topic_name, msg);
            });

        monitored_topics_[topic_name] = data;
    }

    void topic_callback(const std::string& topic_name, const std_msgs::msg::Int32::SharedPtr msg)
    {
        auto it = monitored_topics_.find(topic_name);
        if (it != monitored_topics_.end()) {
            it->second.last_distance_cm = msg->data;
            it->second.active = true;
            
            // Add current timestamp to the history buffer
            it->second.history_buffer.push_back(this->now());
        }
    }

    void print_stats()
    {
        std::cout << CLEAR;

        const int id_width = 10; 
        const int data_width = 14; 
        const int total_width = (id_width * 2) + (data_width * 3);
        std::string separator_line(total_width, '-');
        std::string group_separator = "";
        for(int i=0; i < total_width/2; i++) group_separator += "- ";
        if(group_separator.length() < (size_t)total_width) group_separator += "-";

        // --- Dynamic header ---
        std::string title_text = "ELIKO UWB NETWORK STATUS";
        int title_len = title_text.length();
        int padding_len = (total_width - title_len) / 2;
        if (padding_len < 2) padding_len = 2; 
        
        std::string padding_str(padding_len - 1, '='); 

        std::cout << BOLD;
        std::cout << padding_str << " " << title_text << " " << padding_str;
        if ((padding_str.length() * 2 + title_len + 2) < (size_t)total_width) std::cout << "=";
        std::cout << RESET << std::endl;

        std::cout << "Target Frequency: " << target_freq_ << " Hz" << std::endl;
        std::cout << "Total Topics: " << monitored_topics_.size() << std::endl;
        std::cout << separator_line << std::endl;
        
        std::cout << std::left 
                  << BOLD << std::setw(id_width) << "TAG ID" 
                  << std::setw(data_width) << "ANCHOR ID" 
                  << std::setw(data_width) << "DISTANCE (m)" 
                  << std::setw(data_width) << "RX RATE (Hz)" 
                  << std::setw(data_width) << "LOSS (%)" << RESET << std::endl;
        std::cout << separator_line << std::endl;

        if (monitored_topics_.empty()) {
            std::cout << "Waiting for topics /Distance/..." << std::endl;
            return;
        }

        std::map<std::string, std::vector<TopicData*>> grouped_data;
        for (auto& pair : monitored_topics_) {
            grouped_data[pair.second.tag_id].push_back(&pair.second);
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration one_second(1, 0); 

        size_t tag_counter = 0;
        for (auto& group : grouped_data) {
            std::vector<TopicData*>& anchors_list = group.second;

            std::sort(anchors_list.begin(), anchors_list.end(), 
                [](TopicData* a, TopicData* b) {
                    return a->anchor_id < b->anchor_id;
                });

            for (TopicData* data : anchors_list) {
                // Buffer cleaning
                while (!data->history_buffer.empty() && 
                      (now - data->history_buffer.front()) > one_second) {
                    data->history_buffer.pop_front();
                }

                if (!data->active && data->history_buffer.empty()) continue; 

                double distance_m = data->last_distance_cm / 100.0;
                double real_freq_hz = (double)data->history_buffer.size(); 
                
                double loss_percent = 0.0;
                std::string color_code = GREEN;

                if (target_freq_ > 0) {
                    loss_percent = ((target_freq_ - real_freq_hz) / target_freq_) * 100.0;
                    if (loss_percent < 0.0) loss_percent = 0.0;

                    if (loss_percent < 25.0) color_code = GREEN;
                    else if (loss_percent <= 50.0) color_code = YELLOW;
                    else color_code = RED;
                }

                // Print rows using the two widths defined
                std::cout << std::left 
                          << std::setw(id_width) << data->tag_id 
                          << std::setw(data_width+2) << data->anchor_id 
                          << std::setw(data_width) << std::fixed << std::setprecision(2) << distance_m 
                          << std::setw(data_width) << std::setprecision(1) << real_freq_hz 
                          << color_code << std::setw(data_width) << std::setprecision(1) << loss_percent << RESET << std::endl;
            }

            tag_counter++;
            if (tag_counter < grouped_data.size()) {
                 std::cout << group_separator << std::endl;
            }
        }
        
        std::cout << separator_line << std::endl;
        std::cout << "Press Ctrl+C to exit" << std::endl;
    }

    rclcpp::TimerBase::SharedPtr discovery_timer_;
    rclcpp::TimerBase::SharedPtr report_timer_;
    std::map<std::string, TopicData> monitored_topics_;
    double target_freq_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceMonitor>());
    rclcpp::shutdown();
    return 0;
}