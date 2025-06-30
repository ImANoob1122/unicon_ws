#include "rclcpp/rclcpp.hpp"
#include "uart.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <memory>
#include <regex>
#include "rogilink_flex_interfaces/msg/frame.hpp"
#include "rogilink_flex_interfaces/srv/is_connected.hpp"
#include "rogilink_flex_interfaces/srv/get_config.hpp"
#include "nlohmann/json.hpp"
#include <unistd.h>

using std::placeholders::_1;
using std::placeholders::_2;

using json = nlohmann::json;

const uint8_t SYSTEM_MESSAGE_FRAME_ID = 0x00;
typedef enum {
    CONNECTION_CHECK = 0x01,
    CONNECTION_CHECK_REPLY = 0x02,
} SystemMessageID;
const std::chrono::milliseconds CONNECTION_TIMEOUT = std::chrono::milliseconds(200);

class UartLinkNode : public rclcpp::Node
{
    private:
        rclcpp::TimerBase::SharedPtr read_timer;
        std::shared_ptr<UartLink> uart;

        // フレームIDとフレーム名のマップ
        std::map<uint8_t, std::string> reception_frame_map;
        // フレームIDとフレーム名のマップ
        std::map<uint8_t, std::string> transmission_frame_map;

        // フレームIDとパブリッシャのマップ
        std::map<uint8_t, rclcpp::Publisher<rogilink_flex_interfaces::msg::Frame>::SharedPtr> pub_map;
        // フレームIDとサブスクライバのマップ
        std::map<uint8_t, rclcpp::Subscription<rogilink_flex_interfaces::msg::Frame>::SharedPtr> sub_map;
        rclcpp::Subscription<rogilink_flex_interfaces::msg::Frame>::SharedPtr general_sub;

        //IsConnectedサービス
        rclcpp::Service<rogilink_flex_interfaces::srv::IsConnected>::SharedPtr is_connected_service;

        // config server
        rclcpp::Service<rogilink_flex_interfaces::srv::GetConfig>::SharedPtr get_config_service;
        
        std::vector<std::string> device_list;
        std::chrono::system_clock::time_point connected_time;
        bool is_connected = false;
        uint8_t device_id;

        std::string config_path;

        rclcpp::TimerBase::SharedPtr connection_timer;

        BaudRate baud_rate;
        
    public:
        UartLinkNode(): Node("uartlink")
        {
            uart = nullptr;
            device_list = UartLink::get_device_list(); // デバイスリストを取得

            declare_parameter("config_path", "/home/ryunosuke/ros2_ws/src/Catch2024SetoGiwa/settings/rogilinkflex.json"); // 設定ファイルのパスのパラメータを宣言
            config_path = get_parameter("config_path").as_string(); // パラメータから設定ファイルのパスを取得
            std::ifstream f(config_path);
            json config = json::parse(f); // 設定ファイルを読み込む

            device_id = config["device_id"]; // デバイスIDを取得
            
            if (config.find("baud_rate") != config.end())
            {
                int baud_rate_int = config["baud_rate"];
                switch (baud_rate_int) {
                    case 9600:
                        baud_rate = BaudRate::B_9600;
                        break;
                    case 19200:
                        baud_rate = BaudRate::B_19200;
                        break;
                    case 38400:
                        baud_rate = BaudRate::B_38400;
                        break;
                    case 57600:
                        baud_rate = BaudRate::B_57600;
                        break;
                    case 115200:
                        baud_rate = BaudRate::B_115200;
                        break;
                    default:
                        baud_rate = BaudRate::B_115200;
                }
            }else{
                baud_rate = BaudRate::B_115200;
            }


            // 受信publisherの作成
            for (auto& msg : config["reception_messages"]) {
                uint8_t id = msg["id"];
                std::string name = msg["name"];
                reception_frame_map[id] = name;
                pub_map[id] = this->create_publisher<rogilink_flex_interfaces::msg::Frame>("rogilink_reception/" + name, 10); // フレームのパブリッシャを作成
            }

            // 送信subscriberの作成
            for (auto& msg : config["transmission_messages"]) {
                uint8_t id = msg["id"];
                std::string name = msg["name"];
                transmission_frame_map[id] = name;
                sub_map[id] = this->create_subscription<rogilink_flex_interfaces::msg::Frame>("rogilink_transmission/" + name, 10, 
                    [this, id](rogilink_flex_interfaces::msg::Frame::SharedPtr msg){
                        this->frame_callback(msg, id);
                    }
                ); // フレームのサブスクライバを作成
            }

            general_sub = this->create_subscription<rogilink_flex_interfaces::msg::Frame>("rogilink_general_transmission", 10, 
                [this](rogilink_flex_interfaces::msg::Frame::SharedPtr msg){
                    this->frame_callback(msg, msg->frame_id);
                }
            ); // フレームのサブスクライバを作成

            is_connected_service = this->create_service<rogilink_flex_interfaces::srv::IsConnected>("is_connected", 
                [this](const std::shared_ptr<rogilink_flex_interfaces::srv::IsConnected::Request> request, std::shared_ptr<rogilink_flex_interfaces::srv::IsConnected::Response> response){
                    response->connected = is_connected;
                    response->device_id = device_id;
                }
            ); // IsConnectedサービスを作成

            get_config_service = this->create_service<rogilink_flex_interfaces::srv::GetConfig>("get_config", 
                [this](const std::shared_ptr<rogilink_flex_interfaces::srv::GetConfig::Request> request, std::shared_ptr<rogilink_flex_interfaces::srv::GetConfig::Response> response){
                    response->path = config_path;
                }
            ); // GetConfigサービスを作成
        }

        // メインループ
        void mainloop(){
            while (rclcpp::ok())
            {
                try{
                    loop();
                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(), e.what());
                    disconnect();
                }
            }
        }

        void loop(){
            if (uart == nullptr)
            {
                try_connect(); // 接続を試みる
            }

            if (!is_connected) {
                if (std::chrono::system_clock::now() - connected_time > CONNECTION_TIMEOUT) {
                    if (uart != nullptr) {
                        int v = uart->reset_device();
                        // RCLCPP_INFO(get_logger(), "Connection timeout. Disconnecting... (%d)", v);
                    }
                    disconnect();
                }
            }

            if (uart != nullptr){
                uart->loop(); // 受信ループ
            }
            rclcpp::spin_some(shared_from_this()); // ros2のコールバックを呼ぶ
        }

        void init(std::string device)
        {
            uart = std::make_shared<UartLink>(device, baud_rate); // UartLinkクラスを初期化
            uart->set_callback(SYSTEM_MESSAGE_FRAME_ID, std::bind(&UartLinkNode::system_message_callback, this, _1)); // システムメッセージのコールバックをセット
            uart->set_general_callback(std::bind(&UartLinkNode::general_message_callback, this, _1, _2)); // すべてのメッセージのコールバックをセット
            uart->set_disconnect_callback(std::bind(&UartLinkNode::disconnect, this)); // 切断時のコールバックをセット
        }

        void try_connect()
        {
            if (device_list.size() == 0)
            {
                // RCLCPP_INFO(get_logger(), "No device found. Trying to search again...");
                device_list = UartLink::get_device_list(); // デバイスリストを再取得
            }

            if (device_list.size() == 0)
            {
                return;
            }

            auto device = device_list.at(0); // 0番目のデバイスを選択
            device_list.erase(device_list.begin()); // 0番目のデバイスを削除

            // RCLCPP_INFO(get_logger(), "Trying to connect to %s", device.c_str());
            init(device); // デバイスに接続
            uart->send({CONNECTION_CHECK}, SYSTEM_MESSAGE_FRAME_ID); // 接続確認メッセージを送信
            connected_time = std::chrono::system_clock::now(); // 接続時刻を記録

            // 接続確認を無効にする場合は以下を1に変更
            #if 0
                connection_reply_callback(device_id);
            #endif
        }

        // 接続確認メッセージを送信
        void disconnect() 
        {
            uart = nullptr; // UartLinkクラスを削除
            is_connected = false; // 接続フラグをfalseに
            // RCLCPP_INFO(get_logger(), "Disconnected. Trying to reconnect...");
        }

        // システムメッセージのコールバック
        void system_message_callback(std::vector<uint8_t> data)
        {
            RCLCPP_INFO(get_logger(), "System message received");
            if (data.size() == 0)
            {
                return;
            }

            uint8_t id = data.at(0);
            std::vector<uint8_t> payload(data.begin() + 1, data.end());

            switch (id)
            {
                case CONNECTION_CHECK_REPLY:
                    //is_connected = true;
                    connected_time = std::chrono::system_clock::now();
                    if (payload.size() == 0) return; // ペイロードが空の場合は処理しない
                    connection_reply_callback(payload.at(0));
                    break;
                default:
                    break;
            }
        }

        // 接続確認メッセージの返信を受信したときの処理
        void connection_reply_callback(uint8_t id)
        {
            if (id == device_id)
            {
                is_connected = true; // 接続完了
                RCLCPP_INFO(get_logger(), "Connected to device %d successfully", id);
            }else{
                RCLCPP_INFO(get_logger(), "Connected to device %d failed", id);
                disconnect(); // 接続失敗
                try_connect(); // 再接続を試みる
            }
        }

        // すべてのメッセージのコールバック
        void general_message_callback(uint8_t id, std::vector<uint8_t> data)
        {

            if (id == SYSTEM_MESSAGE_FRAME_ID) return; // システムメッセージは処理しない
            
            if (!is_connected) return; // 接続されていない場合は処理しない

            auto frame = rogilink_flex_interfaces::msg::Frame();
            frame.frame_id = id;
            frame.device_id = device_id;
            frame.data.clear();
            for (auto byte : data)
            {
                frame.data.push_back(byte);
            }
            
            if (pub_map.find(id) != pub_map.end())
            {
                pub_map[id]->publish(frame); // フレームをパブリッシュ
            }else{
                pub_map[id] = this->create_publisher<rogilink_flex_interfaces::msg::Frame>("rogilink_reception/" + std::to_string(id), 10);
                pub_map[id]->publish(frame); // フレームをパブリッシュ
            }
        }

        // フレームのコールバック
        void frame_callback(const rogilink_flex_interfaces::msg::Frame::SharedPtr msg, uint8_t id)
        {
            if (!is_connected) return; // 接続されていない場合は処理しない
            if (msg->device_id != device_id) return; // デバイスIDが一致しない場合は処理しない

            std::vector<uint8_t> data;
            for (auto byte : msg->data)
            {
                data.push_back(byte);
            }
            uart->send(data, id); // フレームを送信
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UartLinkNode>();
    node->mainloop();
    rclcpp::shutdown();
    return 0;
}