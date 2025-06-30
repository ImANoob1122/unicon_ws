#pragma once

#include "rogilink_flex_interfaces/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "Serializer.hpp"

constexpr uint8_t MAX_FRAME_LENGTH = 256;

// TODO: 動作確認
namespace RogiLinkLib
{
    template <typename... Args>
    class RogiLinkPublisher
    {
        private:
            rclcpp::Publisher<rogilink_flex_interfaces::msg::Frame>::SharedPtr publisher;
            std::string topic_name;

        public:
            RogiLinkPublisher(
                rclcpp::Node::SharedPtr node,
                std::string topic_name;
                uint8_t qos = 10
            )
            {
                publisher = node->create_publisher<rogilink_flex_interfaces::msg::Frame>(
                    "rogilink_transmission/" + topic_name, qos
                );
            }

            void publish(Args... args)
            {
                rogilink_flex_interfaces::msg::Frame frame;
                uint8_t buffer[MAX_FRAME_LENGTH];
                size_t size;
                serialize(buffer, &size, args...);
                for (int i = 0; i < size; i++)
                {
                    frame.data.push_back(buffer[i]);
                }
                publisher->publish(frame);
            }
    };

    template <typename... Args>
    class RogiLinkSubscription
    {
        private:
            rclcpp::Subscription<rogilink_flex_interfaces::msg::Frame>::SharedPtr subscription;
            std::function<void(Args...)> callback;
            rclcpp::Node::SharedPtr node;
            TupleDeserializer<Args...> deserializer;

        public:
            RogiLinkSubscription(
                rclcpp::Node::SharedPtr node,
                std::string topic_name;
                std::function<void(Args...)> callback,
                uint8_t qos = 10
            )
            {
                this->callback = callback;
                this->node = node;
                std::string topic_name = "rogiLink_reception/" + topic_name;
                subscription = node->create_subscription<rogilink_flex_interfaces::msg::Frame>(
                    topic_name, qos,
                    std::bind(&RogiLinkSubscription::topic_callback, this, std::placeholders::_1)
                );
            }

            void topic_callback(rogilink_flex_interfaces::msg::Frame::SharedPtr msg)
            {
                uint8_t buffer[MAX_FRAME_LENGTH];
                for (int i = 0; i < msg->data.size(); i++)
                {
                    buffer[i] = msg->data[i];
                }
                std::tuple<Args...> args;
                deserializer.deserialize_tuple(buffer, args, size); // デシリアライズ

                std::apply(callback, args); // ここでコールバックを呼び出す
            }
    };
} 