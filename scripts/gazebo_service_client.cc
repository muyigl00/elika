#include <iostream>
#include <string>
#include <vector>
#include <google/protobuf/text_format.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>

template<class Request>
std::string request(gz::transport::Node &node, const std::string &endpoint,
                    const std::string &text) {
  Request message;
  if (!google::protobuf::TextFormat::ParseFromString(text, &message))
    return "error: invalid protobuf request";
  gz::msgs::Boolean reply;
  bool accepted = false;
  const std::string service = endpoint.front() == '/'
      ? endpoint : "/world/shahed_detection/" + endpoint;
  if (!node.Request(service, message, 5000u, reply, accepted))
    return "timeout";
  return accepted && reply.data() ? "ok" : "error: service rejected request";
}

int main() {
  gz::transport::Node node;
  std::cout << "ready" << std::endl;
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line == "list") {
      std::vector<std::string> services;
      node.ServiceList(services);
      std::cout << "services";
      for (const auto &service : services) std::cout << '\t' << service;
      std::cout << std::endl;
      continue;
    }
    const auto first = line.find('\t');
    const auto second = line.find('\t', first == std::string::npos ? 0 : first + 1);
    if (first == std::string::npos || second == std::string::npos) {
      std::cout << "error: invalid protocol line" << std::endl;
      continue;
    }
    const auto endpoint = line.substr(0, first);
    const auto kind = line.substr(first + 1, second - first - 1);
    const auto body = line.substr(second + 1);
    std::string result;
    if (kind == "Pose") result = request<gz::msgs::Pose>(node, endpoint, body);
    else if (kind == "WorldControl") result = request<gz::msgs::WorldControl>(node, endpoint, body);
    else result = "error: unsupported request type";
    std::cout << result << std::endl;
  }
}

