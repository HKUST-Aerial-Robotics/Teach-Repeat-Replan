#pragma once

#include <std_msgs/Header.h>
#include <mutex>
#include <queue>

struct SyncSession_t {
    std::mutex locker;
    enum class Status { Free, RecvReq, ForwardReq, RecvAck, ProcessAck };
    Status status;
    int freq;
    std_msgs::Header header;

    SyncSession_t() : status(Status::Free), freq(0){};
};

class SyncReqInfo {
  public:
    static constexpr int SingleRequestValue = 0;
    int freq;
    SyncReqInfo(int freq_) : freq(freq_){};
    SyncReqInfo() : SyncReqInfo(-1){};
    SyncReqInfo(const SyncReqInfo& other) : SyncReqInfo(other.freq){};
};

class SyncAckInfo {
  public:
    ros::Time stamp;
    int seq;
    SyncAckInfo(const ros::Time& stamp_, const int seq_) : stamp(stamp_), seq(seq_){};
    SyncAckInfo() : SyncAckInfo(ros::Time(0), -1){};
    SyncAckInfo(const SyncAckInfo& other) : SyncAckInfo(other.stamp, other.seq){};
};

class HardwareSynchronizer {
  public:
    std::queue<SyncReqInfo> req_queue;
    std::mutex req_mutex;
    std::queue<SyncAckInfo> ack_queue;
    std::mutex ack_mutex;

    HardwareSynchronizer() {
        // req_queue.reserve(100);
        // ack_queue.reserve(100);
    };

  private:
};