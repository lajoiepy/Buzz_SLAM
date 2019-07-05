#ifndef BUZZ_SLAM_SINGLETON_H
#define BUZZ_SLAM_SINGLETON_H

#include "specialized/no_sensing/buzz_slam_no_sensing.h"
#include "specialized/dataset/buzz_slam_dataset.h"
#include <memory>
#include <utility>
#include <map>

namespace buzz_slam {

/*
* Buzz SLAM Singleton class, it manages the instances of buzz SLAM.
*/
class BuzzSLAMSingleton {
    public:
        static BuzzSLAMSingleton& GetInstance()
        {
            static BuzzSLAMSingleton instance; 
            return instance;
        }

        template <class T>
        void InsertBuzzSLAM(const int& robot_id) {
            buzz_slam_map_.insert(std::make_pair(robot_id, std::unique_ptr<T>(new T)));
        }

        std::unique_ptr<BuzzSLAM>& GetBuzzSLAM(const int& robot_id) {
            return buzz_slam_map_[robot_id];
        }

    private:
        BuzzSLAMSingleton() {}       

    public:
        BuzzSLAMSingleton(BuzzSLAMSingleton const&) = delete;
        void operator=(BuzzSLAMSingleton const&) = delete;

    private:
        std::map<int, std::unique_ptr<BuzzSLAM>> buzz_slam_map_;
};
}

#endif