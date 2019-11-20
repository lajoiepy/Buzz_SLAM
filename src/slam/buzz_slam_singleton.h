#ifndef BUZZ_SLAM_SINGLETON_H
#define BUZZ_SLAM_SINGLETON_H

#include "specialized/no_sensing/buzz_slam_no_sensing.h"
#include "specialized/dataset/buzz_slam_dataset.h"
#include "specialized/ros/buzz_slam_ros.h"
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
        T* GetBuzzSLAM(const int& robot_id) {
            if (buzz_slam_map_.count(robot_id) == 0) {
                buzz_slam_map_.insert(std::make_pair(robot_id, new T()));
            }
            return static_cast<T*>(buzz_slam_map_[robot_id]);
        }

        void Destroy() {
            if ( !is_destroyed ) {
                for (auto buzz_slam : buzz_slam_map_) {
                    delete buzz_slam.second;
                }
                is_destroyed = true;
            }
        }

    private:
        BuzzSLAMSingleton() {}       

    public:
        BuzzSLAMSingleton(BuzzSLAMSingleton const&) = delete;
        void operator=(BuzzSLAMSingleton const&) = delete;

    private:
        std::map<int, BuzzSLAM*> buzz_slam_map_;
        bool is_destroyed = false;
};
}

#endif