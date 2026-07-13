#include <chrono>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <utility>


//全局的互斥锁
std::mutex out_put_mutex;
//摄像头结构体
struct cameraspec{
    std::string device;
    std::string output;  

};
//摄像头类 包括V4L2 dmabuf mpp gst 四大部分
class CameraSession
{
    private:
    std::size_t index_;
    cameraspec camera_;

    public:

    CameraSession(std::size_t index, cameraspec camera): index_(index),camera_(std::move(camera))
    { 
    }
    void run()
    {
        for(int frame = 0 ; frame < 5 ; ++frame)
        {    
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::lock_guard<std::mutex>lock(out_put_mutex);
            std::cout << "[camera" << index_ <<"]"
            <<" device= "<<camera_.device
            <<" output= "<<camera_.output
            <<" frame= "<< frame 
            <<std::endl;
        };
    }

};
int main()
{
    std::vector<cameraspec> cameras
    {
        {"/dev/video22","camera22.mp4"},{"/dev/video31","camera32.mp4"}

    };

    std::vector<std::thread> workers;
    for(std::size_t index = 0; index < cameras.size(); ++index)
    {
        workers.emplace_back([&,index]
        {
            CameraSession session(index , cameras[index]);
           session.run(); 

        });
    };
    for (std::thread& worker : workers)
    {
        worker.join();



    }
    return 0 ;

}