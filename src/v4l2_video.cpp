//把264/265包送入gstreamer管线
#include <gst/app/gstappsrc.h>
//gst核心api
#include <gst/gst.h>

//打开摄像头设备和设置文件描述符
#include <fcntl.h>
//V4L2核心定义
#include <linux/videodev2.h>
//等待摄像头新帧产生 不循环占用
#include <poll.h>
//捕获系统停止信号等
#include <signal.h>
//向V4L2设备发送控制命令
#include <sys/ioctl.h>
//posix基础系统接口
#include <unistd.h>

//管理mpp使用的内存缓冲区 分配dmabuf
#include <mpp_buffer.h>
//描述送入编码器的原始图像帧
#include <mpp_frame.h>
//表示编码器输出的 264 265数据包
#include <mpp_packet.h>
//rockchip mpp 主接口 创建初始化操作硬件编码器
#include <rk_mpi.h>
//配置视频编码器
#include <rk_venc_cfg.h> 

//C++标准库
#include <cerrno>
#include <atomic>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>


namespace 
{

    volatile sig_atomic_t signal_requested = 0;
    std::atomic_bool stop_requested{false};
    std::mutex output_mutex;

    void signal_handler(int) {
        signal_requested = 1;
    }

    bool should_run() 
    {
        return signal_requested == 0 &&!stop_requested.load(std::memory_order_relaxed);
    }

    void require(bool ok, const std::string& message) 
    {
        if (!ok) {
            throw std::runtime_error(message);
        }
    }

    int xioctl(int fd, unsigned long request, void* arg) 
    {
        int result;
        do 
        {
            result = ioctl(fd, request, arg);
        } 
        while (result < 0 && errno == EINTR);
        return result;
    }

    // uint32_t align_up(uint32_t value, uint32_t alignment) {
    //     return (value + alignment - 1) & ~(alignment - 1);
    // }

    std::string fourcc_to_string(uint32_t value) 
    {
        char text[5] = {
            static_cast<char>(value & 0xff),
            static_cast<char>((value >> 8) & 0xff),
            static_cast<char>((value >> 16) & 0xff),
            static_cast<char>((value >> 24) & 0xff),
            '\0',
        };
        return text;
    }

    enum class VideoCodec 
    {
        H264,
        H265,
    };

    VideoCodec parse_codec(std::string value) 
    {
        for (char& character : value) {
            character = static_cast<char>(
                std::tolower(static_cast<unsigned char>(character)));
        }

        if (value == "h264" || value == "avc") 
        {
            return VideoCodec::H264;
        }
        if (value == "h265" || value == "hevc") 
        {
            return VideoCodec::H265;
        }
        throw std::runtime_error(
            "invalid codec '" + value + "' (expected h264 or h265)");
    }

    const char* codec_name(VideoCodec codec) 
    {
        return codec == VideoCodec::H264 ? "h264" : "h265";
    }

    MppCodingType mpp_coding_type(VideoCodec codec) 
    {
        return codec == VideoCodec::H264 ? MPP_VIDEO_CodingAVC : MPP_VIDEO_CodingHEVC;
    }

    const char* gst_parser_name(VideoCodec codec) 
    {
        return codec == VideoCodec::H264 ? "h264parse" : "h265parse";
    }

    const char* gst_media_type(VideoCodec codec) 
    {
        return codec == VideoCodec::H264 ? "video/x-h264" : "video/x-h265";
    }

    struct CaptureBuffer 
    {
        int dma_fd = -1;
        size_t length = 0;
        MppBuffer mpp_buffer = nullptr;
    };

    struct CapturedFrame 
    {
        uint32_t index = 0;
        uint32_t sequence = 0;
        int64_t timestamp_ns = 0;
    };

    //负责打开摄像头、配置格式、申请缓冲、建立DMABUF零拷贝关系、开始/停止采集、 取帧、归还帧和释放资源 
    //一路摄像头采集资源的管理器
    class V4L2Capture
    {

    public:
        V4L2Capture(const std::string& device,
                    uint32_t width,
                    uint32_t height,
                    uint32_t fps,
                    uint32_t buffer_count)
            : requested_width_(width), requested_height_(height) 
            {
            fd_ = open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
            require(fd_ >= 0, "open " + device + " failed: " + std::strerror(errno));

            v4l2_capability capability{};
            //查询能力
            require(xioctl(fd_, VIDIOC_QUERYCAP, &capability) == 0,
                    "VIDIOC_QUERYCAP failed: " + std::string(std::strerror(errno)));
            //获取能力
            uint32_t caps = capability.capabilities;

            if (caps & V4L2_CAP_DEVICE_CAPS) 
            {
                caps = capability.device_caps;
            }
            //查询是否支持多平面采集能力
            require(caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE,
                    "device does not support multiplanar capture");
            //检查是否支持Streaming
            require(caps & V4L2_CAP_STREAMING,
                    "device does not support streaming I/O");
            //配置图像格式 
            configure_format();
            //配置fps
            configure_fps(fps);
            //申请V4L2缓冲，导出DMABUF，并导入MPP，建立零拷贝关系
            allocate_and_import_buffers(buffer_count);
            //把buffer放入队列
            queue_all_buffers();
            }
        //禁止复制初始化和复制赋值
        V4L2Capture(const V4L2Capture&) = delete;
        V4L2Capture& operator =(const V4L2Capture&) = delete;
        //析构函数 释放 销毁时候自动启用 mpp_buffer fd v4l2缓冲区 
        ~V4L2Capture()
         {
            stop();

            for (auto& buffer : buffers_) 
            {
                if (buffer.mpp_buffer) 
                {
                    mpp_buffer_put(buffer.mpp_buffer);
                    buffer.mpp_buffer = nullptr;
                }
                if (buffer.dma_fd >= 0) 
                {
                    close(buffer.dma_fd);
                    buffer.dma_fd = -1;
                }
            }

            if (fd_ >= 0) 
            {
                v4l2_requestbuffers request{};
                request.type = type_;
                request.memory = V4L2_MEMORY_MMAP;
                request.count = 0;
                xioctl(fd_, VIDIOC_REQBUFS, &request);
                close(fd_); 
                fd_ = -1;
            }
        }

        void start() 
        {
            require(fd_ >= 0, "cannot start a closed V4L2 device");
            require(!streaming_, "V4L2 stream is already running");

            v4l2_buf_type type = type_;
            require(xioctl(fd_, VIDIOC_STREAMON, &type) == 0,
                    "VIDIOC_STREAMON failed: " +
                        std::string(std::strerror(errno)));
            streaming_ = true;
        }

        void stop() 
        {
            if (!streaming_ || fd_ < 0)
            {
                return;
            }
                v4l2_buf_type type = type_;
                if (xioctl(fd_, VIDIOC_STREAMOFF, &type) < 0) 
            {
                std::cerr << "warning: VIDIOC_STREAMOFF failed: "
                        << std::strerror(errno) << std::endl;
            }
            streaming_ = false;
        }

        bool dequeue(CapturedFrame& frame) 
        {
            while (should_run()) 
            {
                //创建监控 
                pollfd descriptor{};
                descriptor.fd = fd_;
                descriptor.events = POLLIN;
                //等待2000ms 监控一个fd
                const int poll_result = poll(&descriptor, 1, 2000);
                if (poll_result < 0) 
                {
                    if (errno == EINTR) 
                    {
                        continue;
                    }
                    throw std::runtime_error("poll failed: " + std::string(std::strerror(errno)));
                }
                // = 0 赋值    == 0 比较
                if (poll_result == 0) 
                {
                    throw std::runtime_error("V4L2 capture timeout");
                }
                v4l2_buffer buffer{};
                v4l2_plane planes[VIDEO_MAX_PLANES]{};
                buffer.type = type_;
                buffer.memory = V4L2_MEMORY_MMAP;
                buffer.length = 1;
                buffer.m.planes = planes;
                //从V4L2完成队列中取出一个已经填好图像的缓冲
                if (xioctl(fd_, VIDIOC_DQBUF, &buffer) == 0) 
                {
                    require(buffer.index < buffers_.size(), "invalid V4L2 buffer index");
                    //填充到自定义帧结构中
                    frame.index = buffer.index;
                    frame.sequence = buffer.sequence;
                    //转换成ns 固定
                    frame.timestamp_ns =
                        static_cast<int64_t>(buffer.timestamp.tv_sec) *
                            1000000000LL +
                        static_cast<int64_t>(buffer.timestamp.tv_usec) * 1000LL;
                    return true;
                }
                if (errno != EAGAIN && errno != EINTR) 
                {
                    throw std::runtime_error("VIDIOC_DQBUF failed: " +
                                            std::string(std::strerror(errno)));
                }
            }
            return false;
        }
        //把已经编码完成的缓冲重新交给摄像头
        void requeue(uint32_t index)
         {
            require(index < buffers_.size(), "invalid V4L2 buffer index");

            v4l2_buffer buffer{};
            v4l2_plane planes[VIDEO_MAX_PLANES]{};
            buffer.type = type_;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = index;
            buffer.length = 1;
            buffer.m.planes = planes;

            require(xioctl(fd_, VIDIOC_QBUF, &buffer) == 0,
                    "VIDIOC_QBUF failed: " + std::string(std::strerror(errno)));
        }

        MppBuffer mpp_buffer(uint32_t index) const 
        {
            require(index < buffers_.size(), "invalid V4L2 buffer index");
            return buffers_[index].mpp_buffer;
        }

        void prepare_for_mpp(uint32_t index) const 
        {
            require(index < buffers_.size(), "invalid V4L2 buffer index");
            // Match Rockchip's official camera_source path. EXT_DMA buffers are
            // normally uncached, making this a no-op; on a cacheable exporter it
            // performs the required ownership transition before MPP reads it.
            require(mpp_buffer_sync_end(buffers_[index].mpp_buffer) == MPP_OK,
                    "mpp_buffer_sync_end failed");
        }
        //获取视频帧尺寸和布局信息的只读访问函数
        uint32_t width() const { return requested_width_; }
        uint32_t height() const { return requested_height_; }
        uint32_t horizontal_stride() const { return horizontal_stride_; }
        uint32_t vertical_stride() const { return vertical_stride_; }
        uint32_t storage_height() const { return storage_height_; }
        uint32_t size_image() const { return size_image_; }

    private:
        void configure_format() 
        {
            
            v4l2_format format{};
            format.type = type_;
            format.fmt.pix_mp.width = requested_width_;
            // RK3588 MPP uses a 16-line aligned NV12 storage height. The valid
            // image remains requested_height_ and is configured as the crop.
            format.fmt.pix_mp.height = requested_height_;
            format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
            format.fmt.pix_mp.field = V4L2_FIELD_NONE;

            require(xioctl(fd_, VIDIOC_S_FMT, &format) == 0,
                    "VIDIOC_S_FMT NV12 failed: " +
                        std::string(std::strerror(errno)));
            require(format.fmt.pix_mp.pixelformat == V4L2_PIX_FMT_NV12,
                    "driver changed NV12 to " +
                        fourcc_to_string(format.fmt.pix_mp.pixelformat));
            require(format.fmt.pix_mp.num_planes == 1,
                    "single-plane NV12 required, driver returned " +
                        std::to_string(format.fmt.pix_mp.num_planes) + " planes");
            require(format.fmt.pix_mp.width == requested_width_,
                    "driver changed capture width to " +
                        std::to_string(format.fmt.pix_mp.width));
            require(format.fmt.pix_mp.height >= requested_height_,
                    "driver returned a capture height smaller than requested");

            storage_height_ = format.fmt.pix_mp.height;
            //保存驱动返回的真实行跨度。
            horizontal_stride_ = format.fmt.pix_mp.plane_fmt[0].bytesperline;

            size_image_ = format.fmt.pix_mp.plane_fmt[0].sizeimage;

            require(horizontal_stride_ >= requested_width_, "invalid NV12 stride");

            const uint64_t minimum_size =
                static_cast<uint64_t>(horizontal_stride_) * storage_height_ * 3 / 2;
            require(size_image_ >= minimum_size,
                    "V4L2 NV12 buffer is too small for its returned stride/height");
            vertical_stride_ = storage_height_;

            std::cout << "V4L2 format=NV12 planes=1 visible="
                    << requested_width_ << 'x' << requested_height_
                    << " storage=" << horizontal_stride_ << 'x' << vertical_stride_
                    << " sizeimage=" << size_image_ << std::endl;
        }

        void configure_fps(uint32_t fps) 
        {
            v4l2_streamparm parameter{};
            parameter.type = type_;
            parameter.parm.capture.timeperframe.numerator = 1;
            parameter.parm.capture.timeperframe.denominator = fps;
            if (xioctl(fd_, VIDIOC_S_PARM, &parameter) < 0)
            {
                std::cerr << "warning: VIDIOC_S_PARM failed: "
                        << std::strerror(errno)
                        << "; application will pace/drop captured frames"
                        << std::endl;
            }
        }

        void allocate_and_import_buffers(uint32_t buffer_count) 
        {
            v4l2_requestbuffers request{};
            request.count = buffer_count;
            request.type = type_;
            request.memory = V4L2_MEMORY_MMAP;
            require(xioctl(fd_, VIDIOC_REQBUFS, &request) == 0,
                    "VIDIOC_REQBUFS failed: " + std::string(std::strerror(errno)));
            require(request.count >= 3, "V4L2 allocated too few capture buffers");

            buffers_.resize(request.count);
            for (uint32_t index = 0; index < request.count; ++index) 
            {
                v4l2_buffer buffer{};
                v4l2_plane planes[VIDEO_MAX_PLANES]{};
                buffer.type = type_;
                buffer.memory = V4L2_MEMORY_MMAP;
                buffer.index = index;
                buffer.length = 1;
                buffer.m.planes = planes;
                require(xioctl(fd_, VIDIOC_QUERYBUF, &buffer) == 0, "VIDIOC_QUERYBUF failed: " +  std::string(std::strerror(errno)));

                v4l2_exportbuffer exported{};
                exported.type = type_;
                exported.index = index;
                exported.plane = 0;
                exported.flags = O_CLOEXEC;
                require(xioctl(fd_, VIDIOC_EXPBUF, &exported) == 0,"VIDIOC_EXPBUF failed: " + std::string(std::strerror(errno)));

                CaptureBuffer& target = buffers_[index];
                target.dma_fd = exported.fd;
                target.length = planes[0].length;

                MppBufferInfo info{};
                info.type = MPP_BUFFER_TYPE_EXT_DMA;
                info.fd = target.dma_fd;
                info.size = target.length;
                info.index = index;
                require(mpp_buffer_import(&target.mpp_buffer, &info) == MPP_OK &&target.mpp_buffer,"mpp_buffer_import failed for V4L2 buffer " +std::to_string(index));
            }

            std::cout << "imported " << buffers_.size() << " persistent V4L2 DMABUFs into MPP" << std::endl;
        }

        void queue_all_buffers() 
        {
            for (uint32_t index = 0; index < buffers_.size(); ++index) 
            {
                requeue(index);
            }
        }
        int fd_ = -1;
        const v4l2_buf_type type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        bool streaming_ = false;
        uint32_t requested_width_ = 0;
        uint32_t requested_height_ = 0;
        uint32_t storage_height_ = 0;
        uint32_t horizontal_stride_ = 0;
        uint32_t vertical_stride_ = 0;
        uint32_t size_image_ = 0;
        std::vector<CaptureBuffer> buffers_;
    };

    class MppVideoEncoder 
    {
    public:
        MppVideoEncoder(VideoCodec codec,
                        uint32_t width,
                        uint32_t height,
                        uint32_t horizontal_stride,
                        uint32_t vertical_stride,
                        uint32_t fps,
                        uint32_t bitrate)
            : width_(width),
            height_(height),
            horizontal_stride_(horizontal_stride),
            vertical_stride_(vertical_stride) 
        {
            require(mpp_create(&context_, &api_) == MPP_OK && context_ && api_,"mpp_create failed");

            // MPP 1.5 declares the timeout control payload as RK_S64.  Do not
            // pass MppPollType here: that enum is normally 32-bit and the control
            // API reads an RK_S64 through the untyped pointer.
            RK_S64 block = MPP_POLL_BLOCK;
            require(api_->control(context_, MPP_SET_INPUT_TIMEOUT, &block) == MPP_OK,"MPP_SET_INPUT_TIMEOUT failed");
            require(api_->control(context_, MPP_SET_OUTPUT_TIMEOUT, &block) == MPP_OK,"MPP_SET_OUTPUT_TIMEOUT failed");
            const MppCodingType coding = mpp_coding_type(codec);
            require(mpp_init(context_, MPP_CTX_ENC, coding) == MPP_OK,std::string("mpp_init ") + codec_name(codec) + " encoder failed");
            require(mpp_enc_cfg_init(&config_) == MPP_OK && config_,"mpp_enc_cfg_init failed");
            require(api_->control(context_, MPP_ENC_GET_CFG, config_) == MPP_OK,"MPP_ENC_GET_CFG failed");

            set_s32("prep:width", width);
            set_s32("prep:height", height);
            set_s32("prep:hor_stride", horizontal_stride);
            set_s32("prep:ver_stride", vertical_stride);
            set_s32("prep:format", MPP_FMT_YUV420SP);
            set_s32("codec:type", coding);

            set_s32("rc:mode", MPP_ENC_RC_MODE_CBR);
            set_s32("rc:bps_target", bitrate);
            set_s32("rc:bps_max", bitrate * 17 / 16);
            set_s32("rc:bps_min", bitrate * 15 / 16);
            set_s32("rc:fps_in_flex", 0);
            set_s32("rc:fps_in_num", fps);
            set_s32("rc:fps_in_denorm", 1);
            set_s32("rc:fps_out_flex", 0);
            set_s32("rc:fps_out_num", fps);
            set_s32("rc:fps_out_denorm", 1);
            set_s32("rc:gop", fps);
            set_u32("rc:max_reenc_times", 0);

            const std::string codec_prefix =
                codec == VideoCodec::H264 ? "h264:" : "h265:";
            set_s32((codec_prefix + "qp_init").c_str(), 26);
            set_s32((codec_prefix + "qp_max").c_str(), 51);
            set_s32((codec_prefix + "qp_min").c_str(), 10);
            set_s32((codec_prefix + "qp_max_i").c_str(), 46);
            set_s32((codec_prefix + "qp_min_i").c_str(), 18);

            require(api_->control(context_, MPP_ENC_SET_CFG, config_) == MPP_OK,
                    "MPP_ENC_SET_CFG failed");

            MppEncHeaderMode header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
            require(api_->control(context_, MPP_ENC_SET_HEADER_MODE, &header_mode) ==
                        MPP_OK,
                    "MPP_ENC_SET_HEADER_MODE failed");
        }

        MppVideoEncoder(const MppVideoEncoder&) = delete;
        MppVideoEncoder& operator=(const MppVideoEncoder&) = delete;

        ~MppVideoEncoder() 
        {
            if (config_) 
            {
                mpp_enc_cfg_deinit(config_);
                config_ = nullptr;
            }
            if (context_) 
            {
                api_->reset(context_);
                mpp_destroy(context_);
                context_ = nullptr;
                api_ = nullptr;
            }
        }

        std::vector<uint8_t> encode(MppBuffer input, int64_t pts_us) 
        {
            MppFrame frame = nullptr;
            require(mpp_frame_init(&frame) == MPP_OK && frame,
                    "mpp_frame_init failed");

            mpp_frame_set_width(frame, width_);
            mpp_frame_set_height(frame, height_);
            mpp_frame_set_hor_stride(frame, horizontal_stride_);
            mpp_frame_set_ver_stride(frame, vertical_stride_);
            mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);
            mpp_frame_set_pts(frame, pts_us);
            mpp_frame_set_eos(frame, 0);
            mpp_frame_set_buffer(frame, input);

            const MPP_RET put_result = api_->encode_put_frame(context_, frame);
            mpp_frame_deinit(&frame);
            require(put_result == MPP_OK, "encode_put_frame failed");

            MppPacket packet = nullptr;
            require(api_->encode_get_packet(context_, &packet) == MPP_OK && packet,"encode_get_packet failed");

            const auto* data = static_cast<const uint8_t*>(mpp_packet_get_pos(packet));
            const size_t size = mpp_packet_get_length(packet);
            std::vector<uint8_t> result(data, data + size);
            mpp_packet_deinit(&packet);
            return result;
        }

    private:
        void set_s32(const char* key, int32_t value) 
        {
            require(mpp_enc_cfg_set_s32(config_, key, value) == MPP_OK, std::string("failed to set MPP config ") + key);
        }

        void set_u32(const char* key, uint32_t value) 
        {
            require(mpp_enc_cfg_set_u32(config_, key, value) == MPP_OK,std::string("failed to set MPP config ") + key);
        }

        MppCtx context_ = nullptr;
        MppApi* api_ = nullptr;
        MppEncCfg config_ = nullptr;
        uint32_t width_ = 0;
        uint32_t height_ = 0;
        uint32_t horizontal_stride_ = 0;
        uint32_t vertical_stride_ = 0;
    };

    class Mp4Muxer 
    {
    public:
        Mp4Muxer(VideoCodec codec,
                const std::string& output,
                uint32_t width,
                uint32_t height,
                uint32_t fps)
            : fps_(fps) {
            pipeline_ = gst_pipeline_new("video-mp4-muxer");
            source_ = gst_element_factory_make("appsrc", "encoded-source");
            GstElement* parser = gst_element_factory_make(gst_parser_name(codec), "parser");
            GstElement* muxer = gst_element_factory_make("qtmux", "muxer");
            GstElement* sink = gst_element_factory_make("filesink", "sink");
            require(pipeline_ && source_ && parser && muxer && sink,
                    "create appsrc/parser/qtmux/filesink failed");

            g_object_set(source_,
                        "is-live", TRUE,
                        "format", GST_FORMAT_TIME,
                        "block", TRUE,
                        nullptr);
            g_object_set(parser, "config-interval", -1, nullptr);
            g_object_set(sink, "location", output.c_str(), nullptr);

            GstCaps* caps = gst_caps_new_simple(
                gst_media_type(codec),
                "stream-format", G_TYPE_STRING, "byte-stream",
                "alignment", G_TYPE_STRING, "au",
                "width", G_TYPE_INT, static_cast<int>(width),
                "height", G_TYPE_INT, static_cast<int>(height),
                "framerate", GST_TYPE_FRACTION, static_cast<int>(fps), 1,
                nullptr);
            gst_app_src_set_caps(GST_APP_SRC(source_), caps);
            gst_caps_unref(caps);
            gst_bin_add_many(GST_BIN(pipeline_), source_, parser, muxer, sink, nullptr);
            require(gst_element_link_many(source_, parser, muxer, sink, nullptr),"link appsrc ! parser ! qtmux ! filesink failed");
            require(gst_element_set_state(pipeline_, GST_STATE_PLAYING) != GST_STATE_CHANGE_FAILURE,"start MP4 muxing pipeline failed");
        }

        Mp4Muxer(const Mp4Muxer&) = delete;
        Mp4Muxer& operator=(const Mp4Muxer&) = delete;
        ~Mp4Muxer() {
            if (pipeline_) 
            {
                gst_element_set_state(pipeline_, GST_STATE_NULL);
                gst_object_unref(pipeline_);
                pipeline_ = nullptr;
                source_ = nullptr;
            }
        }

        void push(const std::vector<uint8_t>& packet, uint64_t timeline_frame_number) 
        {
            GstBuffer* buffer = gst_buffer_new_allocate(nullptr, packet.size(), nullptr);
            require(buffer != nullptr, "allocate encoded GstBuffer failed");
            require(gst_buffer_fill(buffer, 0, packet.data(), packet.size()) ==
                        packet.size(),
                    "fill encoded GstBuffer failed");

            GST_BUFFER_PTS(buffer) =gst_util_uint64_scale(timeline_frame_number, GST_SECOND, fps_);
            GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);
            GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(1, GST_SECOND, fps_);
            const GstFlowReturn flow = gst_app_src_push_buffer(GST_APP_SRC(source_), buffer);
            require(flow == GST_FLOW_OK,"appsrc push failed: " + std::to_string(flow));
        }

        void finish() {
            if (finished_) 
            {
                return;
            }
            finished_ = true;

            gst_app_src_end_of_stream(GST_APP_SRC(source_));
            GstBus* bus = gst_element_get_bus(pipeline_);
            GstMessage* message = gst_bus_timed_pop_filtered(bus,10 * GST_SECOND,static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
            if (!message) 
            {
                gst_object_unref(bus);
                throw std::runtime_error("timeout waiting for MP4 EOS");
            }

            if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) 
            {
                GError* error = nullptr;
                gchar* debug = nullptr;
                gst_message_parse_error(message, &error, &debug);
                std::string text = error ? error->message : "unknown GStreamer error";
                if (debug) 
                {
                    text += "; ";
                    text += debug;
                }
                if (error) g_error_free(error);
                if (debug) g_free(debug);
                gst_message_unref(message);
                gst_object_unref(bus);
                throw std::runtime_error(text);
            }

            gst_message_unref(message);
            gst_object_unref(bus);
        }

    private:
        GstElement* pipeline_ = nullptr;
        GstElement* source_ = nullptr;
        uint32_t fps_ = 0;
        bool finished_ = false;
    };

    struct CameraSpec 
    {
        std::string device;
        std::string output;
    };
    //默认值
    struct ProgramOptions 
    {
        uint32_t width = 4000;
        uint32_t height = 3000;
        uint32_t fps = 30;
        uint32_t duration_seconds = 300;
        uint32_t bitrate = 20000000;
        uint32_t buffer_count = 6;
        VideoCodec codec = VideoCodec::H265;
        std::vector<CameraSpec> cameras;
    };

    uint32_t parse_positive_u32(const std::string& text, const char* name) 
    {
        size_t consumed = 0;
        const unsigned long value = std::stoul(text, &consumed);
        require(consumed == text.size(), std::string("invalid ") + name + ": " + text);
        require(value > 0 && value <= std::numeric_limits<uint32_t>::max(),std::string(name) + " is out of range: " + text);
        return static_cast<uint32_t>(value);
    }

    void validate_camera_specs(const std::vector<CameraSpec>& cameras) 
    {
        require(!cameras.empty(), "at least one --camera DEVICE OUTPUT is required");

        std::unordered_set<std::string> devices;
        std::unordered_set<std::string> outputs;
        for (const CameraSpec& camera : cameras) 
        {
            require(devices.insert(camera.device).second,
                    "duplicate camera device: " + camera.device);
            require(outputs.insert(camera.output).second,
                    "duplicate output file: " + camera.output);
        }
    }

    ProgramOptions parse_arguments(int argc, char** argv) 
    {
        ProgramOptions options;

        if (argc == 1) 
        {
            options.cameras.push_back({"/dev/video22", "camera22.mp4"});

            options.cameras.push_back({"/dev/video31", "camera31.mp4"});  
            return options;

        }

        // Preserve the original single-camera positional command line.
        if (argv[1][0] != '-') 
        {
            require(argc <= 9, "too many positional arguments; use --help for usage");

            CameraSpec camera{argv[1], "zero-copy.mp4"};
            if (argc >= 3) options.width = parse_positive_u32(argv[2], "width");
            if (argc >= 4) options.height = parse_positive_u32(argv[3], "height");
            if (argc >= 5) options.fps = parse_positive_u32(argv[4], "fps");
            if (argc >= 6) 
            {
                options.duration_seconds =
                    parse_positive_u32(argv[5], "duration_sec");
            }
            if (argc >= 7) camera.output = argv[6];
            if (argc >= 8) 
            {
                options.bitrate = parse_positive_u32(argv[7], "bitrate_bps");
            }
            if (argc >= 9) options.codec = parse_codec(argv[8]);
            options.cameras.push_back(std::move(camera));
            return options;
        }

        for (int index = 1; index < argc; ++index) 
        {
            const std::string argument = argv[index];
            auto next_value = [&](const char* option) -> std::string 
            {
                require(index + 1 < argc,
                        std::string("missing value after ") + option);
                return argv[++index];
            };

            if (argument == "--camera") 
            {
                const std::string device = next_value("--camera");
                const std::string output = next_value("--camera DEVICE");
                options.cameras.push_back({device, output});
            } 
            else if (argument == "--width") 
            {
                options.width = parse_positive_u32(next_value("--width"), "width");
            } 
            else if (argument == "--height") 
            {
                options.height = parse_positive_u32(next_value("--height"), "height");
            } 
            else if (argument == "--fps") 
            {
                options.fps = parse_positive_u32(next_value("--fps"), "fps");
            } 
            else if (argument == "--duration")
            {
                options.duration_seconds =
                    parse_positive_u32(next_value("--duration"), "duration");
            } 
            else if (argument == "--bitrate") 
            {
                options.bitrate =
                    parse_positive_u32(next_value("--bitrate"), "bitrate");
            } 
            else if (argument == "--buffers") 
            {
                options.buffer_count =
                    parse_positive_u32(next_value("--buffers"), "buffers");
            } 
            else if (argument == "--codec") 
            {
                options.codec = parse_codec(next_value("--codec"));
            } 
            else 
            {
                throw std::runtime_error("unknown option: " + argument);
            }
        }
        validate_camera_specs(options.cameras);
        require(options.buffer_count >= 3, "--buffers must be at least 3");
        return options;
    }

    class CameraSession 
    {
    public:
        CameraSession(size_t index,
                    CameraSpec camera,
                    const ProgramOptions& options)
            : index_(index),
            camera_(std::move(camera)),
            width_(options.width),
            height_(options.height),
            fps_(options.fps),
            duration_seconds_(options.duration_seconds),
            bitrate_(options.bitrate),
            codec_(options.codec) 
            {
                //创建采集器 创建一个由 智能指针capture_ 独占管理的 V4L2Capture 对象  函数销毁 智能指针自动销毁
                capture_ = std::make_unique<V4L2Capture>(
                    camera_.device,
                    width_,
                    height_,
                    fps_,
                    options.buffer_count);
                //创建编码器 
                encoder_ = std::make_unique<MppVideoEncoder>(
                    codec_,
                    capture_->width(),
                    capture_->height(),
                    capture_->horizontal_stride(),
                    capture_->vertical_stride(),
                    fps_,
                    bitrate_);
                //创建MP4封装器
                muxer_ = std::make_unique<Mp4Muxer>(
                    codec_, 
                    camera_.output,
                    width_, height_, 
                    fps_);
        }
        
        CameraSession(const CameraSession&) = delete;
        CameraSession& operator=(const CameraSession&) = delete;

        void run() 
        {
            //日志锁
            {
                std::lock_guard<std::mutex> lock(output_mutex);
                std::cout << prefix()
                        << "codec=" << codec_name(codec_)
                        << " bitrate=" << bitrate_ << " bps"
                        << " target_fps=" << fps_
                        << " real_duration=" << duration_seconds_ << " sec"
                        << " output=" << camera_.output << std::endl;
            }
            //只锁 保护打印 
            //缓冲已准备 开始采集
            capture_->start();
            
            const uint64_t target_frame_count =
                static_cast<uint64_t>(fps_) * duration_seconds_;
            const int64_t target_duration_ns =
                static_cast<int64_t>(duration_seconds_) * 1000000000LL;

            uint64_t captured_count = 0;
            uint64_t encoded_count = 0;
            uint64_t dropped_count = 0;
            uint64_t next_timeline_frame = 0;
            int64_t first_capture_timestamp_ns = 0;
            int64_t last_capture_elapsed_ns = 0;
            bool timeline_started = false;
            auto recording_wall_start = std::chrono::steady_clock::now();

            while (should_run()) 
            {
                CapturedFrame captured;
                if (!capture_->dequeue(captured))
                {
                    break;
                }
                //成功处理并写入视频流程的帧数
                ++captured_count;

                if (!timeline_started) 
                {
                    require(captured.timestamp_ns > 0,"V4L2 did not provide a valid capture timestamp");
                    timeline_started = true;
                    first_capture_timestamp_ns = captured.timestamp_ns; //摄像头/V4L2时间
                    recording_wall_start = std::chrono::steady_clock::now(); //RK3588主机时间

                }

                require(captured.timestamp_ns >= first_capture_timestamp_ns,"V4L2 capture timestamp moved backwards");
                const int64_t capture_elapsed_ns = captured.timestamp_ns - first_capture_timestamp_ns;
                last_capture_elapsed_ns = capture_elapsed_ns;
                if (capture_elapsed_ns >= target_duration_ns) 
                {
                    capture_->requeue(captured.index);
                    break;
                }

                const uint64_t capture_timeline_frame = gst_util_uint64_scale(static_cast<uint64_t>(capture_elapsed_ns), fps_, GST_SECOND);

                // Preserve a real source/processing gap instead of encoding a
                // burst of stale queued frames after this camera falls behind.
                if (capture_timeline_frame > next_timeline_frame) 
                {
                    next_timeline_frame = capture_timeline_frame;
                }
                if (next_timeline_frame >= target_frame_count) 
                {
                    capture_->requeue(captured.index);
                    break;
                }

                const uint64_t next_frame_due_ns = gst_util_uint64_scale(next_timeline_frame, GST_SECOND, fps_);
                constexpr uint64_t timestamp_tolerance_ns = 1000000ULL;
                if (static_cast<uint64_t>(capture_elapsed_ns) + timestamp_tolerance_ns < next_frame_due_ns) 
                {
                    capture_->requeue(captured.index);
                    ++dropped_count;
                    continue;
                }
                const uint64_t timeline_frame = next_timeline_frame;
                const int64_t pts_us = static_cast<int64_t>(timeline_frame * 1000000ULL / fps_);
                try 
                {
                    capture_->prepare_for_mpp(captured.index);
                    std::vector<uint8_t> packet = encoder_->encode(capture_->mpp_buffer(captured.index), pts_us);
                    muxer_->push(packet, timeline_frame);
                    capture_->requeue(captured.index);
                } 
                catch (...) 
                {
                    capture_->requeue(captured.index);
                    throw;
                }

                next_timeline_frame = timeline_frame + 1;
                ++encoded_count;
                if (encoded_count % fps_ == 0) 
                {
                    std::lock_guard<std::mutex> lock(output_mutex);
                    std::cout << prefix()
                            << "captured=" << captured_count
                            << " encoded=" << encoded_count
                            << " dropped=" << dropped_count << std::endl;
                }
            }

            const auto recording_wall_end = std::chrono::steady_clock::now();
            capture_->stop();
            const auto finalize_start = std::chrono::steady_clock::now();
            muxer_->finish();
            const auto finalize_end = std::chrono::steady_clock::now();
            const double recording_wall_seconds = std::chrono::duration<double>(recording_wall_end -recording_wall_start).count();
            const double capture_timeline_seconds = static_cast<double>(last_capture_elapsed_ns) / 1000000000.0;
            const double finalize_seconds = std::chrono::duration<double>(finalize_end - finalize_start).count();
            std::lock_guard<std::mutex> lock(output_mutex);
            std::cout << prefix()
                    << "captured=" << captured_count
                    << " encoded=" << encoded_count
                    << " dropped=" << dropped_count
                    << " capture_timeline=" << capture_timeline_seconds << " sec"
                    << " recording_wall=" << recording_wall_seconds << " sec"
                    << " finalize=" << finalize_seconds << " sec"
                    << " output_fps=" << fps_
                    << " saved=" << camera_.output << std::endl;
        }
        std::string prefix() const 
        {
            std::ostringstream stream;
            stream << "[camera " << index_ << ' ' << camera_.device << "] ";
            return stream.str();
        }

    private:
        size_t index_ = 0;
        CameraSpec camera_;
        uint32_t width_ = 0;
        uint32_t height_ = 0;
        uint32_t fps_ = 0;
        uint32_t duration_seconds_ = 0;
        uint32_t bitrate_ = 0;
        VideoCodec codec_ = VideoCodec::H265;
        std::unique_ptr<V4L2Capture> capture_;
        std::unique_ptr<MppVideoEncoder> encoder_;
        std::unique_ptr<Mp4Muxer> muxer_;
    };

    void print_usage(const char* program) 
    {
        std::cout
            << "usage:\n"
            << "  " << program
            << " [device] [width] [height] [fps] [duration_sec]"
            " [output.mp4] [bitrate_bps] [codec]\n\n"
            << "  " << program
            << " --camera DEVICE OUTPUT [--camera DEVICE OUTPUT ...]"
            " [--width N] [--height N] [--fps N] [--duration N]"
            " [--bitrate N] [--codec h264|h265] [--buffers N]\n\n"
            << "examples:\n"
            << "  " << program
            << " /dev/video31 4000 3000 30 60 camera0.mp4 20000000 h265\n\n"
            << "  " << program
            << " --camera /dev/video31 camera0.mp4"
            " --camera /dev/video41 camera1.mp4"
            " --width 4000 --height 3000 --fps 30 --duration 60"
            " --bitrate 20000000 --codec h265\n";
    }

}  
// namespace

/**
 *   V4L2确定图像布局
 *   MPP按照该布局初始化
 *   GStreamer准备接收编码数据
 */
int main(int argc, char** argv) 
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    try {
            if (argc == 2 &&
                (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) 
                {
                    print_usage(argv[0]);
                    return 0;
                }

        ProgramOptions options = parse_arguments(argc, argv);
        validate_camera_specs(options.cameras);
        require(options.buffer_count >= 3, "buffer count must be at least 3");

        // Application options were already parsed above. Avoid asking
        // GStreamer to interpret repeated --camera arguments.
        gst_init(nullptr, nullptr);
        // 入口
        std::vector<std::unique_ptr<CameraSession>> sessions;
        sessions.reserve(options.cameras.size());
        for (size_t index = 0; index < options.cameras.size(); ++index) 
        {
            {
                std::lock_guard<std::mutex> lock(output_mutex);
                std::cout << "preparing camera " << index << ": "
                          << options.cameras[index].device << " -> "
                          << options.cameras[index].output << std::endl;
            }
            sessions.push_back(std::make_unique<CameraSession>(index, options.cameras[index], options));
        }

        std::mutex start_mutex;
        std::condition_variable start_condition;
        bool start_sessions = false;
        std::vector<std::exception_ptr> errors(sessions.size());
        std::vector<std::thread> workers;
        workers.reserve(sessions.size());
        for (size_t index = 0; index < sessions.size(); ++index) 
        {
            workers.emplace_back([&, index] 
            {
                {
                    std::unique_lock<std::mutex> lock(start_mutex);
                    start_condition.wait(lock, [&] { return start_sessions; });
                }
                try 
                {
                    sessions[index]->run();
                } 
                catch (...) 
                {
                    errors[index] = std::current_exception();
                    stop_requested.store(true, std::memory_order_relaxed);
                }
            });
        }

        {
            std::lock_guard<std::mutex> lock(start_mutex);
            start_sessions = true;
        }
        start_condition.notify_all();

        for (std::thread& worker : workers) 
        {
            worker.join();
        }

        bool failed = false;
        for (size_t index = 0; index < errors.size(); ++index)
        {
            if (!errors[index]) 
            {
                continue;
            }
            failed = true;
            try 
            {
                std::rethrow_exception(errors[index]);
            } 
            catch (const std::exception& error) 
            {
                std::cerr << sessions[index]->prefix()
                          << "error: " << error.what() << std::endl;
            } 
            catch (...)
            {
                std::cerr << sessions[index]->prefix()
                          << "error: unknown exception" << std::endl;
            }
        }

        return failed ? 1 : 0;
    } 
    catch (const std::exception& error) 
    {
        std::cerr << "error: " << error.what() << std::endl;
        return 1;
    }
}
