#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

struct PlaneBuffer {
    void* start = nullptr;
    size_t length = 0;
};

struct Buffer {
    std::vector<PlaneBuffer> planes;
};

static int xioctl(int fd, unsigned long request, void* arg) {
    int ret;
    do {
        ret = ioctl(fd, request, arg);
    } while (ret == -1 && errno == EINTR);
    return ret;
}

static std::string fourcc_to_string(uint32_t fmt) {
    std::string s;
    s.push_back(static_cast<char>(fmt & 0xff));
    s.push_back(static_cast<char>((fmt >> 8) & 0xff));
    s.push_back(static_cast<char>((fmt >> 16) & 0xff));
    s.push_back(static_cast<char>((fmt >> 24) & 0xff));
    return s;
}

static uint32_t pixel_format_from_string(const std::string& fmt) {
    if (fmt == "RG10") return V4L2_PIX_FMT_SRGGB10;
    if (fmt == "GR10") return V4L2_PIX_FMT_SGRBG10;
    if (fmt == "GB10") return V4L2_PIX_FMT_SGBRG10;
    if (fmt == "BA10" || fmt == "BG10") return V4L2_PIX_FMT_SBGGR10;
    if (fmt == "Y10" || fmt == "Y10 ") return V4L2_PIX_FMT_Y10;
    if (fmt == "NV12") return V4L2_PIX_FMT_NV12;
    if (fmt == "NV21") return V4L2_PIX_FMT_NV21;
    if (fmt == "NV12M") return V4L2_PIX_FMT_NV12M;
    if (fmt == "NV21M") return V4L2_PIX_FMT_NV21M;
    if (fmt == "YUV420M") return V4L2_PIX_FMT_YUV420M;
    if (fmt == "YVU420M") return V4L2_PIX_FMT_YVU420M;

    throw std::runtime_error("unsupported format: " + fmt);
}

static void cleanup_buffers(std::vector<Buffer>& buffers) {
    for (auto& buffer : buffers) {
        for (auto& plane : buffer.planes) {
            if (plane.start && plane.start != MAP_FAILED) {
                munmap(plane.start, plane.length);
                plane.start = nullptr;
                plane.length = 0;
            }
        }
    }
}

static void save_plane(const void* data,
                       size_t size,
                       int frame_index,
                       uint32_t plane_index,
                       const std::string& fmt_name) {
    const std::string filename =
        "frame_" + std::to_string(frame_index) +
        ".plane_" + std::to_string(plane_index) +
        "." + fmt_name + ".raw";

    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs) {
        throw std::runtime_error("failed to open output file: " + filename);
    }

    ofs.write(static_cast<const char*>(data), static_cast<std::streamsize>(size));
    if (!ofs) {
        throw std::runtime_error("failed to write output file: " + filename);
    }

    std::cout << "saved " << filename << ", bytes=" << size << std::endl;
}

static void save_metadata(int frame_index,
                          const std::string& fmt_name,
                          const v4l2_format& fmt,
                          const v4l2_buffer& buf,
                          const v4l2_plane* planes) {
    const std::string filename =
        "frame_" + std::to_string(frame_index) + "." + fmt_name + ".txt";

    std::ofstream ofs(filename);
    if (!ofs) {
        throw std::runtime_error("failed to open metadata file: " + filename);
    }

    ofs << "width=" << fmt.fmt.pix_mp.width << "\n";
    ofs << "height=" << fmt.fmt.pix_mp.height << "\n";
    ofs << "requested_format=" << fmt_name << "\n";
    ofs << "actual_fourcc=" << fourcc_to_string(fmt.fmt.pix_mp.pixelformat) << "\n";
    ofs << "num_planes=" << fmt.fmt.pix_mp.num_planes << "\n";
    ofs << "sequence=" << buf.sequence << "\n";
    ofs << "timestamp_sec=" << buf.timestamp.tv_sec << "\n";
    ofs << "timestamp_usec=" << buf.timestamp.tv_usec << "\n";

    for (uint32_t p = 0; p < fmt.fmt.pix_mp.num_planes; ++p) {
        ofs << "plane[" << p << "].bytesperline="
            << fmt.fmt.pix_mp.plane_fmt[p].bytesperline << "\n";
        ofs << "plane[" << p << "].sizeimage="
            << fmt.fmt.pix_mp.plane_fmt[p].sizeimage << "\n";
        ofs << "plane[" << p << "].bytesused="
            << planes[p].bytesused << "\n";
        ofs << "plane[" << p << "].data_offset="
            << planes[p].data_offset << "\n";
    }
}

static void print_usage(const char* program) {
    std::cout << "usage:\n"
              << "  " << program << " [device] [width] [height] [format] [frame_count]\n\n"
              << "example:\n"
              << "  " << program << " /dev/video0 1920 1080 NV12M 5\n\n"
              << "formats:\n"
              << "  RG10 GR10 GB10 BA10 BG10 Y10 NV12 NV21 NV12M NV21M YUV420M YVU420M\n";
}

int main(int argc, char** argv) {
    std::string device = "/dev/video0";
    int width = 4000;
    int height = 3000;
    std::string fmt_name = "RG10";
    int frame_count = 5;

    if (argc >= 2) {
        const std::string arg = argv[1];
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
        device = arg;
    }
    if (argc >= 3) width = std::stoi(argv[2]);
    if (argc >= 4) height = std::stoi(argv[3]);
    if (argc >= 5) fmt_name = argv[4];
    if (argc >= 6) frame_count = std::stoi(argv[5]);

    if (width <= 0 || height <= 0 || frame_count <= 0) {
        std::cerr << "invalid argument" << std::endl;
        return 1;
    }

    const uint32_t requested_pixfmt = pixel_format_from_string(fmt_name);

    std::cout << "device      : " << device << std::endl;
    std::cout << "resolution  : " << width << "x" << height << std::endl;
    std::cout << "format      : " << fmt_name << std::endl;
    std::cout << "frame_count : " << frame_count << std::endl;

    int fd = open(device.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd < 0) {
        std::cerr << "open failed: " << strerror(errno) << std::endl;
        return 1;
    }

    std::vector<Buffer> buffers;
    bool streaming = false;
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    try {
        v4l2_capability cap {};
        if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
            throw std::runtime_error("VIDIOC_QUERYCAP failed: " + std::string(strerror(errno)));
        }

        std::cout << "driver      : " << cap.driver << std::endl;
        std::cout << "card        : " << cap.card << std::endl;
        std::cout << "bus_info    : " << cap.bus_info << std::endl;
        std::cout << "capabilities: 0x" << std::hex << cap.capabilities << std::dec << std::endl;
        std::cout << "device_caps : 0x" << std::hex << cap.device_caps << std::dec << std::endl;

        uint32_t caps = cap.capabilities;
        if (caps & V4L2_CAP_DEVICE_CAPS) {
            caps = cap.device_caps;
        }

        if (!(caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE)) {
            throw std::runtime_error("device does not support VIDEO_CAPTURE_MPLANE");
        }
        if (!(caps & V4L2_CAP_STREAMING)) {
            throw std::runtime_error("device does not support STREAMING");
        }

        v4l2_format fmt {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.width = static_cast<uint32_t>(width);
        fmt.fmt.pix_mp.height = static_cast<uint32_t>(height);
        fmt.fmt.pix_mp.pixelformat = requested_pixfmt;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;

        if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            throw std::runtime_error("VIDIOC_S_FMT failed: " + std::string(strerror(errno)));
        }

        const uint32_t num_planes = fmt.fmt.pix_mp.num_planes;
        if (num_planes == 0 || num_planes > VIDEO_MAX_PLANES) {
            throw std::runtime_error("invalid num_planes");
        }

        std::cout << "actual fmt  : "
                  << fmt.fmt.pix_mp.width << "x" << fmt.fmt.pix_mp.height
                  << " " << fourcc_to_string(fmt.fmt.pix_mp.pixelformat)
                  << " num_planes=" << num_planes
                  << std::endl;

        if (fmt.fmt.pix_mp.pixelformat != requested_pixfmt) {
            std::cerr << "warning: driver changed requested format "
                      << fourcc_to_string(requested_pixfmt)
                      << " to "
                      << fourcc_to_string(fmt.fmt.pix_mp.pixelformat)
                      << std::endl;
        }

        for (uint32_t p = 0; p < num_planes; ++p) {
            std::cout << "plane[" << p << "]"
                      << " bytesperline=" << fmt.fmt.pix_mp.plane_fmt[p].bytesperline
                      << " sizeimage=" << fmt.fmt.pix_mp.plane_fmt[p].sizeimage
                      << std::endl;
        }

        v4l2_requestbuffers req {};
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        req.memory = V4L2_MEMORY_MMAP;

        if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
            throw std::runtime_error("VIDIOC_REQBUFS failed: " + std::string(strerror(errno)));
        }
        if (req.count < 2) {
            throw std::runtime_error("insufficient buffer memory");
        }

        buffers.resize(req.count);

        for (uint32_t i = 0; i < req.count; ++i) {
            v4l2_buffer buf {};
            v4l2_plane planes[VIDEO_MAX_PLANES] {};

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            buf.length = num_planes;
            buf.m.planes = planes;

            if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
                throw std::runtime_error("VIDIOC_QUERYBUF failed: " + std::string(strerror(errno)));
            }

            buffers[i].planes.resize(num_planes);

            for (uint32_t p = 0; p < num_planes; ++p) {
                buffers[i].planes[p].length = buf.m.planes[p].length;
                buffers[i].planes[p].start = mmap(nullptr,
                                                  buf.m.planes[p].length,
                                                  PROT_READ | PROT_WRITE,
                                                  MAP_SHARED,
                                                  fd,
                                                  buf.m.planes[p].m.mem_offset);

                if (buffers[i].planes[p].start == MAP_FAILED) {
                    buffers[i].planes[p].start = nullptr;
                    buffers[i].planes[p].length = 0;
                    throw std::runtime_error("mmap failed: " + std::string(strerror(errno)));
                }

                std::cout << "buffer[" << i << "].plane[" << p << "]"
                          << " length=" << buf.m.planes[p].length
                          << " offset=" << buf.m.planes[p].m.mem_offset
                          << std::endl;
            }
        }

        for (uint32_t i = 0; i < buffers.size(); ++i) {
            v4l2_buffer buf {};
            v4l2_plane planes[VIDEO_MAX_PLANES] {};

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            buf.length = num_planes;
            buf.m.planes = planes;

            if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
                throw std::runtime_error("VIDIOC_QBUF failed: " + std::string(strerror(errno)));
            }
        }

        if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            throw std::runtime_error("VIDIOC_STREAMON failed: " + std::string(strerror(errno)));
        }
        streaming = true;
        std::cout << "stream on" << std::endl;

        int captured = 0;
        while (captured < frame_count) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(fd, &fds);

            timeval tv {};
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            const int ret = select(fd + 1, &fds, nullptr, nullptr, &tv);
            if (ret < 0) {
                if (errno == EINTR) continue;
                throw std::runtime_error("select failed: " + std::string(strerror(errno)));
            }
            if (ret == 0) {
                throw std::runtime_error("select timeout");
            }

            v4l2_buffer buf {};
            v4l2_plane planes[VIDEO_MAX_PLANES] {};

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.length = num_planes;
            buf.m.planes = planes;

            if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
                if (errno == EAGAIN) continue;
                throw std::runtime_error("VIDIOC_DQBUF failed: " + std::string(strerror(errno)));
            }

            if (buf.index >= buffers.size()) {
                throw std::runtime_error("VIDIOC_DQBUF returned invalid buffer index");
            }

            std::cout << "frame " << captured
                      << " index=" << buf.index
                      << " sequence=" << buf.sequence
                      << " timestamp=" << buf.timestamp.tv_sec
                      << "." << buf.timestamp.tv_usec
                      << std::endl;

            for (uint32_t p = 0; p < num_planes; ++p) {
                const size_t bytesused = planes[p].bytesused;
                const size_t data_offset = planes[p].data_offset;
                const size_t mapped_length = buffers[buf.index].planes[p].length;

                if (data_offset > mapped_length) {
                    throw std::runtime_error("plane data_offset exceeds mapped length");
                }
                if (bytesused > mapped_length - data_offset) {
                    throw std::runtime_error("plane bytesused exceeds mapped length");
                }

                const char* plane_data =
                    static_cast<const char*>(buffers[buf.index].planes[p].start) + data_offset;

                std::cout << "  plane[" << p << "]"
                          << " bytesused=" << bytesused
                          << " data_offset=" << data_offset
                          << " mapped_length=" << mapped_length
                          << std::endl;

                save_plane(plane_data, bytesused, captured, p, fmt_name);
            }

            save_metadata(captured, fmt_name, fmt, buf, planes);

            if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
                throw std::runtime_error("VIDIOC_QBUF requeue failed: " + std::string(strerror(errno)));
            }

            ++captured;
        }

        if (xioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
            throw std::runtime_error("VIDIOC_STREAMOFF failed: " + std::string(strerror(errno)));
        }
        streaming = false;
        std::cout << "stream off" << std::endl;

        cleanup_buffers(buffers);
        close(fd);
        return 0;
    } 
    catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << std::endl;

        if (streaming) {
            xioctl(fd, VIDIOC_STREAMOFF, &type);
        }

        cleanup_buffers(buffers);
        close(fd);
        return 1;
    }
}