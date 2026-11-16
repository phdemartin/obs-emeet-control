#pragma once

#include <dirent.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace emeet {

struct V4L2DeviceInfo {
    std::string path;
    std::string name;
    bool is_emeet = false;
};

inline std::string read_sysfs_trim(const std::string &path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open())
        return {};
    std::string val;
    std::getline(ifs, val);
    while (!val.empty() && (val.back() == '\n' || val.back() == '\r' || val.back() == ' '))
        val.pop_back();
    return val;
}

inline std::vector<V4L2DeviceInfo> list_v4l2_devices()
{
    std::vector<V4L2DeviceInfo> devices;
    DIR *dir = opendir("/dev");
    if (!dir)
        return devices;

    std::vector<std::string> nodes;
    while (struct dirent *ent = readdir(dir)) {
        std::string name = ent->d_name;
        if (name.rfind("video", 0) == 0)
            nodes.push_back("/dev/" + name);
    }
    closedir(dir);
    std::sort(nodes.begin(), nodes.end());

    for (const auto &dev_path : nodes) {
        int fd = open(dev_path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0)
            continue;

        struct v4l2_capability cap {};
        int rc = ioctl(fd, VIDIOC_QUERYCAP, &cap);
        close(fd);
        if (rc < 0)
            continue;

        uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
        if (!(caps & V4L2_CAP_VIDEO_CAPTURE))
            continue;

        std::string driver = reinterpret_cast<const char *>(cap.driver);
        std::string card = reinterpret_cast<const char *>(cap.card);
        if (driver.find("v4l2loopback") != std::string::npos)
            continue;

        bool is_emeet = (card.find("EMEET") != std::string::npos || card.find("emeet") != std::string::npos);
        std::string dev_name = dev_path.substr(5); // videoN
        std::string vendor = read_sysfs_trim("/sys/class/video4linux/" + dev_name + "/device/../idVendor");
        if (vendor == "328f")
            is_emeet = true;

        devices.push_back({dev_path, card, is_emeet});
    }

    std::sort(devices.begin(), devices.end(), [](const V4L2DeviceInfo &a, const V4L2DeviceInfo &b) {
        if (a.is_emeet != b.is_emeet)
            return a.is_emeet > b.is_emeet;
        return a.path < b.path;
    });
    return devices;
}

inline bool v4l2_get_ctrl(const std::string &dev_path, uint32_t id, int32_t &out_val)
{
    int fd = open(dev_path.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0)
        return false;
    struct v4l2_control ctrl {};
    ctrl.id = id;
    int rc = ioctl(fd, VIDIOC_G_CTRL, &ctrl);
    close(fd);
    if (rc < 0)
        return false;
    out_val = ctrl.value;
    return true;
}

inline bool v4l2_set_ctrl(const std::string &dev_path, uint32_t id, int32_t val)
{
    int fd = open(dev_path.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0)
        return false;
    struct v4l2_control ctrl {};
    ctrl.id = id;
    ctrl.value = val;
    int rc = ioctl(fd, VIDIOC_S_CTRL, &ctrl);
    close(fd);
    return rc >= 0;
}

inline bool v4l2_get_default(const std::string &dev_path, uint32_t id, int32_t &out_def)
{
    int fd = open(dev_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
        return false;
    struct v4l2_queryctrl qctrl {};
    qctrl.id = id;
    int rc = ioctl(fd, VIDIOC_QUERYCTRL, &qctrl);
    close(fd);
    if (rc < 0)
        return false;
    out_def = qctrl.default_value;
    return true;
}

} // namespace emeet
