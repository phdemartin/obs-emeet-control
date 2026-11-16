#pragma once

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace emeet {

enum class HdrMode {
    Off = 0,
    Balanced = 1,
    Strong = 2,
};

class SoftwareHdr {
public:
    static void apply(cv::Mat &bgr, HdrMode mode)
    {
        if (mode == HdrMode::Off || bgr.empty())
            return;

        // Precompute piecewise-linear shadow-lift / highlight-protect LUT
        static const std::array<uint8_t, 256> lut_balanced = build_curve({
            {0.0f, 0.0f},
            {0.15f, 0.24f},
            {0.45f, 0.52f},
            {0.80f, 0.86f},
            {1.0f, 1.0f},
        });
        static const std::array<uint8_t, 256> lut_strong = build_curve({
            {0.0f, 0.0f},
            {0.12f, 0.26f},
            {0.40f, 0.54f},
            {0.78f, 0.86f},
            {1.0f, 1.0f},
        });

        const auto &tbl = (mode == HdrMode::Strong) ? lut_strong : lut_balanced;
        cv::Mat cv_lut(1, 256, CV_8UC1, const_cast<uint8_t *>(tbl.data()));
        cv::LUT(bgr, cv_lut, bgr);

        // Subtle local contrast / unsharp mask
        cv::Mat blurred;
        cv::GaussianBlur(bgr, blurred, cv::Size(0, 0), 2.0);
        const double amount = (mode == HdrMode::Strong) ? 0.35 : 0.22;
        cv::addWeighted(bgr, 1.0 + amount, blurred, -amount, 0.0, bgr);
    }

private:
    static std::array<uint8_t, 256> build_curve(const std::vector<std::pair<float, float>> &pts)
    {
        std::array<uint8_t, 256> out {};
        for (int i = 0; i < 256; ++i) {
            float x = i / 255.0f;
            float y = x;
            for (size_t k = 1; k < pts.size(); ++k) {
                if (x <= pts[k].first) {
                    float x0 = pts[k - 1].first, y0 = pts[k - 1].second;
                    float x1 = pts[k].first, y1 = pts[k].second;
                    float t = (x1 > x0) ? (x - x0) / (x1 - x0) : 0.0f;
                    y = y0 + t * (y1 - y0);
                    break;
                }
            }
            out[i] = static_cast<uint8_t>(std::clamp(std::lround(y * 255.0f), 0L, 255L));
        }
        return out;
    }
};

class Lut3D {
public:
    bool load(const std::string &path)
    {
        if (path == loaded_path_ && !table_.empty())
            return true;

        table_.clear();
        size_ = 0;
        loaded_path_.clear();
        if (path.empty())
            return false;

        std::string lower = path;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

        bool ok = false;
        if (lower.size() >= 4 && lower.rfind(".png") == lower.size() - 4)
            ok = load_png(path);
        else if (lower.size() >= 5 && lower.rfind(".cube") == lower.size() - 5)
            ok = load_cube(path);

        if (ok)
            loaded_path_ = path;
        return ok;
    }

    bool is_loaded() const { return size_ > 1 && !table_.empty(); }

    void apply(cv::Mat &bgr) const
    {
        if (!is_loaded() || bgr.empty())
            return;

        const int n = size_;
        const int n2 = n * n;
        const float scale = (n - 1) / 255.0f;

        std::array<int, 256> idx_map {};
        for (int i = 0; i < 256; ++i)
            idx_map[i] = static_cast<int>(i * scale + 0.5f);

        for (int y = 0; y < bgr.rows; ++y) {
            uint8_t *row = bgr.ptr<uint8_t>(y);
            for (int x = 0; x < bgr.cols; ++x) {
                int b_i = idx_map[row[x * 3 + 0]];
                int g_i = idx_map[row[x * 3 + 1]];
                int r_i = idx_map[row[x * 3 + 2]];
                const size_t offset = static_cast<size_t>((b_i * n2 + g_i * n + r_i) * 3);
                // table_ stores RGB -> write BGR
                row[x * 3 + 0] = table_[offset + 2];
                row[x * 3 + 1] = table_[offset + 1];
                row[x * 3 + 2] = table_[offset + 0];
            }
        }
    }

private:
    bool load_png(const std::string &path)
    {
        cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
        if (img.empty() || img.cols != 512 || img.rows != 512)
            return false;

        size_ = 64;
        table_.resize(64 * 64 * 64 * 3);
        for (int b = 0; b < 64; ++b) {
            int tx = b % 8;
            int ty = b / 8;
            for (int g = 0; g < 64; ++g) {
                const uint8_t *src = img.ptr<uint8_t>(ty * 64 + g) + (tx * 64) * 3;
                for (int r = 0; r < 64; ++r) {
                    size_t dst = static_cast<size_t>(((b * 64 + g) * 64 + r) * 3);
                    table_[dst + 0] = src[r * 3 + 2]; // R
                    table_[dst + 1] = src[r * 3 + 1]; // G
                    table_[dst + 2] = src[r * 3 + 0]; // B
                }
            }
        }
        return true;
    }

    bool load_cube(const std::string &path)
    {
        std::ifstream ifs(path);
        if (!ifs.is_open())
            return false;

        int n = 0;
        std::vector<uint8_t> tmp;
        std::string line;
        while (std::getline(ifs, line)) {
            if (line.empty() || line[0] == '#')
                continue;
            if (line.rfind("LUT_3D_SIZE", 0) == 0) {
                std::istringstream iss(line.substr(11));
                iss >> n;
                if (n > 1)
                    tmp.reserve(static_cast<size_t>(n) * n * n * 3);
                continue;
            }
            if (std::isalpha(static_cast<unsigned char>(line[0])))
                continue;
            std::istringstream iss(line);
            float r, g, b;
            if (iss >> r >> g >> b) {
                tmp.push_back(static_cast<uint8_t>(std::clamp(std::lround(r * 255.0f), 0L, 255L)));
                tmp.push_back(static_cast<uint8_t>(std::clamp(std::lround(g * 255.0f), 0L, 255L)));
                tmp.push_back(static_cast<uint8_t>(std::clamp(std::lround(b * 255.0f), 0L, 255L)));
            }
        }
        if (n <= 1 || tmp.size() != static_cast<size_t>(n) * n * n * 3)
            return false;
        size_ = n;
        table_ = std::move(tmp);
        return true;
    }

    std::string loaded_path_;
    int size_ = 0;
    std::vector<uint8_t> table_;
};

} // namespace emeet
