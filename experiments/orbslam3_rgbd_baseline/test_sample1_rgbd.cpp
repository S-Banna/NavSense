#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <functional>
#include <limits.h>
#include <unistd.h>
#include <cstddef>
#include <cstdint>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "System.h"
#include "MapDrawer.h"

namespace
{
// Update only this path when moving the project or changing your calibration.
const std::string kSettings =
    "/home/jad/Desktop/Final Year Project/Navsense-git/NavSense/experiments/orbslam3_rgbd_baseline/orbslam3_tecno.yaml";

const std::string kVocabulary =
    "/home/jad/opt/orbslam-deps/ORB_SLAM3/Vocabulary/ORBvoc.txt";

struct FramePair
{
    std::string stem;
    std::string rgb_path;
    std::string depth_path;
    double timestamp;
};

struct VoxelKey
{
    int x, y, z;
    bool operator==(const VoxelKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelHash
{
    std::size_t operator()(const VoxelKey& voxel) const
    {
        std::size_t h = std::hash<int>{}(voxel.x);
        h ^= std::hash<int>{}(voxel.y) + 0x9e3779b9u + (h << 6) + (h >> 2);
        h ^= std::hash<int>{}(voxel.z) + 0x9e3779b9u + (h << 6) + (h >> 2);
        return h;
    }
};

std::string joinPath(const std::string& left, const std::string& right)
{
    return left.empty() ? right :
           (left.back() == '/' ? left + right : left + "/" + right);
}

bool isDirectory(const std::string& path)
{
    struct stat info;
    return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

bool isFile(const std::string& path)
{
    struct stat info;
    return stat(path.c_str(), &info) == 0 && S_ISREG(info.st_mode);
}

bool makeDirectory(const std::string& path)
{
    if (isDirectory(path)) return true;
    return mkdir(path.c_str(), 0755) == 0;
}

std::string stemOf(const std::string& path)
{
    const std::size_t slash = path.find_last_of("/\\");
    const std::string filename = slash == std::string::npos ? path : path.substr(slash + 1);
    const std::size_t dot = filename.find_last_of('.');
    return dot == std::string::npos ? filename : filename.substr(0, dot);
}

std::string extOf(const std::string& path)
{
    const std::size_t dot = path.find_last_of('.');
    if (dot == std::string::npos) return "";
    std::string ext = path.substr(dot);
    std::transform(ext.begin(), ext.end(), ext.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

std::map<std::string, std::string> listImages(const std::string& folder, bool depth)
{
    std::map<std::string, std::string> images;
    DIR* directory = opendir(folder.c_str());
    if (!directory) return images;

    struct dirent* entry;
    while ((entry = readdir(directory)) != nullptr)
    {
        const std::string path = joinPath(folder, entry->d_name);
        if (!isFile(path)) continue;
        const std::string ext = extOf(path);
        const bool supported = depth
            ? (ext == ".png" || ext == ".tif" || ext == ".tiff")
            : (ext == ".jpg" || ext == ".jpeg" || ext == ".png");
        if (supported) images[stemOf(path)] = path;
    }
    closedir(directory);
    return images;
}

bool timestampFromStem(const std::string& stem, double& timestamp)
{
    // Session filenames are integer phone-clock nanoseconds, e.g. 5234567890123456.jpg.
    if (stem.empty() || stem.find_first_not_of("0123456789") != std::string::npos)
        return false;
    try
    {
        timestamp = static_cast<double>(std::stoull(stem)) / 1000000000.0;
        return std::isfinite(timestamp);
    }
    catch (...)
    {
        return false;
    }
}

std::vector<FramePair> findPairs(const std::string& session)
{
    const auto rgb = listImages(joinPath(session, "mav0/cam0/data"), false);
    const auto depth = listImages(joinPath(session, "depth"), true);
    std::vector<FramePair> pairs;

    for (const auto& item : rgb)
    {
        const auto found = depth.find(item.first);
        if (found == depth.end()) continue;
        double timestamp = 0.0;
        if (!timestampFromStem(item.first, timestamp)) continue;
        pairs.push_back({item.first, item.second, found->second, timestamp});
    }

    std::sort(pairs.begin(), pairs.end(),
        [](const FramePair& a, const FramePair& b)
        {
            return a.timestamp < b.timestamp;
        });
    return pairs;
}

// Use one fixed source YAML, but create a run-specific copy so Atlas output
// goes into this session rather than overwriting another session's map.
bool writeRunSettings(const std::string& output, std::string& run_settings)
{
    std::ifstream source(kSettings);
    if (!source) return false;
    run_settings = joinPath(output, "orbslam3_run.yaml");
    std::ofstream target(run_settings);
    if (!target) return false;

    std::string line;
    while (std::getline(source, line))
    {
        const std::size_t pos = line.find_first_not_of(" \t");
        const std::string trimmed = pos == std::string::npos ? "" : line.substr(pos);
        if (trimmed.find("System.LoadAtlasFromFile:") == 0 ||
            trimmed.find("System.SaveAtlasToFile:") == 0)
            continue;
        target << line << '\n';
    }
    target << "System.SaveAtlasToFile: \"slam/orbslam3/atlas\"\n";
    return target.good();
}

void writeMetadata(const std::string& output, int ok, int lost,
                   int failed, int processed, bool trajectory_saved)
{
    std::ofstream meta(joinPath(output, "trajectory_meta.json"));
    meta << "{\n"
         << "  \"frame\": \"camera\",\n"
         << "  \"method\": \"ORB-SLAM3 RGB-D\",\n"
         << "  \"mode\": \"mapping\",\n"
         << "  \"scale\": \"metric\",\n"
         << "  \"tracking_lost_count\": " << lost << ",\n"
         << "  \"tracking_ok_count\": " << ok << ",\n"
         << "  \"failed_image_pairs\": " << failed << ",\n"
         << "  \"processed_frames\": " << processed << ",\n"
         << "  \"trajectory_saved\": " << (trajectory_saved ? "true" : "false") << ",\n"
         << "  \"notes\": \"Phone-clock timestamps in seconds; RGB-D uses depth PNGs in millimetres.\"\n"
         << "}\n";
}

const char* trackingStateName(int state)
{
    switch (state)
    {
        case 0: return "NO_IMAGES_YET";
        case 1: return "NOT_INITIALIZED";
        case 2: return "OK";
        case 3: return "RECENTLY_LOST";
        case 4: return "LOST";
        case 5: return "OK_KLT";
        default: return "UNKNOWN";
    }
}
}

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " SESSION_FOLDER\n";
        return 2;
    }

    char resolved[PATH_MAX];
    if (!realpath(argv[1], resolved))
    {
        std::cerr << "Could not resolve session path: " << argv[1] << '\n';
        return 1;
    }
    const std::string session = resolved;
    if (!isDirectory(session))
    {
        std::cerr << "Session folder not found: " << session << '\n';
        return 1;
    }
    if (!isFile(kSettings) || !isFile(kVocabulary))
    {
        std::cerr << "Missing settings YAML or ORB vocabulary.\n"
                  << "Settings: " << kSettings << '\n'
                  << "Vocabulary: " << kVocabulary << '\n';
        return 1;
    }

    const std::vector<FramePair> frames = findPairs(session);
    if (frames.empty())
    {
        std::cerr << "No matching timestamp-named RGB/depth images found.\n"
                  << "RGB: " << joinPath(session, "mav0/cam0/data") << '\n'
                  << "Depth: " << joinPath(session, "depth") << '\n';
        return 1;
    }

    const std::string slam = joinPath(session, "slam");
    const std::string output = joinPath(slam, "orbslam3");
    if (!makeDirectory(slam) || !makeDirectory(output))
    {
        std::cerr << "Could not create output folder: " << output << '\n';
        return 1;
    }

    std::string run_settings;
    if (!writeRunSettings(output, run_settings))
    {
        std::cerr << "Could not prepare settings from: " << kSettings << '\n';
        return 1;
    }

    cv::FileStorage calibration(run_settings, cv::FileStorage::READ);
    if (!calibration.isOpened())
    {
        std::cerr << "Could not read ORB-SLAM3 settings.\n";
        return 1;
    }
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0, depth_factor = 0.0;
    calibration["Camera1.fx"] >> fx;
    calibration["Camera1.fy"] >> fy;
    calibration["Camera1.cx"] >> cx;
    calibration["Camera1.cy"] >> cy;
    calibration["RGBD.DepthMapFactor"] >> depth_factor;
    calibration.release();
    if (fx <= 0 || fy <= 0 || depth_factor <= 0)
    {
        std::cerr << "Invalid camera intrinsics or depth factor in YAML.\n";
        return 1;
    }

    std::cout << "Session: " << session << '\n'
              << "Matched RGB/depth frames: " << frames.size() << '\n'
              << "Camera settings: " << kSettings << '\n'
              << "Output: " << output << '\n' << std::flush;

    // ORB-SLAM3 prepends "./" to its Atlas save setting, so run in the
    // session folder to place atlas.osa under session/slam/orbslam3/.
    if (chdir(session.c_str()) != 0)
    {
        std::cerr << "Could not change to session folder.\n";
        return 1;
    }

    // Show the native ORB-SLAM3 viewer. No console prompt: the hub can launch this.
    ORB_SLAM3::System SLAM(kVocabulary, run_settings, ORB_SLAM3::System::RGBD, true);

    std::unordered_set<VoxelKey, VoxelHash> seen_voxels;
    const int pixel_stride = 8;
    const float voxel_size = 0.03f;
    std::size_t dense_points_added = 0;
    int processed = 0, failed = 0, tracking_ok = 0, lost = 0, recent_lost = 0;

    for (std::size_t index = 0; index < frames.size(); ++index)
    {
        const FramePair& pair = frames[index];
        cv::Mat image = cv::imread(pair.rgb_path, cv::IMREAD_COLOR);
        cv::Mat depth = cv::imread(pair.depth_path, cv::IMREAD_UNCHANGED);
        if (image.empty() || depth.empty() || image.size() != depth.size() ||
            depth.type() != CV_16UC1)
        {
            std::cerr << "Skipping invalid RGB/depth pair: " << pair.stem << '\n';
            ++failed;
            continue;
        }

        const Sophus::SE3f Tcw = SLAM.TrackRGBD(image, depth, pair.timestamp);
        const int state = SLAM.GetTrackingState();
        ++processed;
        if (state == 2 || state == 5) ++tracking_ok;
        if (state == 3) ++recent_lost;
        if (state == 4) ++lost;

        if (state == 2 && index % 2 == 0)
        {
            const Sophus::SE3f Twc = Tcw.inverse();
            std::vector<ORB_SLAM3::MapDrawer::DensePoint> new_points;
            new_points.reserve(10000);

            for (int v = 0; v < depth.rows; v += pixel_stride)
            {
                for (int u = 0; u < depth.cols; u += pixel_stride)
                {
                    const std::uint16_t raw = depth.at<std::uint16_t>(v, u);
                    if (raw == 0) continue;
                    const float z = static_cast<float>(raw / depth_factor);
                    if (!std::isfinite(z) || z < 0.25f || z > 8.0f) continue;
                    const Eigen::Vector3f camera_point(
                        static_cast<float>((u - cx) * z / fx),
                        static_cast<float>((v - cy) * z / fy), z);
                    const Eigen::Vector3f world_point = Twc * camera_point;
                    if (!world_point.allFinite()) continue;
                    const VoxelKey key{
                        static_cast<int>(std::floor(world_point.x() / voxel_size)),
                        static_cast<int>(std::floor(world_point.y() / voxel_size)),
                        static_cast<int>(std::floor(world_point.z() / voxel_size))
                    };
                    if (!seen_voxels.insert(key).second) continue;
                    const cv::Vec3b color = image.at<cv::Vec3b>(v, u);
                    new_points.push_back({world_point.x(), world_point.y(), world_point.z(),
                        color[2] / 255.0f, color[1] / 255.0f, color[0] / 255.0f});
                }
            }
            if (!new_points.empty())
            {
                SLAM.AddDensePoints(new_points);
                dense_points_added += new_points.size();
            }
        }

        std::cout << "Frame " << index << "/" << frames.size()
                  << " [" << pair.stem << "]"
                  << " | " << trackingStateName(state)
                  << " | dense points=" << dense_points_added << std::endl;
    }

    std::cout << "Shutting down and saving Atlas..." << std::endl;
    SLAM.Shutdown(); // System.SaveAtlasToFile in run-specific YAML saves atlas.osa here.

    bool trajectory_saved = false;
    if (tracking_ok > 0)
    {
        const std::string trajectory = joinPath(output, "trajectory_tum.txt");
        SLAM.SaveTrajectoryTUM(trajectory);
        SLAM.SaveKeyFrameTrajectoryTUM(joinPath(output, "keyframe_trajectory_tum.txt"));
        trajectory_saved = isFile(trajectory);
    }
    writeMetadata(output, tracking_ok, lost, failed, processed, trajectory_saved);

    std::cout << "Processed=" << processed << " OK=" << tracking_ok
              << " RECENTLY_LOST=" << recent_lost << " LOST=" << lost
              << " Failed pairs=" << failed << '\n'
              << "Atlas: " << joinPath(output, "atlas.osa") << '\n'
              << "Results: " << output << std::endl;

    if (!tracking_ok || !trajectory_saved)
    {
        std::cerr << "SLAM did not produce a valid trajectory.\n";
        return 1;
    }
    return 0;
}
