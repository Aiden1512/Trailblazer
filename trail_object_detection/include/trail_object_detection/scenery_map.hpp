#ifndef TRAIL_OBJECT_DETECTION__SCENERY_MAP_HPP_
#define TRAIL_OBJECT_DETECTION__SCENERY_MAP_HPP_

// ROS-free "known scenery" map: the occupancy grid saved by slam_toolbox / map_saver at the end of the
// Phase 1 mapping run. Everything in it (trees, rocks, fences) was there when the trails were mapped, and
// the robot drove past it, so the LiDAR stop ignores it; only obstacles that are NEW since mapping (not in
// this map) can block a trail. Some trees have low foliage reaching over the trail, which the LiDAR
// rightly sees and the camera rightly says is "on the trail", so this is what tells them apart from a rock
// that has been put on the trail since.
//
// Occupied cells are grown by `tolerance` metres, because the SLAM scan is a single height slice and the
// LiDAR also sees lower parts of the same tree slightly further out.

#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace trail_object_detection
{

class SceneryMap
{
public:
  // grid is row 0 = BOTTOM of the map (y = origin_y), occupied = true. Cells within tolerance of an occupied
  // cell count as scenery.
  void setGrid(int width, int height, double resolution, double origin_x, double origin_y,
               const std::vector<bool> &occupied, double tolerance)
  {
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    near_.assign(static_cast<std::size_t>(width) * height, false);
    occupied_cells_ = 0;
    const int r = static_cast<int>(std::ceil(tolerance / resolution));
    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        if (!occupied[static_cast<std::size_t>(y) * width + x])
        {
          continue;
        }
        ++occupied_cells_;
        for (int dy = -r; dy <= r; ++dy)
        {
          for (int dx = -r; dx <= r; ++dx)
          {
            const int nx = x + dx, ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= width || ny >= height || dx * dx + dy * dy > r * r)
            {
              continue;
            }
            near_[static_cast<std::size_t>(ny) * width + nx] = true;
          }
        }
      }
    }
  }

  // Load a map_saver YAML (+ its PGM image). Returns false with `error` set on failure.
  bool load(const std::string &yaml_path, double tolerance, std::string &error)
  {
    try
    {
      const YAML::Node yaml = YAML::LoadFile(yaml_path);
      std::string image = yaml["image"].as<std::string>();
      if (!image.empty() && image[0] != '/')
      {
        const auto slash = yaml_path.find_last_of('/');
        image = (slash == std::string::npos ? std::string() : yaml_path.substr(0, slash + 1)) + image;
      }
      const double resolution = yaml["resolution"].as<double>();
      const double origin_x = yaml["origin"][0].as<double>();
      const double origin_y = yaml["origin"][1].as<double>();
      const double occupied_thresh = yaml["occupied_thresh"].as<double>(0.65);
      const bool negate = yaml["negate"].as<int>(0) != 0;

      int width = 0, height = 0;
      std::vector<std::uint8_t> pixels;
      if (!readPgm(image, width, height, pixels, error))
      {
        return false;
      }
      // map_server: occupancy p = (255 - value) / 255 (or value / 255 if negate); occupied if p > thresh.
      // PGM row 0 is the TOP of the map.
      std::vector<bool> occupied(static_cast<std::size_t>(width) * height, false);
      for (int row = 0; row < height; ++row)
      {
        for (int x = 0; x < width; ++x)
        {
          const double value = pixels[static_cast<std::size_t>(row) * width + x];
          const double p = negate ? value / 255.0 : (255.0 - value) / 255.0;
          occupied[static_cast<std::size_t>(height - 1 - row) * width + x] = p > occupied_thresh;
        }
      }
      setGrid(width, height, resolution, origin_x, origin_y, occupied, tolerance);
      return true;
    }
    catch (const std::exception &ex)
    {
      error = std::string("could not read map YAML: ") + ex.what();
      return false;
    }
  }

  // Is (x, y) in the map frame at (or within tolerance of) something that was there when the trails were mapped?
  bool isScenery(double x, double y) const
  {
    if (near_.empty())
    {
      return false;
    }
    const int cx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    const int cy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_)
    {
      return false; // outside the mapped area: unknown, so not scenery
    }
    return near_[static_cast<std::size_t>(cy) * width_ + cx];
  }

  bool loaded() const { return !near_.empty(); }
  int occupiedCells() const { return occupied_cells_; }

private:
  static bool readPgm(const std::string &path, int &width, int &height, std::vector<std::uint8_t> &pixels,
                      std::string &error)
  {
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
      error = "could not open map image '" + path + "'";
      return false;
    }
    // Header: "P5", width, height, maxval, separated by whitespace, with optional '#' comments
    std::vector<std::string> fields;
    std::string token;
    while (fields.size() < 4 && in >> token)
    {
      if (token[0] == '#')
      {
        std::string rest;
        std::getline(in, rest);
        continue;
      }
      fields.push_back(token);
    }
    if (fields.size() < 4 || fields[0] != "P5")
    {
      error = "map image '" + path + "' is not a binary (P5) PGM";
      return false;
    }
    width = std::stoi(fields[1]);
    height = std::stoi(fields[2]);
    if (std::stoi(fields[3]) > 255)
    {
      error = "16-bit PGM map images are not supported";
      return false;
    }
    in.get(); // the single whitespace byte before the pixel data
    pixels.resize(static_cast<std::size_t>(width) * height);
    in.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
    if (in.gcount() != static_cast<std::streamsize>(pixels.size()))
    {
      error = "map image '" + path + "' is truncated";
      return false;
    }
    return true;
  }

  int width_ = 0, height_ = 0;
  double resolution_ = 0.05, origin_x_ = 0.0, origin_y_ = 0.0;
  std::vector<bool> near_;
  int occupied_cells_ = 0;
};

} // namespace trail_object_detection

#endif
