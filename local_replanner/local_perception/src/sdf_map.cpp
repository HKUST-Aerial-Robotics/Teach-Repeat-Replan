#include "local_perception/sdf_map.h"

// #define current_img_ depth_image_[image_cnt_ & 1]
// #define last_img_ depth_image_[!(image_cnt_ & 1)]

void SDFMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  min_pos(0) = max(min_pos(0), min_range(0));
  min_pos(1) = max(min_pos(1), min_range(1));
  min_pos(2) = max(min_pos(2), min_range(2));

  max_pos(0) = min(max_pos(0), max_range(0));
  max_pos(1) = min(max_pos(1), max_range(1));
  max_pos(2) = min(max_pos(2), max_range(2));

  Eigen::Vector3i min_id, max_id;

  posToIndex(min_pos, min_id);
  posToIndex(max_pos - Eigen::Vector3d(resolution / 2, resolution / 2, resolution / 2), max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        occupancy_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = 0.0;
        distance_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = 10000;
      }
}

inline bool SDFMap::isInMap(Eigen::Vector3d pos)
{
  if (pos(0) < min_range(0) + 1e-4 || pos(1) < min_range(1) + 1e-4 || pos(2) < min_range(2) + 1e-4)
  {
    // cout << "less than min range!" << endl;
    return false;
  }

  if (pos(0) > max_range(0) - 1e-4 || pos(1) > max_range(1) - 1e-4 || pos(2) > max_range(2) - 1e-4)
  {
    // cout << "larger than max range!" << endl;
    return false;
  }

  return true;
}

inline void SDFMap::posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - origin(i)) * resolution_inv);
}

inline void SDFMap::indexToPos(Eigen::Vector3i id, Eigen::Vector3d& pos)
{
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * resolution + origin(i);
}

inline void SDFMap::setOccupancy(Eigen::Vector3d pos, double occ)
{
  if (occ != 1 && occ != 0)
  {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  occupancy_buffer[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)] = occ;
}

inline int SDFMap::getOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return occupancy_buffer[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)] > min_occupancy_log_ ? 1 :
                                                                                                                     0;
}

inline int SDFMap::getInflateOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return occupancy_buffer_inflate_[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)] >
                 min_occupancy_log_ ?
             1 :
             0;
}

inline int SDFMap::getOccupancy(Eigen::Vector3i id)
{
  if (id(0) < 0 || id(0) >= grid_size(0) || id(1) < 0 || id(1) >= grid_size(1) || id(2) < 0 || id(2) >= grid_size(2))
    return -1;

  return occupancy_buffer[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)] > min_occupancy_log_ ? 1 :
                                                                                                                     0;
}

inline double SDFMap::getDistance(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  // (x, y, z) -> x*ny*nz + y*nz + z
  return distance_buffer_all[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)];
}

inline double SDFMap::getDistance(Eigen::Vector3i id, int sign)
{
  id(0) = max(min(id(0), grid_size(0) - 1), 0);
  id(1) = max(min(id(1), grid_size(1) - 1), 0);
  id(2) = max(min(id(2), grid_size(2) - 1), 0);

  return distance_buffer_all[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)];
}

inline double SDFMap::getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad)
{
  if (!isInMap(pos))
  {
    grad.setZero();
    return 0;
  }

  /* use trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * resolution * Eigen::Vector3d::Ones();

  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);

  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);

  diff = (pos - idx_pos) * resolution_inv;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
  {
    for (int y = 0; y < 2; y++)
    {
      for (int z = 0; z < 2; z++)
      {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }
    }
  }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];

  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;

  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * resolution_inv;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

  grad[0] *= resolution_inv;

  return dist;
}

/*inline void SDFMap::setLocalRange(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  min_pos(0) = max(min_pos(0), min_range(0));
  min_pos(1) = max(min_pos(1), min_range(1));
  min_pos(2) = max(min_pos(2), min_range(2));

  max_pos(0) = min(max_pos(0), max_range(0));
  max_pos(1) = min(max_pos(1), max_range(1));
  max_pos(2) = min(max_pos(2), max_range(2));

  posToIndex(min_pos, min_vec);
  posToIndex(max_pos - Eigen::Vector3d(resolution / 2, resolution / 2, resolution / 2), max_vec);
}
*/
template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
{
  int v[grid_size(dim)];
  double z[grid_size(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++)
  {
    k++;
    double s;

    do
    {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++)
  {
    while (z[k + 1] < q)
      k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d()
{
  /* ========== compute positive DT ========== */
  for (int x = esdf_min_[0]; x <= esdf_max_[0]; x++)
  {
    for (int y = esdf_min_[1]; y <= esdf_max_[1]; y++)
    {
      fillESDF(
          [&](int z) {
            return occupancy_buffer_inflate_[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] >
                           min_occupancy_log_ ?
                       0 :
                       std::numeric_limits<double>::max();
          },
          [&](int z, double val) { tmp_buffer1[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = val; },
          esdf_min_[2], esdf_max_[2], 2);
    }
  }

  for (int x = esdf_min_[0]; x <= esdf_max_[0]; x++)
  {
    for (int z = esdf_min_[2]; z <= esdf_max_[2]; z++)
    {
      fillESDF([&](int y) { return tmp_buffer1[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z]; },
               [&](int y, double val) { tmp_buffer2[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = val; },
               esdf_min_[1], esdf_max_[1], 1);
    }
  }

  for (int y = esdf_min_[1]; y <= esdf_max_[1]; y++)
  {
    for (int z = esdf_min_[2]; z <= esdf_max_[2]; z++)
    {
      fillESDF([&](int x) { return tmp_buffer2[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z]; },
               [&](int x, double val) {
                 //  distance_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = resolution *
                 //  std::sqrt(val);
                 distance_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] =
                     min(resolution * std::sqrt(val),
                         distance_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z]);
               },
               esdf_min_[0], esdf_max_[0], 0);
    }
  }

  /* ========== compute negative distance ========== */
  for (int x = esdf_min_(0); x <= esdf_max_(0); ++x)
    for (int y = esdf_min_(1); y <= esdf_max_(1); ++y)
      for (int z = esdf_min_(2); z <= esdf_max_(2); ++z)
      {
        int idx = x * grid_size(1) * grid_size(2) + y * grid_size(2) + z;
        if (occupancy_buffer_inflate_[idx] <= min_occupancy_log_)
        {
          occupancy_buffer_neg[idx] = clamp_max_log_;
        }
        else if (occupancy_buffer_inflate_[idx] > min_occupancy_log_)
        {
          occupancy_buffer_neg[idx] = clamp_min_log_;
        }
        else
        {
          std::cout << "what" << std::endl;
        }
      }

  tmp_buffer1.clear();
  tmp_buffer2.clear();

  ros::Time t1, t2;

  for (int x = esdf_min_[0]; x <= esdf_max_[0]; x++)
  {
    for (int y = esdf_min_[1]; y <= esdf_max_[1]; y++)
    {
      fillESDF(
          [&](int z) {
            return occupancy_buffer_neg[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] > min_occupancy_log_ ?
                       0 :
                       std::numeric_limits<double>::max();
          },
          [&](int z, double val) { tmp_buffer1[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = val; },
          esdf_min_[2], esdf_max_[2], 2);
    }
  }

  for (int x = esdf_min_[0]; x <= esdf_max_[0]; x++)
  {
    for (int z = esdf_min_[2]; z <= esdf_max_[2]; z++)
    {
      fillESDF([&](int y) { return tmp_buffer1[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z]; },
               [&](int y, double val) { tmp_buffer2[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = val; },
               esdf_min_[1], esdf_max_[1], 1);
    }
  }

  for (int y = esdf_min_[1]; y <= esdf_max_[1]; y++)
  {
    for (int z = esdf_min_[2]; z <= esdf_max_[2]; z++)
    {
      fillESDF([&](int x) { return tmp_buffer2[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z]; },
               [&](int x, double val) {
                 distance_buffer_neg[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] =
                     resolution * std::sqrt(val);
               },
               esdf_min_[0], esdf_max_[0], 0);
    }
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = esdf_min_(0); x <= esdf_max_(0); ++x)
    for (int y = esdf_min_(1); y <= esdf_max_(1); ++y)
      for (int z = esdf_min_(2); z <= esdf_max_(2); ++z)
      {
        int idx = x * grid_size(1) * grid_size(2) + y * grid_size(2) + z;

        if (distance_buffer_neg[idx] > 0.0)
          distance_buffer_all[idx] = distance_buffer[idx] - distance_buffer_neg[idx] + resolution;
        else
          distance_buffer_all[idx] = distance_buffer[idx];
      }
}

bool SDFMap::tryFillMinima(const Eigen::Vector3d& pt, Eigen::Vector3d& center, Eigen::Vector3d& cube_len)
{
  vector<Eigen::Vector3d> peaks = findPeaks(pt);
  bool fill = fillLocalMinima(peaks, center, cube_len);

  if (fill)
  {
    // setLocalRange(camera_pos_ - sensor_range_, camera_pos_ + sensor_range_);
    updateESDF3d();
  }

  return fill;
}

bool SDFMap::fillLocalMinima(const vector<Eigen::Vector3d>& peaks, Eigen::Vector3d& center, Eigen::Vector3d& cube_len)
{
  if (peaks.size() == 0)
    return false;

  /* ---------- find bounding box ---------- */
  Eigen::Vector3d box_min, box_max, pk;
  box_min(0) = box_min(1) = box_min(2) = 10000;
  box_max(0) = box_max(1) = box_max(2) = -10000;

  for (int i = 0; i < (int)peaks.size(); ++i)
  {
    pk = peaks[i];
    if (pk(0) > box_max(0))
      box_max(0) = pk(0);
    if (pk(1) > box_max(1))
      box_max(1) = pk(1);
    if (pk(2) > box_max(2))
      box_max(2) = pk(2);

    if (pk(0) < box_min(0))
      box_min(0) = pk(0);
    if (pk(1) < box_min(1))
      box_min(1) = pk(1);
    if (pk(2) < box_min(2))
      box_min(2) = pk(2);
  }
  // cout << "[sdf]: min: " << box_min.transpose() << endl;
  // cout << "[sdf]: max: " << box_max.transpose() << endl;

  /* ---------- expand slice ---------- */
  Eigen::Vector3d mid_pt = 0.5 * (box_min + box_max);
  if ((mid_pt - last_fill_pt).norm() < 0.55)
    return false;

  Eigen::Vector3d cube_scale = box_max - box_min;
  int max_idx = -1;
  double max_scale = -10000;
  for (int i = 0; i < 3; ++i)
  {
    if (cube_scale(i) > max_scale)
    {
      max_scale = cube_scale(i);
      max_idx = i;
    }
  }
  double temp_scale = max(cube_scale((max_idx + 1) % 3), cube_scale((max_idx + 2) % 3));
  cube_scale((max_idx + 1) % 3) = temp_scale;
  cube_scale((max_idx + 2) % 3) = temp_scale;

  const double shrink = 0.5 * resolution;
  cube_scale(0) = max(2 * resolution, cube_scale(0) - shrink);
  cube_scale(1) = max(2 * resolution, cube_scale(1) - shrink);
  cube_scale(2) = max(2 * resolution, cube_scale(2) - shrink);
  // box_max += infla * Eigen::Vector3d::Ones();
  // box_min -= infla * Eigen::Vector3d::Ones();

  /* ---------- fill bounding box ---------- */
  for (double x = mid_pt(0) - cube_scale(0) / 2.0; x <= mid_pt(0) + cube_scale(0) / 2.0; x += resolution)
    for (double y = mid_pt(1) - cube_scale(1) / 2.0; y <= mid_pt(1) + cube_scale(1) / 2.0; y += resolution)
      for (double z = mid_pt(2) - cube_scale(2) / 2.0; z <= mid_pt(2) + cube_scale(2) / 2.0; z += resolution)
      {
        setOccupancy(Eigen::Vector3d(x + 0.01, y + 0.01, z + 0.01));
      }

  /* ---------- set last fill pt ---------- */
  last_fill_pt = mid_pt;
  center = mid_pt;
  cube_len = cube_scale;

  return true;
}

vector<Eigen::Vector3d> SDFMap::findPeaks(const Eigen::Vector3d& pt)
{
  vector<Eigen::Vector3d> peaks;
  /* ---------- iterate surrounding to find peak ---------- */
  Eigen::Vector3i idx;
  posToIndex(pt, idx);

  const int radius = ceil(0.3 / resolution);
  const int radius_z = radius;

  Eigen::Vector3d coord;
  bool peak;
  for (int x = idx(0) - radius; x <= idx(0) + radius; ++x)
    for (int y = idx(1) - radius; y <= idx(1) + radius; ++y)
      for (int z = idx(2) - radius_z; z <= idx(2) + radius_z; ++z)
      {
        indexToPos(Eigen::Vector3i(x, y, z), coord);
        peak = checkPeak(coord);
        if (peak)
        {
          peaks.push_back(coord);
        }
      }
  return peaks;
}

bool SDFMap::checkPeak(Eigen::Vector3d pos)
{
  Eigen::Vector3d grad1, grad2, pos1, pos2;
  double grad_2nd;
  const double thresh = 0.5 * resolution_inv;
  /* ---------- check x axis ---------- */
  pos1 = pos2 = pos;
  pos1(0) = pos(0) - resolution;
  pos2(0) = pos(0) + resolution;
  getDistWithGradTrilinear(pos1, grad1);
  getDistWithGradTrilinear(pos2, grad2);

  grad_2nd = (grad2(0) - grad1(0)) / (2.0 * resolution);
  if (grad_2nd > thresh)
  {
    return true;
  }

  /* ---------- check y axis ---------- */
  pos1 = pos2 = pos;
  pos1(1) = pos(1) - resolution;
  pos2(1) = pos(1) + resolution;
  getDistWithGradTrilinear(pos1, grad1);
  getDistWithGradTrilinear(pos2, grad2);

  grad_2nd = (grad2(1) - grad1(1)) / (2.0 * resolution);
  if (grad_2nd > thresh)
  {
    return true;
  }

  /* ---------- check z axis ---------- */
  pos1 = pos2 = pos;
  pos1(2) = pos(2) - resolution;
  pos2(2) = pos(2) + resolution;
  getDistWithGradTrilinear(pos1, grad1);
  getDistWithGradTrilinear(pos2, grad2);

  grad_2nd = (grad2(2) - grad1(2)) / (2.0 * resolution);
  if (grad_2nd > thresh)
  {
    return true;
  }

  /* ---------- no peak ---------- */
  return false;
}

void SDFMap::initMap(ros::NodeHandle& nh)
{
  node_ = nh;

  /* ---------- get parameter ---------- */
  double x_size, y_size, z_size;
  node_.param("sdf_map/resolution", resolution, -1.0);
  node_.param("sdf_map/global/x_size", x_size, -1.0);
  node_.param("sdf_map/global/y_size", y_size, -1.0);
  node_.param("sdf_map/global/z_size", z_size, -1.0);
  node_.param("sdf_map/local_radius_x", sensor_range_(0), -1.0);
  node_.param("sdf_map/local_radius_y", sensor_range_(1), -1.0);
  node_.param("sdf_map/local_radius_z", sensor_range_(2), -1.0);
  node_.param("sdf_map/inflate_val", inflate_val_, -1.0);

  node_.param("sdf_map/fx", fx_, -1.0);
  node_.param("sdf_map/fy", fy_, -1.0);
  node_.param("sdf_map/cx", cx_, -1.0);
  node_.param("sdf_map/cy", cy_, -1.0);

  node_.param("sdf_map/use_depth_filter", use_depth_filter_, true);
  node_.param("sdf_map/depth_filter_tolerance", depth_filter_tolerance_, -1.0);
  node_.param("sdf_map/depth_filter_maxdist", depth_filter_maxdist_, -1.0);
  node_.param("sdf_map/depth_filter_mindist", depth_filter_mindist_, -1.0);
  node_.param("sdf_map/depth_filter_margin", depth_filter_margin_, -1);
  node_.param("sdf_map/k_depth_scaling_factor", k_depth_scaling_factor_, -1.0);
  node_.param("sdf_map/skip_pixel", skip_pixel_, -1);

  std::cout << "use_depth_filter_: " << use_depth_filter_ << std::endl;
  std::cout << "min: " << depth_filter_mindist_ << std::endl;
  std::cout << "max: " << depth_filter_maxdist_ << std::endl;
  std::cout << "mtol: " << depth_filter_tolerance_ << std::endl;
  std::cout << "fac: " << k_depth_scaling_factor_ << std::endl;

  node_.param("sdf_map/p_hit", p_hit_, 0.70);
  node_.param("sdf_map/p_miss", p_miss_, 0.35);
  node_.param("sdf_map/p_min", p_min_, 0.12);
  node_.param("sdf_map/p_max", p_max_, 0.97);
  node_.param("sdf_map/p_occ", p_occ_, 0.80);
  node_.param("sdf_map/min_ray_length", min_ray_length_, -0.1);
  node_.param("sdf_map/max_ray_length", max_ray_length_, -0.1);

  node_.param("sdf_map/slice_height", slice_height_, -0.1);
  node_.param("sdf_map/cut_height", cut_height_, -0.1);

  node_.param("sdf_map/show_occ_time", show_occ_time_, false);
  node_.param("sdf_map/show_esdf_time", show_esdf_time_, false);
  node_.param("sdf_map/use_uniform_update", use_uniform_update_, true);

  node_.param("sdf_map/pose_type", pose_type_, 1);
  //node_.param("sdf_map/input_data_type", input_data_type_, string("depth"));

  node_.param("sdf_map/frame_id", frame_id_, string("world"));
  node_.param("sdf_map/esdf_inflate", esdf_inflate_, 1.0);
  node_.param("sdf_map/local_map_margin", local_map_margin_, 1);
  node_.param("sdf_map/ground_z", ground_z_, 1.0);

  esdf_inflate_ = max(resolution, esdf_inflate_);

  resolution_inv = 1 / resolution;
  origin   = Eigen::Vector3d(-y_size / 2.0, -y_size / 2.0, ground_z_);
  map_size = Eigen::Vector3d(x_size, y_size, z_size);

  prob_hit_log_ = logit(p_hit_);
  prob_miss_log_ = logit(p_miss_);
  clamp_min_log_ = logit(p_min_);
  clamp_max_log_ = logit(p_max_);
  min_occupancy_log_ = logit(p_occ_);

  // ----------ouput logit------------
  cout << "hit: " << prob_hit_log_ << endl;
  cout << "miss: " << prob_miss_log_ << endl;
  cout << "min: " << clamp_min_log_ << endl;
  cout << "max: " << clamp_max_log_ << endl;
  cout << "thresh: " << min_occupancy_log_ << endl;
  cout << "inflate: " << inflate_val_ << endl;
  cout << "skip: " << skip_pixel_ << endl;

  /* ---------- debug ---------- */
  std::cout << "origin: " << origin.transpose() << std::endl;
  std::cout << "map size: " << map_size.transpose() << std::endl;
  std::cout << "sensor range: " << sensor_range_.transpose() << std::endl;
  std::cout << "res: " << resolution << std::endl;
  std::cout << "fx: " << fx_ << ", " << fy_ << ", " << cx_ << ", " << cy_ << std::endl;
  std::cout << "min: " << min_ray_length_ << std::endl;
  std::cout << "max: " << max_ray_length_ << std::endl;
  std::cout << "min log: " << min_occupancy_log_ << std::endl;
  std::cout << "pose type: " << pose_type_ << std::endl;
  std::cout << "frame id: " << frame_id_ << std::endl;
  std::cout << "esdf inflate: " << esdf_inflate_ << std::endl;

  /* ---------- init map ---------- */

  for (int i = 0; i < 3; ++i)
    grid_size(i) = ceil(map_size(i) / resolution);
  // cout << "grid num:" << grid_size.transpose() << endl;
  min_range = origin;
  max_range = origin + map_size;
  /*min_vec = Eigen::Vector3i::Zero();
  max_vec = grid_size - Eigen::Vector3i::Ones();*/
  last_fill_pt.setZero();

  // initialize size of buffer
  occupancy_buffer.resize(grid_size(0) * grid_size(1) * grid_size(2));
  distance_buffer.resize(grid_size(0) * grid_size(1) * grid_size(2));

  occupancy_buffer_inflate_.resize(grid_size(0) * grid_size(1) * grid_size(2));

  occupancy_buffer_neg.resize(grid_size(0) * grid_size(1) * grid_size(2));
  distance_buffer_neg.resize(grid_size(0) * grid_size(1) * grid_size(2));

  distance_buffer_all.resize(grid_size(0) * grid_size(1) * grid_size(2));

  tmp_buffer1.resize(grid_size(0) * grid_size(1) * grid_size(2));
  tmp_buffer2.resize(grid_size(0) * grid_size(1) * grid_size(2));

  cache_all_.resize(grid_size(0) * grid_size(1) * grid_size(2));
  cache_hit_.resize(grid_size(0) * grid_size(1) * grid_size(2));

  cache_rayend_.resize(grid_size(0) * grid_size(1) * grid_size(2));
  cache_traverse_.resize(grid_size(0) * grid_size(1) * grid_size(2));
  raycast_num_ = 0;

  fill(occupancy_buffer.begin(), occupancy_buffer.end(), clamp_min_log_);
  fill(occupancy_buffer_neg.begin(), occupancy_buffer_neg.end(), clamp_min_log_);
  fill(occupancy_buffer_inflate_.begin(), occupancy_buffer_inflate_.end(), clamp_min_log_);

  fill(distance_buffer.begin(), distance_buffer.end(), 10000);
  fill(distance_buffer_neg.begin(), distance_buffer_neg.end(), 10000);
  fill(distance_buffer_all.begin(), distance_buffer_all.end(), 10000);

  fill(cache_all_.begin(), cache_all_.end(), 0);
  fill(cache_hit_.begin(), cache_hit_.end(), 0);

  fill(cache_rayend_.begin(), cache_rayend_.end(), -1);
  fill(cache_traverse_.begin(), cache_traverse_.end(), -1);

  proj_points_.resize(640 * 480 / skip_pixel_ / skip_pixel_);
  proj_points_cnt = 0;

  output_buffer = new Eigen::Vector3d[200];
  output_points_cnt = 0;

  /* ---------- init callback ---------- */
  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/sdf_map/depth", 100));

  if (pose_type_ == POSE_STAMPED)
  {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/sdf_map/pose", 50));

    sync_image_pose_.reset(
        new message_filters::Synchronizer<SyncPolicyImagePose>(SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));

    sync_image_pose_->registerCallback(boost::bind(&SDFMap::depthPoseCallback, this, _1, _2));
  }
  else if (pose_type_ == ODOMETRY)
  {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/sdf_map/odom", 100));

    sync_image_odom_.reset(
        new message_filters::Synchronizer<SyncPolicyImageOdom>(SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));

    sync_image_odom_->registerCallback(boost::bind(&SDFMap::depthOdomCallback, this, _1, _2));
  }

  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("/sdf_map/odom", 10, &SDFMap::odomCallback, this);

  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("/sdf_map/cloud", 10, &SDFMap::cloudCallback, this);

  occ_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::updateOccupancyCallback, this);
  esdf_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::updateESDFCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_inflate", 10);
  esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
  test_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/test", 10);
  update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);

  occ_need_update_ = false;
  esdf_need_update_ = false;
  has_first_depth_ = false;
  has_odom_  = false;
  has_cloud_ = false;
  image_cnt_ = 0;

  esdf_time_ = 0.0;
  fuse_time_ = 0.0;
  update_num_ = 0;

  max_esdf_time_ = 0.0;
  max_fuse_time_ = 0.0;
}

int SDFMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
  {
    return INVALID_IDX;
  }

  if (!isInMap(pos))
  {
    return INVALID_IDX;
    // --------------- find the nearest point in map range cube --------------------
    // Eigen::Vector3d diff = pos - camera_pos_;
  }

  Eigen::Vector3i id;
  posToIndex(pos, id);

  int idx_ctns = id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2);

  cache_all_[idx_ctns] += 1;

  if (cache_all_[idx_ctns] == 1)
  {
    // cache_voxel_.push(idx_ctns);
    cache_voxel_.push(id);
  }

  if (occ == 1)
    cache_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void SDFMap::projectDepthImage()
{
  // proj_points_.clear();
  proj_points_cnt = 0;

  uint16_t* row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = depth_image_.cols;
  int rows = depth_image_.rows;

  double depth;

  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();

  // cout << "rotate: " << camera_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << camera_pos_ << std::endl;

  if (!use_depth_filter_)
  {
    for (int v = 0; v < rows; v++)
    {
      row_ptr = depth_image_.ptr<uint16_t>(v);
      for (int u = 0; u < cols; u++)
      {
        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / k_depth_scaling_factor_;
        proj_pt(0) = (u - cx_) * depth / fx_;
        proj_pt(1) = (v - cy_) * depth / fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        proj_points_[proj_points_cnt++] = proj_pt;

        // proj_points_.push_back(proj_pt);
      }
    }

    // proj_cloud.width = proj_cloud.points.size();
    // proj_cloud.height = 1;
    // proj_cloud.is_dense = true;
    // proj_cloud.header.frame_id = frame_id_;
    // sensor_msgs::PointCloud2 cloud_msg;
    // pcl::toROSMsg(proj_cloud, cloud_msg);

    // test_pub_.publish(cloud_msg);
  }
  /* ---------- use depth filter ---------- */
  else
  {
    if (!has_first_depth_)
      has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = last_camera_q_.inverse();

      const double inv_factor = 1.0 / k_depth_scaling_factor_;

      for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_)
      {
        row_ptr = depth_image_.ptr<uint16_t>(v) + depth_filter_margin_;
        for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_)
        {
          depth = (*row_ptr) * inv_factor;

          pt_cur(0) = (u - cx_) * depth / fx_;
          pt_cur(1) = (v - cy_) * depth / fy_;
          pt_cur(2) = depth;

          row_ptr = row_ptr + skip_pixel_;

          /* jump too far or too close */
          if (depth > depth_filter_maxdist_ || depth < depth_filter_mindist_)
            continue;

          /* check consistency */
          pt_world = camera_r * pt_cur + camera_pos_;
          pt_reproj = last_camera_r_inv * (pt_world - last_camera_pos_);

          double uu = pt_reproj.x() * fx_ / pt_reproj.z() + cx_;
          double vv = pt_reproj.y() * fy_ / pt_reproj.z() + cy_;

          if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
          {
            if (fabs(last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor - pt_reproj.z()) <
                depth_filter_tolerance_)
            {
              proj_points_[proj_points_cnt++] = pt_world;
              // proj_points_.push_back(pt_world);
            }
          }
          else
          {
            proj_points_[proj_points_cnt++] = pt_world;
          }
        }
      }
    }
  }

  /* ---------- maintain last ---------- */
  last_camera_pos_ = camera_pos_;
  last_camera_q_ = camera_q_;
  last_depth_image_ = depth_image_;
}

void SDFMap::raycastProcess()
{
  // if (proj_points_.size() == 0)
  if (proj_points_cnt == 0)
    return;

  // raycast_num_ = (raycast_num_ + 1) % 100000;
  raycast_num_ += 1;

  // std::cout << "size: " << proj_points_.size() << std::endl;

  /* ---------- iterate projected points ---------- */
  ros::Time t1, t2;

  int set_cache_idx;
  /* ---------- for updating esdf ---------- */
  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = max_range(0);
  min_y = max_range(1);
  min_z = max_range(2);

  max_x = min_range(0);
  max_y = min_range(1);
  max_z = min_range(2);

  // std::cout << "proj point cnt: " << proj_points_cnt << std::endl;
  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt;

  for (int i = 0; i < proj_points_cnt; ++i)
  {
    /* ---------- occupancy of ray end ---------- */
    Eigen::Vector3d pt_w = proj_points_[i];

    if (!isInMap(pt_w))
      continue;

    double length = (pt_w - camera_pos_).norm();

    if (length < min_ray_length_)
      continue;
    else if (length > max_ray_length_)
    {
      pt_w = (pt_w - camera_pos_) / length * max_ray_length_ + camera_pos_;
      set_cache_idx = setCacheOccupancy(pt_w, 0);
    }
    else
      set_cache_idx = setCacheOccupancy(pt_w, 1);

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    /* ---------- ignore close end ray ---------- */
    if (set_cache_idx != INVALID_IDX)
    {
      if (cache_rayend_[set_cache_idx] == raycast_num_)
      {
        continue;
      }
      else
        cache_rayend_[set_cache_idx] = raycast_num_;
    }

    bool need_ray = raycaster.setInput(pt_w / resolution,
                                       camera_pos_ / resolution /* , box_min / resolution, box_max / resolution */);

    if (!need_ray)
      continue;

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * resolution;
      length = (tmp - camera_pos_).norm();

      if (length < min_ray_length_)
        break;

      set_cache_idx = setCacheOccupancy(tmp, 0);

      if (set_cache_idx != INVALID_IDX)
      {
        if (cache_traverse_[set_cache_idx] == raycast_num_)
          break;
        else
          cache_traverse_[set_cache_idx] = raycast_num_;
      }
    }
  }

  /* ---------- cut and save local esdf update range ---------- */
  min_x = min(min_x, camera_pos_(0));
  min_y = min(min_y, camera_pos_(1));
  min_z = min(min_z, camera_pos_(2));

  max_x = max(max_x, camera_pos_(0));
  max_y = max(max_y, camera_pos_(1));
  max_z = max(max_z, camera_pos_(2));

  max_z = max(max_z, ground_z_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), esdf_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), esdf_min_);

  int esdf_inf = ceil(esdf_inflate_ / resolution);
  esdf_max_ += esdf_inf * Eigen::Vector3i::Ones();
  esdf_min_ -= esdf_inf * Eigen::Vector3i::Ones();

  /* avoid exceed global range */
  esdf_max_ = esdf_max_.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  esdf_max_ = esdf_max_.array().max(Eigen::Vector3i::Zero().array());

  esdf_min_ = esdf_min_.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  esdf_min_ = esdf_min_.array().max(Eigen::Vector3i::Zero().array());

  /* ---------- update occupancy in batch ---------- */
  while (!cache_voxel_.empty())
  {
    Eigen::Vector3i idx = cache_voxel_.front();
    int idx_ctns = idx(0) * grid_size(1) * grid_size(2) + idx(1) * grid_size(2) + idx(2);
    cache_voxel_.pop();

    double log_odds_update =
        cache_hit_[idx_ctns] >= cache_all_[idx_ctns] - cache_hit_[idx_ctns] ? prob_hit_log_ : prob_miss_log_;

    cache_hit_[idx_ctns] = cache_all_[idx_ctns] = 0;

    if ((log_odds_update >= 0 && occupancy_buffer[idx_ctns] >= clamp_max_log_) ||
        (log_odds_update <= 0 && occupancy_buffer[idx_ctns] <= clamp_min_log_))
      continue;

    Eigen::Vector3d local_range_min = camera_pos_ - sensor_range_;
    Eigen::Vector3d local_range_max = camera_pos_ + sensor_range_;
    Eigen::Vector3i min_id, max_id;
    posToIndex(local_range_min, min_id);
    posToIndex(local_range_max, max_id);

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) && idx(1) <= max_id(1) &&
                    idx(2) >= min_id(2) && idx(2) <= max_id(2);

    if (!in_local)
    {
      occupancy_buffer[idx_ctns] = clamp_min_log_;
    }

    occupancy_buffer[idx_ctns] =
        std::min(std::max(occupancy_buffer[idx_ctns] + log_odds_update, clamp_min_log_), clamp_max_log_);
  }
}

void SDFMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts)
{
  int num = 0;

  for (int x = -step; x <= step; ++x)
  {
    if (x == 0)
      continue;
    pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  }
  for (int y = -step; y <= step; ++y)
  {
    if (y == 0)
      continue;
    pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  }
  for (int z = -1; z <= 1; ++z)
  {
    pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  }
}

void SDFMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  // Eigen::Vector3i max_vec_margin = max_vec + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = esdf_min_ - Eigen::Vector3i(local_map_margin_, local_map_margin_, local_map_margin_);
  Eigen::Vector3i max_cut = esdf_max_ + Eigen::Vector3i(local_map_margin_, local_map_margin_, local_map_margin_);

  max_cut = max_cut.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  max_cut = max_cut.array().max(Eigen::Vector3i::Zero().array());

  min_cut = min_cut.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  min_cut = min_cut.array().max(Eigen::Vector3i::Zero().array());

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  max_cut_m = max_cut_m.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  max_cut_m = max_cut_m.array().max(Eigen::Vector3i::Zero().array());

  min_cut_m = min_cut_m.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  min_cut_m = min_cut_m.array().max(Eigen::Vector3i::Zero().array());

  int inf_step = ceil(inflate_val_ / resolution);
  //int inf_step_z = 1;

  /* ---------- clear map outside local range ---------- */

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
      for (int z = min_cut_m(2); z <= min_cut(2); ++z)
      {
        int idx = x * grid_size(1) * grid_size(2) + y * grid_size(2) + z;
        occupancy_buffer[idx] = clamp_min_log_;
        distance_buffer_all[idx] = 10000;
      }

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
      for (int z = max_cut(2); z <= max_cut_m(2); ++z)
      {
        int idx = x * grid_size(1) * grid_size(2) + y * grid_size(2) + z;
        occupancy_buffer[idx] = clamp_min_log_;
        distance_buffer_all[idx] = 10000;
      }

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= min_cut(1); ++y)
      for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
      {
        int idx = x * grid_size(1) * grid_size(2) + y * grid_size(2) + z;
        occupancy_buffer[idx] = clamp_min_log_;
        distance_buffer_all[idx] = 10000;
      }

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = max_cut(1); y <= max_cut_m(1); ++y)
      for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
      {
        int idx = x * grid_size(1) * grid_size(2) + y * grid_size(2) + z;
        occupancy_buffer[idx] = clamp_min_log_;
        distance_buffer_all[idx] = 10000;
      }

  for (int x = min_cut_m(0); x <= min_cut(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
      for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
      {
        int idx = x * grid_size(1) * grid_size(2) + y * grid_size(2) + z;
        occupancy_buffer[idx] = clamp_min_log_;
        distance_buffer_all[idx] = 10000;
      }

  for (int x = max_cut(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
      for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
      {
        int idx = x * grid_size(1) * grid_size(2) + y * grid_size(2) + z;
        occupancy_buffer[idx] = clamp_min_log_;
        distance_buffer_all[idx] = 10000;
      }

  /* ---------- inflate map ---------- */
  vector<Eigen::Vector3i> inf_pts;
  inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  for (int x = esdf_min_(0); x <= esdf_max_(0); ++x)
    for (int y = esdf_min_(1); y <= esdf_max_(1); ++y)
      for (int z = esdf_min_(2); z <= esdf_max_(2); ++z)
      {
        occupancy_buffer_inflate_[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = clamp_min_log_;
      }

  for (int x = esdf_min_(0); x <= esdf_max_(0); ++x)
    for (int y = esdf_min_(1); y <= esdf_max_(1); ++y)
      for (int z = esdf_min_(2); z <= esdf_max_(2); ++z)
      {
        /* inflate the update map */
        if (occupancy_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] > min_occupancy_log_)
        {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];

            int idx_inf = inf_pt[0] * grid_size(1) * grid_size(2) + inf_pt[1] * grid_size(2) + inf_pt[2];

            if (idx_inf < 0 || idx_inf > grid_size(0) * grid_size(1) * grid_size(2))
            {
              continue;
            }

            occupancy_buffer_inflate_[idx_inf] = clamp_max_log_;
          }
        }
      }
}

void SDFMap::updateOccupancyCallback(const ros::TimerEvent& /*event*/)
{
  if (!occ_need_update_)
    return;
  /*----------- update occupancy -----------*/
  ros::Time t1, t2;

  t1 = ros::Time::now();

  /* reproj depth image to world frame  */
  projectDepthImage();

  /* raycasting */
  raycastProcess();

  /* for inconsistency induced by VIO drift */
  clearAndInflateLocalMap();

  t2 = ros::Time::now();
  fuse_time_ += (t2 - t1).toSec();
  max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());

  if (show_occ_time_)
    ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(), fuse_time_ / update_num_,
             max_fuse_time_);

  occ_need_update_ = false;
  esdf_need_update_ = true;
}

void SDFMap::updateESDFCallback(const ros::TimerEvent& /*event*/)
{
  if (!esdf_need_update_)
    return;

  /* ---------- esdf ---------- */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  updateESDF3d();

  t2 = ros::Time::now();
  esdf_time_ += (t2 - t1).toSec();
  max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());

  if (show_esdf_time_)
    ROS_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(), esdf_time_ / update_num_,
             max_esdf_time_);

  esdf_need_update_ = false;
}

void SDFMap::depthPoseCallback(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose)
{
  /* ---------- get depth image ---------- */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(depth_image_);

  /* ---------- get pose ---------- */
  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;
  camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y,
                                 pose->pose.orientation.z);

  update_num_ += 1;
  occ_need_update_ = true;
}

void SDFMap::depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom)
{
  /* ---------- get pose ---------- */
  camera_pos_(0) = odom->pose.pose.position.x;
  camera_pos_(1) = odom->pose.pose.position.y;
  camera_pos_(2) = odom->pose.pose.position.z;

  camera_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                 odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  /* ---------- get depth image ---------- */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(depth_image_);

  occ_need_update_ = true;
}

void SDFMap::depthCallback(const sensor_msgs::ImageConstPtr& img)
{
  std::cout << "depth: " << img->header.stamp << std::endl;
}
void SDFMap::poseCallback(const geometry_msgs::PoseStampedConstPtr& pose)
{
  std::cout << "pose: " << pose->header.stamp << std::endl;

  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;

  has_odom_ = true;
}

void SDFMap::odomCallback(const nav_msgs::OdometryConstPtr& odom)
{ 
  if(!has_cloud_)
    return;
  
  camera_pos_(0) = odom->pose.pose.position.x;
  camera_pos_(1) = odom->pose.pose.position.y;
  camera_pos_(2) = odom->pose.pose.position.z;

  //std::cout<<"odom"<<std::endl;
  has_odom_ = true;
}

void SDFMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img)
{
  //std::cout << "cloud: " << img->header.stamp << std::endl;

  pcl::PointCloud<pcl::PointXYZ> latest_cloud, cloud_inflate_vis;
  pcl::fromROSMsg(*img, latest_cloud);

  has_cloud_ = true;

  if (!has_odom_)
  {
    //std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(camera_pos_(0)) || isnan(camera_pos_(1)) || isnan(camera_pos_(2)))
    return;

  this->resetBuffer(camera_pos_ - sensor_range_, camera_pos_ + sensor_range_);

  cloud_inflate_vis.clear();
  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(inflate_val_ / resolution);
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = max_range(0);
  min_y = max_range(1);
  min_z = max_range(2);

  max_x = min_range(0);
  max_y = min_range(1);
  max_z = min_range(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* point inside update range */
    Eigen::Vector3d devi = p3d - camera_pos_;
    Eigen::Vector3i inf_pt;
    if (fabs(devi(0)) < sensor_range_(0) && fabs(devi(1)) < sensor_range_(1) && fabs(devi(2)) < sensor_range_(2))
    {
      /* inflate the point */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z)
          {
            p3d_inf(0) = pt.x + x * resolution;
            p3d_inf(1) = pt.y + y * resolution;
            p3d_inf(2) = pt.z + z * resolution;

            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            posToIndex(p3d_inf, inf_pt);

            int idx_inf = inf_pt[0] * grid_size(1) * grid_size(2) + inf_pt[1] * grid_size(2) + inf_pt[2];
            if (idx_inf < 0 || idx_inf > grid_size(0) * grid_size(1) * grid_size(2))            
              continue;

            occupancy_buffer_inflate_[idx_inf] = clamp_max_log_;
          }
    }
  }

  min_x = min(min_x, camera_pos_(0));
  min_y = min(min_y, camera_pos_(1));
  min_z = min(min_z, camera_pos_(2));

  max_x = max(max_x, camera_pos_(0));
  max_y = max(max_y, camera_pos_(1));
  max_z = max(max_z, camera_pos_(2));

  max_z = max(max_z, ground_z_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), esdf_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), esdf_min_);

  /* avoid exceed global range */
  esdf_max_ = esdf_max_.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  esdf_max_ = esdf_max_.array().max(Eigen::Vector3i::Zero().array());

  esdf_min_ = esdf_min_.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  esdf_min_ = esdf_min_.array().max(Eigen::Vector3i::Zero().array());

  has_first_depth_ = true;
  esdf_need_update_ = true;
}

void SDFMap::publishMap()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = esdf_min_ - Eigen::Vector3i(local_map_margin_, local_map_margin_, local_map_margin_);
  Eigen::Vector3i max_cut = esdf_max_ + Eigen::Vector3i(local_map_margin_, local_map_margin_, local_map_margin_);
  max_cut = max_cut.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  max_cut = max_cut.array().max(Eigen::Vector3i::Zero().array());

  min_cut = min_cut.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  min_cut = min_cut.array().max(Eigen::Vector3i::Zero().array());

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (occupancy_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] <= min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        if (pos(2) > cut_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);

        cloud.points.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

void SDFMap::publishMapInflate()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // iterate the local map
  Eigen::Vector3i min_cut = esdf_min_ - Eigen::Vector3i(local_map_margin_, local_map_margin_, local_map_margin_);
  Eigen::Vector3i max_cut = esdf_max_ + Eigen::Vector3i(local_map_margin_, local_map_margin_, local_map_margin_);
  max_cut = max_cut.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  max_cut = max_cut.array().max(Eigen::Vector3i::Zero().array());

  min_cut = min_cut.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  min_cut = min_cut.array().max(Eigen::Vector3i::Zero().array());

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (occupancy_buffer_inflate_[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] <= min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        if (pos(2) > cut_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

void SDFMap::publishUpdateRange()
{
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  indexToPos(esdf_min_, esdf_min_pos);
  indexToPos(esdf_max_, esdf_max_pos);
  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;

  visualization_msgs::Marker mk;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  mk.pose.position.x = cube_pos(0), mk.pose.position.y = cube_pos(1), mk.pose.position.z = cube_pos(2);
  mk.scale.x = cube_scale(0), mk.scale.y = cube_scale(1), mk.scale.z = cube_scale(2);
  mk.color.a = 0.3, mk.color.r = 1.0, mk.color.g = 0.0, mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void SDFMap::publishESDF()
{
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = esdf_min_ - Eigen::Vector3i(local_map_margin_, local_map_margin_, local_map_margin_);
  Eigen::Vector3i max_cut = esdf_max_ + Eigen::Vector3i(local_map_margin_, local_map_margin_, local_map_margin_);
  max_cut = max_cut.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  max_cut = max_cut.array().max(Eigen::Vector3i::Zero().array());

  min_cut = min_cut.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  min_cut = min_cut.array().max(Eigen::Vector3i::Zero().array());

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
    {
      {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, 1), pos);
        pos(2) = slice_height_;

        dist = getDistance(pos);
        dist = min(dist, max_dist);
        dist = max(dist, min_dist);

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = 0.3;
        pt.intensity = (dist - min_dist) / (max_dist - min_dist);

        cloud.push_back(pt);
      }
    }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}

void SDFMap::getSliceESDF(const double height, const double res, Eigen::Vector4d range, vector<Eigen::Vector3d>& slice,
                          vector<Eigen::Vector3d>& grad, int sign)
{
  double dist;
  Eigen::Vector3d gd;
  for (double x = range(0); x <= range(1); x += res)
    for (double y = range(2); y <= range(3); y += res)
    {
      dist = this->getDistWithGradTrilinear(Eigen::Vector3d(x, y, height), gd);
      slice.push_back(Eigen::Vector3d(x, y, dist));
      grad.push_back(gd);
    }
}

void SDFMap::checkDist()
{
  for (int x = 0; x < grid_size(0); ++x)
    for (int y = 0; y < grid_size(1); ++y)
      for (int z = 0; z < grid_size(2); ++z)
      {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        Eigen::Vector3d grad;
        double dist = getDistWithGradTrilinear(pos, grad);

      }
}

// SDFMap::
