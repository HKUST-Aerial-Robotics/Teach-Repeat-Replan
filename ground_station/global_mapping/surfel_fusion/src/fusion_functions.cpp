#include "fusion_functions.h"
#include <thread>
#include <cmath>
#include <chrono>
#include <omp.h>

void FusionFunctions::initialize(
    int _width, int _height,
    float _fx, float _fy, float _cx, float _cy,
    float _fuse_far, float _fuse_near)
{
    image_width = _width;
    image_height = _height;
    sp_width = image_width / SP_SIZE;
    sp_height = image_height / SP_SIZE;
    fx = _fx;
    fy = _fy;
    cx = _cx;
    cy = _cy;

    fuse_far = _fuse_far;
    fuse_near = _fuse_near;

    superpixel_seeds.resize(sp_width * sp_height);
    superpixel_index.resize(image_width * image_height);
    space_map.resize(image_width * image_height * 3);
    norm_map.resize(image_width * image_height * 3);
}

void FusionFunctions::fuse_initialize_map(
    int reference_frame_index,
    cv::Mat &input_image,
    cv::Mat &input_depth,
    Eigen::Matrix4f &pose,
    std::vector<SurfelElement> &local_surfels,
    std::vector<SurfelElement> &new_surfels)
{
    std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    std::chrono::duration<double> total_time;
    start_time = std::chrono::system_clock::now();

    // image = input_image.clone();
    // depth = input_depth.clone();

    image = input_image;
    depth = input_depth;

    local_surfels_ptr = &local_surfels;
    new_surfels_ptr = &new_surfels;

    generate_super_pixels();
    // generate_super_pixels_openmp();

    end_time = std::chrono::system_clock::now();
    total_time = end_time - start_time;
//    printf("FusionFunctions::generate superpixels cost %f ms.\n", total_time.count() * 1000.0);
    start_time = std::chrono::system_clock::now();

    // fuse
    Eigen::Matrix4f inv_pose = pose.inverse();
    std::vector<std::thread> thread_pool;
    for (int i = 0; i < THREAD_NUM; i++)
    {
        std::thread this_thread(
            &FusionFunctions::fuse_surfels_kernel, this, i, THREAD_NUM,
            reference_frame_index,
            pose, inv_pose);
        thread_pool.push_back(std::move(this_thread));
    }
    for (int i = 0; i < thread_pool.size(); i++)
        if (thread_pool[i].joinable())
            thread_pool[i].join();

    end_time = std::chrono::system_clock::now();
    total_time = end_time - start_time;
//   printf("FusionFunctions::fuse superpixels cost %f ms.\n", total_time.count() * 1000.0);
    start_time = std::chrono::system_clock::now();

    // initialize
    initialize_surfels(reference_frame_index, pose);
    end_time = std::chrono::system_clock::now();
    total_time = end_time - start_time;
  //  printf("FusionFunctions::initialize superpixels cost %f ms.\n", total_time.count() * 1000.0);
}

void FusionFunctions::project(float &x, float &y, float &z, float &u, float &v)
{
    u = x * fx / z + cx;
    v = y * fy / z + cy;
}

void FusionFunctions::back_project(
    const float &u, const float &v, const float &depth, double &x, double &y, double &z)
{
    x = (u - cx) / fx * depth;
    y = (v - cy) / fy * depth;
    z = depth;
}

float FusionFunctions::get_weight(float &depth)
{
    return std::min(1.0 / depth / depth, 1.0);
}

void FusionFunctions::get_huber_norm(
    float &nx, float &ny, float &nz, float &nb,
    std::vector<float> &points)
{
    int point_num = points.size() / 3;
    float sum_x, sum_y, sum_z;
    sum_x = sum_y = sum_z = 0.0;
    for(int i = 0; i < point_num; i++)
    {
        sum_x += points[i * 3];
        sum_y += points[i * 3 + 1];
        sum_z += points[i * 3 + 2];
    }
    sum_x /= point_num;
    sum_y /= point_num;
    sum_z /= point_num;
    nb = 0;
    for (int i = 0; i < point_num; i++)
    {
        points[i * 3] -= sum_x;
        points[i * 3 + 1] -= sum_y;
        points[i * 3 + 2] -= sum_z;
    }
    for(int gn_i = 0; gn_i < 5; gn_i++)
    {
        Eigen::Matrix4d hessian = Eigen::Matrix4d::Zero();
        Eigen::Vector4d jacobian = Eigen::Vector4d::Zero();
        for(int i = 0; i < point_num; i++)
        {
            float residual = points[i * 3] * nx + points[i * 3 + 1] * ny + points[i * 3 + 2] * nz + nb;
            if (residual < HUBER_RANGE && residual > -1*HUBER_RANGE)
            {
                jacobian(0) += 2 * residual * points[i * 3];
                jacobian(1) += 2 * residual * points[i * 3 + 1];
                jacobian(2) += 2 * residual * points[i * 3 + 2];
                jacobian(3) += 2 * residual;
                hessian(0, 0) += 2 * points[i * 3] * points[i * 3];
                hessian(0, 1) += 2 * points[i * 3] * points[i * 3 + 1];
                hessian(0, 2) += 2 * points[i * 3] * points[i * 3 + 2];
                hessian(0, 3) += 2 * points[i * 3];
                hessian(1, 0) += 2 * points[i * 3 + 1] * points[i * 3];
                hessian(1, 1) += 2 * points[i * 3 + 1] * points[i * 3 + 1];
                hessian(1, 2) += 2 * points[i * 3 + 1] * points[i * 3 + 2];
                hessian(1, 3) += 2 * points[i * 3 + 1];
                hessian(2, 0) += 2 * points[i * 3 + 2] * points[i * 3];
                hessian(2, 1) += 2 * points[i * 3 + 2] * points[i * 3 + 1];
                hessian(2, 2) += 2 * points[i * 3 + 2] * points[i * 3 + 2];
                hessian(2, 3) += 2 * points[i * 3 + 2];
                hessian(3, 0) += 2 * points[i * 3];
                hessian(3, 1) += 2 * points[i * 3 + 1];
                hessian(3, 2) += 2 * points[i * 3 + 2];
                hessian(3, 3) += 2;
            }
            else if (residual >= HUBER_RANGE)
            {
                jacobian(0) += HUBER_RANGE * points[i * 3];
                jacobian(1) += HUBER_RANGE * points[i * 3 + 1];
                jacobian(2) += HUBER_RANGE * points[i * 3 + 2];
                jacobian(3) += HUBER_RANGE;
            }
            else if (residual <= -1 * HUBER_RANGE)
            {
                jacobian(0) += -1 * HUBER_RANGE * points[i * 3];
                jacobian(1) += -1 * HUBER_RANGE * points[i * 3 + 1];
                jacobian(2) += -1 * HUBER_RANGE * points[i * 3 + 2];
                jacobian(3) += -1 * HUBER_RANGE;
            }
        }
        hessian(0, 0) += 5;
        hessian(1, 1) += 5;
        hessian(2, 2) += 5;
        hessian(3, 3) += 5;
        Eigen::Vector4d update_value = hessian.inverse() * jacobian;
        nx -= update_value(0);
        ny -= update_value(1);
        nz -= update_value(2);
        nb -= update_value(3);
    }
    nb = nb - (nx*sum_x+ny*sum_y+nz*sum_z);
    float norm_length = std::sqrt(nx*nx + ny*ny + nz*nz);
    nx /= norm_length;
    ny /= norm_length;
    nz /= norm_length;
    nb /= norm_length;
}

void FusionFunctions::fuse_surfels_kernel(
    int thread_i, int thread_num,
    int reference_frame_index,
    Eigen::Matrix4f pose,
    Eigen::Matrix4f inv_pose)
{
    std::vector<SurfelElement> &local_surfels = *local_surfels_ptr;

    int step = local_surfels.size() / thread_num;
    int begin_index = step * thread_i;
    int end_index = begin_index + step;
    if (thread_i == thread_num - 1)
        end_index = local_surfels.size();

    for (int i = begin_index; i < end_index; i++)
    {
        // remove unstable
        if (reference_frame_index - local_surfels[i].last_update > 10 && local_surfels[i].update_times < 6)
        {
            local_surfels[i].update_times = 0;
            continue;
        }

        if (local_surfels[i].update_times == 0)
            continue;
        Eigen::Vector4f surfel_p_w;
        surfel_p_w(0) = local_surfels[i].px;
        surfel_p_w(1) = local_surfels[i].py;
        surfel_p_w(2) = local_surfels[i].pz;
        surfel_p_w(3) = 1.0;
        Eigen::Vector4f surfel_p_c = inv_pose * surfel_p_w;
        if (surfel_p_c(2) < fuse_near || surfel_p_c(2) > fuse_far)
            continue;
        Eigen::Vector3f norm_w;
        norm_w(0) = local_surfels[i].nx;
        norm_w(1) = local_surfels[i].ny;
        norm_w(2) = local_surfels[i].nz;
        Eigen::Vector3f norm_c;
        norm_c = inv_pose.block<3, 3>(0, 0) * norm_w;
        // float predict_view_cos = (norm_w(0) * surfel_p_c(0) + norm_w(1) * surfel_p_c(1) + norm_w(2) * surfel_p_c(2)) 
        //     / std::sqrt(surfel_p_c(0)*surfel_p_c(0)+surfel_p_c(1)*surfel_p_c(1)+surfel_p_c(2)*surfel_p_c(2));
        // if(predict_view_cos<MAX_ANGLE_COS)
        //     continue;
        float project_u, project_v;
        project(surfel_p_c(0), surfel_p_c(1), surfel_p_c(2), project_u, project_v);
        int p_u_int = project_u + 0.5;
        int p_v_int = project_v + 0.5;
        if (p_u_int < 1 || p_u_int > image_width - 2 || p_v_int < 1 || p_v_int > image_height - 2)
            continue;
        if (surfel_p_c(2) < depth.at<float>(p_v_int, p_u_int) - 0.5)
        {
            local_surfels[i].update_times -= 1;
            continue;
        }
        int sp_index = superpixel_index[p_v_int * image_width + p_u_int];
        if (superpixel_seeds[sp_index].norm_x == 0 && superpixel_seeds[sp_index].norm_y == 0 && superpixel_seeds[sp_index].norm_z == 0)
            continue;
        if (superpixel_seeds[sp_index].view_cos < MAX_ANGLE_COS)
            continue;

        float camera_f = (fabs(fx) + fabs(fy)) / 2.0;
        float tolerate_diff =
            surfel_p_c(2) * surfel_p_c(2) / (BASELINE * camera_f) * DISPARITY_ERROR;
        tolerate_diff = tolerate_diff < MIN_TOLERATE_DIFF ? MIN_TOLERATE_DIFF : tolerate_diff;

        // ##############################################
        if(surfel_p_c(2) < superpixel_seeds[sp_index].mean_depth - tolerate_diff)
        {
            // local_surfels[i].update_times = 0;
            continue;
        }
        if(surfel_p_c(2) > superpixel_seeds[sp_index].mean_depth + tolerate_diff)
            continue;
        // ##############################################

        float norm_diff_cos = norm_c(0) * superpixel_seeds[sp_index].norm_x 
            + norm_c(1) * superpixel_seeds[sp_index].norm_y 
            + norm_c(2) * superpixel_seeds[sp_index].norm_z;
        if (norm_diff_cos < MAX_ANGLE_COS)
        {
            local_surfels[i].update_times = 0;
            continue;
        }
        float old_weigth = local_surfels[i].weight;
        float new_weight = get_weight(superpixel_seeds[sp_index].mean_depth);
        float sum_weight = old_weigth + new_weight;
        Eigen::Vector4f sp_p_c, sp_p_w;
        sp_p_c(0) = superpixel_seeds[sp_index].posi_x;
        sp_p_c(1) = superpixel_seeds[sp_index].posi_y;
        sp_p_c(2) = superpixel_seeds[sp_index].posi_z;
        sp_p_c(3) = 1.0;
        sp_p_w = pose * sp_p_c;
        float fused_px = (local_surfels[i].px * old_weigth + new_weight * sp_p_w(0)) / sum_weight;
        float fused_py = (local_surfels[i].py * old_weigth + new_weight * sp_p_w(1)) / sum_weight;
        float fused_pz = (local_surfels[i].pz * old_weigth + new_weight * sp_p_w(2)) / sum_weight;
        float fused_nx = norm_c(0) * old_weigth + new_weight * superpixel_seeds[sp_index].norm_x;
        float fused_ny = norm_c(1) * old_weigth + new_weight * superpixel_seeds[sp_index].norm_y;
        float fused_nz = norm_c(2) * old_weigth + new_weight * superpixel_seeds[sp_index].norm_z;
        double new_norm_length = std::sqrt(fused_nx * fused_nx + fused_ny * fused_ny + fused_nz * fused_nz);
        fused_nx /= new_norm_length;
        fused_ny /= new_norm_length;
        fused_nz /= new_norm_length;
        Eigen::Vector3f new_norm_c, new_norm_w;
        new_norm_c(0) = fused_nx;
        new_norm_c(1) = fused_ny;
        new_norm_c(2) = fused_nz;
        new_norm_w = pose.block<3, 3>(0, 0) * new_norm_c;
        local_surfels[i].px = fused_px;
        local_surfels[i].py = fused_py;
        local_surfels[i].pz = fused_pz;
        local_surfels[i].nx = new_norm_w(0);
        local_surfels[i].ny = new_norm_w(1);
        local_surfels[i].nz = new_norm_w(2);
        local_surfels[i].weight = sum_weight;
        local_surfels[i].color = superpixel_seeds[sp_index].mean_intensity;
        float new_size = superpixel_seeds[sp_index].size * fabs(superpixel_seeds[sp_index].mean_depth / (camera_f * superpixel_seeds[sp_index].view_cos));
        if (new_size < local_surfels[i].size)
            local_surfels[i].size = new_size;
        local_surfels[i].last_update = reference_frame_index;
        
        if(local_surfels[i].update_times < 20)
            local_surfels[i].update_times += 1;
        superpixel_seeds[sp_index].fused = true;
    }
}

void FusionFunctions::initialize_surfels(
    int reference_frame_index,
    Eigen::Matrix4f pose)
{
    std::vector<SurfelElement> &new_surfels = *new_surfels_ptr;
    new_surfels.clear();
    Eigen::Vector4f position_temp_c, position_temp_w;
    Eigen::Vector3f norm_temp_c, norm_temp_w;
    for (int i = 0; i < superpixel_seeds.size(); i++)
    {
        if (superpixel_seeds[i].mean_depth < fuse_near || superpixel_seeds[i].mean_depth > fuse_far)
            continue;
        if (superpixel_seeds[i].fused)
            continue;
        if (superpixel_seeds[i].view_cos < MAX_ANGLE_COS)
            continue;
        position_temp_c(0) = superpixel_seeds[i].posi_x;
        position_temp_c(1) = superpixel_seeds[i].posi_y;
        position_temp_c(2) = superpixel_seeds[i].posi_z;
        position_temp_c(3) = 1.0;
        norm_temp_c(0) = superpixel_seeds[i].norm_x;
        norm_temp_c(1) = superpixel_seeds[i].norm_y;
        norm_temp_c(2) = superpixel_seeds[i].norm_z;
        if (norm_temp_c(0) == 0 && norm_temp_c(1) == 0 && norm_temp_c(2) == 0)
            continue;
        position_temp_w = pose * position_temp_c;
        norm_temp_w = pose.block<3, 3>(0, 0) * norm_temp_c;

        SurfelElement new_ele;
        new_ele.px = position_temp_w(0);
        new_ele.py = position_temp_w(1);
        new_ele.pz = position_temp_w(2);
        new_ele.nx = norm_temp_w(0);
        new_ele.ny = norm_temp_w(1);
        new_ele.nz = norm_temp_w(2);
        float camera_f = (fabs(fx) + fabs(fy)) / 2.0;
        float new_size = superpixel_seeds[i].size * fabs(superpixel_seeds[i].mean_depth / (camera_f * superpixel_seeds[i].view_cos));
        // if(new_size > 0.1)
        //     std::cout << "max eigen: " << superpixel_seeds[i].max_eigen_value << ", min eigen: " << superpixel_seeds[i].min_eigen_value << ", camera focal: " << camera_f << ", distence: " << superpixel_seeds[i].mean_depth << ", cos: " << superpixel_seeds[i].view_cos << " ->size: " << new_size << std::endl;
        new_ele.size = new_size;
        new_ele.color = superpixel_seeds[i].mean_intensity;
        new_ele.weight = get_weight(superpixel_seeds[i].mean_depth);
        new_ele.update_times = 1;
        new_ele.last_update = reference_frame_index;
        new_surfels.push_back(new_ele);
    }
}

// superpixel related
bool FusionFunctions::calculate_cost(
    float &nodepth_cost, float &depth_cost,
    const float &pixel_intensity, const float &pixel_inverse_depth,
    const int &x, const int &y,
    const int &sp_x, const int &sp_y)
{
    int sp_index = sp_y * sp_width + sp_x;
    nodepth_cost = 0;
    float dist =
        (superpixel_seeds[sp_index].x - x) * (superpixel_seeds[sp_index].x - x) + (superpixel_seeds[sp_index].y - y) * (superpixel_seeds[sp_index].y - y);
    nodepth_cost += dist / ((SP_SIZE / 2) * (SP_SIZE / 2));
    float intensity_diff = (superpixel_seeds[sp_index].mean_intensity - pixel_intensity);
    nodepth_cost += intensity_diff * intensity_diff / 100.0;
    depth_cost = nodepth_cost;
    if (superpixel_seeds[sp_index].mean_depth > 0 && pixel_inverse_depth > 0)
    {
        float inverse_depth_diff = 1.0 / superpixel_seeds[sp_index].mean_depth - pixel_inverse_depth;
        depth_cost += inverse_depth_diff * inverse_depth_diff * 400.0;
        // float inverse_depth_diff = superpixel_seeds[sp_index].mean_depth - 1.0/pixel_inverse_depth;
        // depth_cost += inverse_depth_diff * inverse_depth_diff * 400.0;
        return true;
    }
    return false;
}

void FusionFunctions::update_pixels_kernel(
    int thread_i, int thread_num)
{
    int step_row = image_height / thread_num;
    int start_row = step_row * thread_i;
    int end_row = start_row + step_row;
    if(thread_i == thread_num - 1)
        end_row = image_height;
    for(int row_i = start_row; row_i < end_row; row_i++)
    for(int col_i = 0; col_i < image_width; col_i++)
    {
        if(superpixel_seeds[superpixel_index[row_i * image_width + col_i]].stable)
            continue;
        float my_intensity = image.at<uchar>(row_i, col_i);
        float my_inv_depth = 0.0;
        if (depth.at<float>(row_i, col_i) > 0.01)
            my_inv_depth = 1.0 / depth.at<float>(row_i, col_i);
        int base_sp_x = col_i / SP_SIZE;
        int base_sp_y = row_i / SP_SIZE;
        float min_dist_depth = 1e6;
        int min_sp_index_depth = -1;
        float min_dist_nodepth = 1e6;
        int min_sp_index_nodepth = -1;
        bool all_has_depth = true;
        for(int check_i = -1; check_i <= 1; check_i ++)
        for(int check_j = -1; check_j <= 1; check_j ++)
        {
            int check_sp_x = base_sp_x + check_i;
            int check_sp_y = base_sp_y + check_j;
            int dist_sp_x = fabs(check_sp_x * SP_SIZE + SP_SIZE/2 - col_i);
            int dist_sp_y = fabs(check_sp_y * SP_SIZE + SP_SIZE/2 - row_i);
            if (dist_sp_x < SP_SIZE && dist_sp_y < SP_SIZE &&
                check_sp_x >= 0 && check_sp_x < sp_width &&
                check_sp_y >= 0 && check_sp_y < sp_height)
            {
                float dist_depth, dist_nodepth;
                all_has_depth &= calculate_cost(
                    dist_nodepth,
                    dist_depth,
                    my_intensity, my_inv_depth,
                    col_i, row_i, check_sp_x, check_sp_y);
                if (dist_depth < min_dist_depth)
                {
                    min_dist_depth = dist_depth;
                    min_sp_index_depth = (base_sp_y + check_j) * sp_width + base_sp_x + check_i;
                }
                if (dist_nodepth < min_dist_nodepth)
                {
                    min_dist_nodepth = dist_nodepth;
                    min_sp_index_nodepth = (base_sp_y + check_j) * sp_width + base_sp_x + check_i;
                }
            }
        }
        if(all_has_depth)
        {
            superpixel_index[row_i * image_width + col_i] = min_sp_index_depth;
            superpixel_seeds[min_sp_index_depth].stable = false;
        }
        else
        {
            superpixel_index[row_i * image_width + col_i] = min_sp_index_nodepth;
            superpixel_seeds[min_sp_index_nodepth].stable = false;
        }
    }
}

void FusionFunctions::update_pixels()
{
    std::vector<std::thread> thread_pool;
    for (int i = 0; i < THREAD_NUM; i++)
    {
        std::thread this_thread(&FusionFunctions::update_pixels_kernel, this, i, THREAD_NUM);
        thread_pool.push_back(std::move(this_thread));
    }
    for (int i = 0; i < thread_pool.size(); i++)
        if (thread_pool[i].joinable())
            thread_pool[i].join();
}

// void FusionFunctions::update_seeds_kernel(
//     int thread_i, int thread_num)
// {
//     int step = superpixel_seeds.size() / thread_num;
//     int begin_index = step * thread_i;
//     int end_index = begin_index + step;
//     if(thread_i == thread_num - 1)
//         end_index = superpixel_seeds.size();
//     for (int seed_i = begin_index; seed_i < end_index; seed_i++)
//     {
//         if(superpixel_seeds[seed_i].stable)
//             continue;
//         int sp_x = seed_i % sp_width;
//         int sp_y = seed_i / sp_width;
//         int check_x_begin = sp_x * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
//         int check_y_begin = sp_y * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
//         int check_x_end = check_x_begin + SP_SIZE * 2;
//         int check_y_end = check_y_begin + SP_SIZE * 2;
//         check_x_begin = check_x_begin > 0 ? check_x_begin : 0;
//         check_y_begin = check_y_begin > 0 ? check_y_begin : 0;
//         check_x_end = check_x_end < image_width - 1 ? check_x_end : image_width - 1;
//         check_y_end = check_y_end < image_height - 1 ? check_y_end : image_height - 1;
//         float sum_x = 0;
//         float sum_y = 0;
//         float sum_intensity = 0.0;
//         float sum_intensity_num = 0.0;
//         float sum_depth = 0.0;
//         float sum_depth_num = 0.0;
//         for (int check_j = check_y_begin; check_j < check_y_end; check_j++)
//         for (int check_i = check_x_begin; check_i < check_x_end; check_i ++)
//         {
//             int pixel_index = check_j * image_width + check_i;
//             if (superpixel_index[pixel_index] == seed_i)
//             {
//                 sum_x += check_i;
//                 sum_y += check_j;
//                 sum_intensity_num += 1.0;
//                 sum_intensity += image.at<uchar>(check_j, check_i);
//                 if (depth.at<float>(check_j, check_i) > 0.1)
//                 {
//                     sum_depth += depth.at<float>(check_j, check_i);
//                     sum_depth_num += 1.0;
//                 }
//             }
//         }
//         if (sum_intensity_num == 0)
//             return;
//         sum_intensity /= sum_intensity_num;
//         sum_x /= sum_intensity_num;
//         sum_y /= sum_intensity_num;
//         float pre_intensity = superpixel_seeds[seed_i].mean_intensity;
//         float pre_x = superpixel_seeds[seed_i].x;
//         float pre_y = superpixel_seeds[seed_i].y;
//         superpixel_seeds[seed_i].mean_intensity = sum_intensity;
//         superpixel_seeds[seed_i].x = sum_x;
//         superpixel_seeds[seed_i].y = sum_y;
//         float update_diff = fabs(pre_intensity - sum_intensity) + fabs(pre_x - sum_x) + fabs(pre_y - sum_y);
//         if (update_diff < 0.2)
//             superpixel_seeds[seed_i].stable = true;
//         if (sum_depth_num > 0)
//         {
//             superpixel_seeds[seed_i].mean_depth = sum_depth / sum_depth_num;
//             // void();
//         }
//         else
//         {
//             superpixel_seeds[seed_i].mean_depth = 0.0;
//         }
//     }
// }

void FusionFunctions::update_seeds_kernel(
    int thread_i, int thread_num)
{
    int step = superpixel_seeds.size() / thread_num;
    int begin_index = step * thread_i;
    int end_index = begin_index + step;
    if(thread_i == thread_num - 1)
        end_index = superpixel_seeds.size();
    for (int seed_i = begin_index; seed_i < end_index; seed_i++)
    {
        if(superpixel_seeds[seed_i].stable)
            continue;
        int sp_x = seed_i % sp_width;
        int sp_y = seed_i / sp_width;
        int check_x_begin = sp_x * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
        int check_y_begin = sp_y * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
        int check_x_end = check_x_begin + SP_SIZE * 2;
        int check_y_end = check_y_begin + SP_SIZE * 2;
        check_x_begin = check_x_begin > 0 ? check_x_begin : 0;
        check_y_begin = check_y_begin > 0 ? check_y_begin : 0;
        check_x_end = check_x_end < image_width - 1 ? check_x_end : image_width - 1;
        check_y_end = check_y_end < image_height - 1 ? check_y_end : image_height - 1;
        float sum_x = 0;
        float sum_y = 0;
        float sum_intensity = 0.0;
        float sum_intensity_num = 0.0;
        float sum_depth = 0.0;
        float sum_depth_num = 0.0;
        std::vector<float> depth_vector;
        for (int check_j = check_y_begin; check_j < check_y_end; check_j++)
        for (int check_i = check_x_begin; check_i < check_x_end; check_i ++)
        {
            int pixel_index = check_j * image_width + check_i;
            if (superpixel_index[pixel_index] == seed_i)
            {
                sum_x += check_i;
                sum_y += check_j;
                sum_intensity_num += 1.0;
                sum_intensity += image.at<uchar>(check_j, check_i);
                float check_depth = depth.at<float>(check_j, check_i);
                if (check_depth > 0.1)
                {
                    depth_vector.push_back(check_depth);
                    sum_depth += check_depth;
                    sum_depth_num += 1.0;
                }
            }
        }
        if (sum_intensity_num == 0)
            return;
        sum_intensity /= sum_intensity_num;
        sum_x /= sum_intensity_num;
        sum_y /= sum_intensity_num;
        float pre_intensity = superpixel_seeds[seed_i].mean_intensity;
        float pre_x = superpixel_seeds[seed_i].x;
        float pre_y = superpixel_seeds[seed_i].y;
        superpixel_seeds[seed_i].mean_intensity = sum_intensity;
        superpixel_seeds[seed_i].x = sum_x;
        superpixel_seeds[seed_i].y = sum_y;
        float update_diff = fabs(pre_intensity - sum_intensity) + fabs(pre_x - sum_x) + fabs(pre_y - sum_y);
        if (update_diff < 0.2)
            superpixel_seeds[seed_i].stable = true;
        if (sum_depth_num > 0)
        {
            float mean_depth = sum_depth / sum_depth_num;
            float sum_a, sum_b;
            for (int newton_i = 0; newton_i < 5; newton_i++)
            {
                sum_a = sum_b = 0;
                for (int p_i = 0; p_i < depth_vector.size(); p_i++)
                {
                    float residual = mean_depth - depth_vector[p_i];
                    if (residual < HUBER_RANGE && residual > -HUBER_RANGE)
                    {
                        sum_a += 2 * residual;
                        sum_b += 2;
                    }
                    else
                    {
                        sum_a += residual > 0 ? HUBER_RANGE : -1 * HUBER_RANGE;
                    }
                }
                mean_depth = mean_depth - sum_a / (sum_b + 10.0);
            }
            superpixel_seeds[seed_i].mean_depth = mean_depth;
        }
        else
        {
            superpixel_seeds[seed_i].mean_depth = 0.0;
        }
    }
}

void FusionFunctions::update_seeds()
{
    std::vector<std::thread> thread_pool;
    for (int i = 0; i < THREAD_NUM; i++)
    {
        std::thread this_thread(&FusionFunctions::update_seeds_kernel, this, i, THREAD_NUM);
        thread_pool.push_back(std::move(this_thread));
    }
    for (int i = 0; i < thread_pool.size(); i++)
        if (thread_pool[i].joinable())
            thread_pool[i].join();
}

void FusionFunctions::initialize_seeds_kernel(
    int thread_i, int thread_num)
{
    int step = superpixel_seeds.size() / thread_num;
    int begin_index = step * thread_i;
    int end_index = begin_index + step;
    if (thread_i == thread_num - 1)
        end_index = superpixel_seeds.size();
    for (int seed_i = begin_index; seed_i < end_index; seed_i++)
    {
        int sp_x = seed_i % sp_width;
        int sp_y = seed_i / sp_width;
        int image_x = sp_x * SP_SIZE + SP_SIZE / 2;
        int image_y = sp_y * SP_SIZE + SP_SIZE / 2;
        image_x = image_x < (image_width - 1) ? image_x : (image_width - 1);
        image_y = image_y < (image_height - 1) ? image_y : (image_height - 1);
        Superpixel_seed this_sp;
        this_sp.x = image_x;
        this_sp.y = image_y;
        this_sp.mean_intensity = image.at<uchar>(image_y, image_x);
        this_sp.fused = false;
        this_sp.stable = false;
        this_sp.mean_depth = depth.at<float>(image_y, image_x);
        if(this_sp.mean_depth < 0.01)
        {
            int check_x_begin = sp_x * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
            int check_y_begin = sp_y * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
            int check_x_end = check_x_begin + SP_SIZE * 2;
            int check_y_end = check_y_begin + SP_SIZE * 2;
            check_x_begin = check_x_begin > 0 ? check_x_begin : 0;
            check_y_begin = check_y_begin > 0 ? check_y_begin : 0;
            check_x_end = check_x_end < image_width - 1 ? check_x_end : image_width - 1;
            check_y_end = check_y_end < image_height - 1 ? check_y_end : image_height - 1;
            bool find_depth = false;
            for (int check_j = check_y_begin; check_j < check_y_end; check_j++)
            {
                for (int check_i = check_x_begin; check_i < check_x_end; check_i ++)
                {
                    float this_depth = depth.at<float>(check_j, check_i);
                    if(this_depth > 0.01)
                    {
                        this_sp.mean_depth = this_depth;
                        find_depth = true;
                        break;
                    }
                }
                if(find_depth)
                    break;
            }
        }
        superpixel_seeds[seed_i] = this_sp;
    }
}

void FusionFunctions::initialize_seeds()
{
    std::vector<std::thread> thread_pool;
    for (int i = 0; i < THREAD_NUM; i++)
    {
        std::thread this_thread(&FusionFunctions::initialize_seeds_kernel, this, i, THREAD_NUM);
        thread_pool.push_back(std::move(this_thread));
    }
    for (int i = 0; i < thread_pool.size(); i++)
        if (thread_pool[i].joinable())
            thread_pool[i].join();
}

void FusionFunctions::calculate_spaces_kernel(int thread_i, int thread_num)
{
    int step_row = image_height / thread_num;
    int start_row = step_row * thread_i;
    int end_row = start_row + step_row;
    if(thread_i == thread_num - 1)
        end_row = image_height;
    for(int row_i = start_row; row_i < end_row; row_i++)
    for(int col_i = 0; col_i < image_width; col_i++)
    {
        int my_index = row_i * image_width + col_i;
        float my_depth = depth.at<float>(row_i, col_i);
        double x,y,z;
        back_project(col_i, row_i, my_depth, x,y,z);
        space_map[my_index * 3] = x;
        space_map[my_index * 3 + 1] = y;
        space_map[my_index * 3 + 2] = z;
    }
}

void FusionFunctions::calculate_pixels_norms_kernel(int thread_i, int thread_num)
{
    int step_row = image_height / thread_num;
    int start_row = step_row * thread_i;
    start_row = start_row > 1 ? start_row : 1;
    int end_row = start_row + step_row;
    if(thread_i == thread_num - 1)
        end_row = image_height - 1;
    for(int row_i = start_row; row_i < end_row; row_i++)
    for(int col_i = 1; col_i < image_width - 1; col_i++)
    {
        int my_index = row_i * image_width + col_i;
        float my_x, my_y, my_z;
        my_x = space_map[my_index * 3];
        my_y = space_map[my_index * 3 + 1];
        my_z = space_map[my_index * 3 + 2];
        float right_x, right_y, right_z;
        right_x = space_map[my_index * 3 + 3];
        right_y = space_map[my_index * 3 + 4];
        right_z = space_map[my_index * 3 + 5];
        float down_x, down_y, down_z;
        down_x = space_map[my_index * 3 + image_width*3];
        down_y = space_map[my_index * 3 + image_width*3 + 1];
        down_z = space_map[my_index * 3 + image_width*3 + 2];
        if (my_z < 0.1 || right_z < 0.1 || down_z < 0.1)
            continue;
        right_x = right_x - my_x;
        right_y = right_y - my_y;
        right_z = right_z - my_z;
        down_x = down_x - my_x;
        down_y = down_y - my_y;
        down_z = down_z - my_z;
        float norm_x, norm_y, norm_z, norm_length;
        norm_x = right_y * down_z - right_z * down_y;
        norm_y = right_z * down_x - right_x * down_z;
        norm_z = right_x * down_y - right_y * down_x;
        norm_length = std::sqrt(norm_x * norm_x + norm_y * norm_y + norm_z * norm_z);
        norm_x /= norm_length;
        norm_y /= norm_length;
        norm_z /= norm_length;
        float view_angle = (norm_x * my_x + norm_y * my_y + norm_z * my_z)
            / std::sqrt(my_x*my_x+my_y*my_y+my_z*my_z);
        if(view_angle > -MAX_ANGLE_COS && view_angle < MAX_ANGLE_COS)
            continue;
        norm_map[my_index * 3] = norm_x;
        norm_map[my_index * 3 + 1] = norm_y;
        norm_map[my_index * 3 + 2] = norm_z;
    }
}

// void FusionFunctions::calculate_sp_norms_kernel(int thread_i, int thread_num)
// {
//     int step = superpixel_seeds.size() / thread_num;
//     int begin_index = step * thread_i;
//     int end_index = begin_index + step;
//     if (thread_i == thread_num - 1)
//         end_index = superpixel_seeds.size();
//     for (int seed_i = begin_index; seed_i < end_index; seed_i++)
//     {
//         int sp_x = seed_i % sp_width;
//         int sp_y = seed_i / sp_width;
//         int check_x_begin = sp_x * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
//         int check_y_begin = sp_y * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
//         double avg_x, avg_y, avg_z;
//         back_project(
//             superpixel_seeds[seed_i].x, superpixel_seeds[seed_i].y, superpixel_seeds[seed_i].mean_depth,
//             avg_x, avg_y, avg_z);
//         std::vector<double> space_points;
//         float max_dist = 0;
//         double sum_x, sum_y, sum_z;
//         int sum_num;
//         sum_x = 0;
//         sum_y = 0;
//         sum_z = 0;
//         sum_num = 0;
//         for (int check_j = check_y_begin; check_j < (check_y_begin + SP_SIZE * 2); check_j++)
//         {
//             for (int check_i = check_x_begin; check_i < (check_x_begin + SP_SIZE * 2); check_i++)
//             {
//                 int pixel_index = check_j * image_width + check_i;
//                 if (pixel_index < 0 || pixel_index >= superpixel_index.size())
//                     continue;
//                 if (superpixel_index[pixel_index] == seed_i)
//                 {
//                     if (space_map[pixel_index * 3 + 2] < 0.1)
//                         continue;
//                     float pixel_x = space_map[pixel_index * 3];
//                     float pixel_y = space_map[pixel_index * 3 + 1];
//                     float pixel_z = space_map[pixel_index * 3 + 2];
//                     // float dist_sp = (pixel_x - avg_x)*(pixel_x - avg_x) + (pixel_y - avg_y)*(pixel_y - avg_y) + (pixel_z - avg_z)*(pixel_z - avg_z);
//                     // if(dist_sp > 0.05*0.05)
//                     //     continue;
//                     space_points.push_back(pixel_x);
//                     space_points.push_back(pixel_y);
//                     space_points.push_back(pixel_z);
//                     sum_x += pixel_x;
//                     sum_y += pixel_y;
//                     sum_z += pixel_z;
//                     sum_num += 1;

//                     float x_diff = check_i - superpixel_seeds[seed_i].x;
//                     float y_diff = check_j - superpixel_seeds[seed_i].y;
//                     float dist = x_diff*x_diff + y_diff*y_diff;
//                     if (dist > max_dist)
//                         max_dist = dist;
//                 }
//             }
//         }
//         if (sum_num < 3)
//         {
//             superpixel_seeds[seed_i].mean_depth = 0.0;
//             continue;
//         }
//         superpixel_seeds[seed_i].size = std::sqrt(max_dist);
//         sum_x /= sum_num;
//         sum_y /= sum_num;
//         sum_z /= sum_num;
//         Eigen::Matrix3d covariance_m = Eigen::Matrix3d::Zero();
//         for (int i = 0; i < sum_num; i++)
//         {
//             double dx = space_points[i * 3] - sum_x;
//             double dy = space_points[i * 3 + 1] - sum_y;
//             double dz = space_points[i * 3 + 2] - sum_z;
//             covariance_m(0, 0) += dx * dx;
//             covariance_m(0, 1) += dx * dy;
//             covariance_m(0, 2) += dx * dz;
//             covariance_m(1, 0) += dy * dx;
//             covariance_m(1, 1) += dy * dy;
//             covariance_m(1, 2) += dy * dz;
//             covariance_m(2, 0) += dz * dx;
//             covariance_m(2, 1) += dz * dy;
//             covariance_m(2, 2) += dz * dz;
//         }
//         Eigen::EigenSolver<Eigen::Matrix3d> cov_eigen(covariance_m);
//         int min_ev_index = 0;
//         int mid_ev_index = 1;
//         int max_ev_index = 2;
//         double n_x, n_y, n_z;
//         if(cov_eigen.eigenvalues().real()(min_ev_index) > cov_eigen.eigenvalues().real()(mid_ev_index))
//             std::swap(min_ev_index, mid_ev_index);
//         if (cov_eigen.eigenvalues().real()(min_ev_index) > cov_eigen.eigenvalues().real()(max_ev_index))
//             std::swap(min_ev_index, max_ev_index);
//         if (cov_eigen.eigenvalues().real()(mid_ev_index) > cov_eigen.eigenvalues().real()(max_ev_index))
//             std::swap(mid_ev_index, max_ev_index);

//         n_x = cov_eigen.eigenvectors().real()(0, min_ev_index);
//         n_y = cov_eigen.eigenvectors().real()(1, min_ev_index);
//         n_z = cov_eigen.eigenvectors().real()(2, min_ev_index);

//         if (cov_eigen.eigenvalues().real()(max_ev_index) / cov_eigen.eigenvalues().real()(min_ev_index) < 3 ||
//             cov_eigen.eigenvalues().real()(mid_ev_index) / cov_eigen.eigenvalues().real()(min_ev_index) < 3)
//             continue;

//         // flip the dir of the norm if needed
//         float view_cos = -1.0 * (n_x * sum_x + n_y * sum_y + n_z * sum_z) / std::sqrt(sum_x * sum_x + sum_y * sum_y + sum_z * sum_z);
//         if (view_cos < 0)
//         {
//             view_cos *= -1.0;
//             n_x *= -1.0;
//             n_y *= -1.0;
//             n_z *= -1.0;
//         }
//         superpixel_seeds[seed_i].norm_x = n_x;
//         superpixel_seeds[seed_i].norm_y = n_y;
//         superpixel_seeds[seed_i].norm_z = n_z;
//         superpixel_seeds[seed_i].posi_x = sum_x;
//         superpixel_seeds[seed_i].posi_y = sum_y;
//         superpixel_seeds[seed_i].posi_z = sum_z;
//         superpixel_seeds[seed_i].mean_depth = sum_z;
//         superpixel_seeds[seed_i].view_cos = view_cos;
//     }
// }

void FusionFunctions::calculate_sp_norms_kernel(int thread_i, int thread_num)
{
    int step = superpixel_seeds.size() / thread_num;
    int begin_index = step * thread_i;
    int end_index = begin_index + step;
    if (thread_i == thread_num - 1)
        end_index = superpixel_seeds.size();
    for (int seed_i = begin_index; seed_i < end_index; seed_i++)
    {
        int sp_x = seed_i % sp_width;
        int sp_y = seed_i / sp_width;
        int check_x_begin = sp_x * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
        int check_y_begin = sp_y * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
        double avg_x, avg_y, avg_z;
        back_project(
            superpixel_seeds[seed_i].x, superpixel_seeds[seed_i].y, superpixel_seeds[seed_i].mean_depth,
            avg_x, avg_y, avg_z);
        float max_dist = 0;
        float sum_norm_x, sum_norm_y, sum_norm_z, sum_num;
        sum_norm_x = 0;
        sum_norm_y = 0;
        sum_norm_z = 0;
        sum_num = 0;
        for (int check_j = check_y_begin; check_j < (check_y_begin + SP_SIZE * 2); check_j++)
        {
            for (int check_i = check_x_begin; check_i < (check_x_begin + SP_SIZE * 2); check_i++)
            {
                int pixel_index = check_j * image_width + check_i;
                if (pixel_index < 0 || pixel_index >= superpixel_index.size())
                    continue;
                if (superpixel_index[pixel_index] == seed_i)
                {
                    if (space_map[pixel_index * 3 + 2] < 0.1)
                        continue;
                    sum_norm_x += norm_map[pixel_index * 3];
                    sum_norm_y += norm_map[pixel_index * 3 + 1];
                    sum_norm_z += norm_map[pixel_index * 3 + 2];
                    sum_num += 1.0;

                    float x_diff = check_i - superpixel_seeds[seed_i].x;
                    float y_diff = check_j - superpixel_seeds[seed_i].y;
                    float dist = x_diff * x_diff + y_diff * y_diff;
                    if (dist > max_dist)
                        max_dist = dist;
                }
            }
        }
        if (sum_num < 3)
            continue;
        superpixel_seeds[seed_i].size = std::sqrt(max_dist);
        float norm_length, n_x, n_y, n_z;
        norm_length = std::sqrt(sum_norm_x * sum_norm_x + sum_norm_y + sum_norm_y + sum_norm_z * sum_norm_z);
        n_x = sum_norm_x / norm_length;
        n_y = sum_norm_y / norm_length;
        n_z = sum_norm_z / norm_length;

        // flip the dir of the norm if needed
        float view_cos = -1.0 * (n_x * avg_x + n_y * avg_y + n_z * avg_z) / std::sqrt(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z);
        if (view_cos < 0)
        {
            view_cos *= -1.0;
            n_x *= -1.0;
            n_y *= -1.0;
            n_z *= -1.0;
        }
        superpixel_seeds[seed_i].norm_x = n_x;
        superpixel_seeds[seed_i].norm_y = n_y;
        superpixel_seeds[seed_i].norm_z = n_z;
        superpixel_seeds[seed_i].posi_x = avg_x;
        superpixel_seeds[seed_i].posi_y = avg_y;
        superpixel_seeds[seed_i].posi_z = avg_z;
        superpixel_seeds[seed_i].mean_depth = avg_z;
        superpixel_seeds[seed_i].view_cos = view_cos;
    }
}

// void FusionFunctions::calculate_sp_depth_norms_kernel(int thread_i, int thread_num)
// {
//     int step = superpixel_seeds.size() / thread_num;
//     int begin_index = step * thread_i;
//     int end_index = begin_index + step;
//     if (thread_i == thread_num - 1)
//         end_index = superpixel_seeds.size();
//     for (int seed_i = begin_index; seed_i < end_index; seed_i++)
//     {
//         int sp_x = seed_i % sp_width;
//         int sp_y = seed_i / sp_width;
//         int check_x_begin = sp_x * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
//         int check_y_begin = sp_y * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
//         std::vector<float> pixel_depth;
//         std::vector<float> pixel_norms;
//         std::vector<float> pixel_positions;
//         std::vector<float> pixel_inlier_positions;
//         float sum_a, sum_b;
//         sum_a = sum_b = 0;
//         float max_dist = 0;
//         for (int check_j = check_y_begin; check_j < (check_y_begin + SP_SIZE * 2); check_j++)
//         {
//         for (int check_i = check_x_begin; check_i < (check_x_begin + SP_SIZE * 2); check_i++)
//         {
//             int pixel_index = check_j * image_width + check_i;
//             if (pixel_index < 0 || pixel_index >= superpixel_index.size())
//                 continue;
//             if (superpixel_index[pixel_index] == seed_i)
//             {
//                 float x_diff = check_i - superpixel_seeds[seed_i].x;
//                 float y_diff = check_j - superpixel_seeds[seed_i].y;
//                 float dist = x_diff * x_diff + y_diff * y_diff;
//                 if (dist > max_dist)
//                     max_dist = dist;

//                 float my_depth = depth.at<float>(check_j, check_i);
//                 if (my_depth > 0.05)
//                 {
//                     pixel_depth.push_back(my_depth);
//                     pixel_norms.push_back(norm_map[pixel_index * 3]);
//                     pixel_norms.push_back(norm_map[pixel_index * 3 + 1]);
//                     pixel_norms.push_back(norm_map[pixel_index * 3 + 2]);
//                     sum_a += my_depth;
//                     sum_b += 1;
//                     pixel_positions.push_back(space_map[pixel_index * 3]);
//                     pixel_positions.push_back(space_map[pixel_index * 3 + 1]);
//                     pixel_positions.push_back(space_map[pixel_index * 3 + 2]);
//                 }
//             }
//         }
//         }
//         if(sum_b < 16)
//             continue;
//         float mean_depth = sum_a / sum_b;
//         // std::cout << "initial depth " << mean_depth << std::endl;
//         for (int newton_i = 0; newton_i < 5; newton_i++)
//         {
//             sum_a = sum_b = 0;
//             for(int p_i = 0; p_i < pixel_depth.size(); p_i++)
//             {
//                 float residual = mean_depth - pixel_depth[p_i];
//                 if (residual < HUBER_RANGE && residual > -HUBER_RANGE)
//                 {
//                     sum_a += 2 * residual;
//                     sum_b += 2;
//                 }
//                 else
//                 {
//                     sum_a += residual > 0 ? HUBER_RANGE : -1 * HUBER_RANGE;
//                     // sum_b += 0;
//                 }
//             }
//             mean_depth = mean_depth - sum_a / (sum_b + 10.0);
//             // std::cout << "iter " << newton_i << " a " << sum_a << " b " << sum_b << " depth " << mean_depth << std::endl;
//         }
//         float norm_x, norm_y, norm_z, norm_b;
//         norm_x = norm_y = norm_z = norm_b = 0.0;
//         float inlier_num = 0;
//         for (int p_i = 0; p_i < pixel_depth.size(); p_i++)
//         {
//             float residual = mean_depth - pixel_depth[p_i];
//             if (residual < HUBER_RANGE && residual > -HUBER_RANGE)
//             {
//                 norm_x += pixel_norms[p_i * 3];
//                 norm_y += pixel_norms[p_i * 3 + 1];
//                 norm_z += pixel_norms[p_i * 3 + 2];
//                 inlier_num += 1;
//                 // for test huber norm
//                 pixel_inlier_positions.push_back(pixel_positions[p_i * 3]);
//                 pixel_inlier_positions.push_back(pixel_positions[p_i * 3 + 1]);
//                 pixel_inlier_positions.push_back(pixel_positions[p_i * 3 + 2]);
//             }
//         }
//         if(inlier_num/pixel_depth.size() < 0.8)
//             continue;
//         float norm_length = std::sqrt(norm_x * norm_x + norm_y * norm_y + norm_z * norm_z);
//         if (norm_length != norm_length)
//         {
//             std::cout << norm_x << " " << norm_y << " " << norm_z << " " << norm_length << std::endl;
//         }
//         norm_x = norm_x / norm_length;
//         norm_y = norm_y / norm_length;
//         norm_z = norm_z / norm_length;
//         {
//             // robust norm
//             float gn_nx = norm_x;
//             float gn_ny = norm_y;
//             float gn_nz = norm_z;
//             float gn_nb = norm_b;
//             get_huber_norm(gn_nx, gn_ny, gn_nz, gn_nb, pixel_inlier_positions);
//             norm_x = gn_nx;
//             norm_y = gn_ny;
//             norm_z = gn_nz;
//             norm_b = gn_nb;
//         }
//         double avg_x, avg_y, avg_z;
//         back_project(
//             superpixel_seeds[seed_i].x, superpixel_seeds[seed_i].y, mean_depth,
//             avg_x, avg_y, avg_z);
//         {
//             // make sure the avg_x, avg_y, and avg_z are one the surfel
//             float k = -1 * (avg_x * norm_x + avg_y * norm_y + avg_z * norm_z) - norm_b;
//             avg_x += k * norm_x;
//             avg_y += k * norm_y;
//             avg_z += k * norm_z;
//             mean_depth = avg_z;
//         }
//         float view_cos = -1.0 * (norm_x * avg_x + norm_y * avg_y + norm_z * avg_z) / std::sqrt(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z);
//         if (view_cos < 0)
//         {
//             view_cos *= -1.0;
//             norm_x *= -1.0;
//             norm_y *= -1.0;
//             norm_z *= -1.0;
//         }
//         superpixel_seeds[seed_i].norm_x = norm_x;
//         superpixel_seeds[seed_i].norm_y = norm_y;
//         superpixel_seeds[seed_i].norm_z = norm_z;
//         superpixel_seeds[seed_i].posi_x = avg_x;
//         superpixel_seeds[seed_i].posi_y = avg_y;
//         superpixel_seeds[seed_i].posi_z = avg_z;
//         superpixel_seeds[seed_i].mean_depth = mean_depth;
//         superpixel_seeds[seed_i].view_cos = view_cos;
//         superpixel_seeds[seed_i].size = std::sqrt(max_dist);
//     }
// }

void FusionFunctions::calculate_sp_depth_norms_kernel(int thread_i, int thread_num)
{
    int step = superpixel_seeds.size() / thread_num;
    int begin_index = step * thread_i;
    int end_index = begin_index + step;
    if (thread_i == thread_num - 1)
        end_index = superpixel_seeds.size();
    for (int seed_i = begin_index; seed_i < end_index; seed_i++)
    {
        int sp_x = seed_i % sp_width;
        int sp_y = seed_i / sp_width;
        int check_x_begin = sp_x * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
        int check_y_begin = sp_y * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
        std::vector<float> pixel_depth;
        std::vector<float> pixel_norms;
        std::vector<float> pixel_positions;
        std::vector<float> pixel_inlier_positions;
        float valid_depth_num = 0;
        float max_dist = 0;
        for (int check_j = check_y_begin; check_j < (check_y_begin + SP_SIZE * 2); check_j++)
        {
            for (int check_i = check_x_begin; check_i < (check_x_begin + SP_SIZE * 2); check_i++)
            {
                int pixel_index = check_j * image_width + check_i;
                if (pixel_index < 0 || pixel_index >= superpixel_index.size())
                    continue;
                if (superpixel_index[pixel_index] == seed_i)
                {
                    float x_diff = check_i - superpixel_seeds[seed_i].x;
                    float y_diff = check_j - superpixel_seeds[seed_i].y;
                    float dist = x_diff * x_diff + y_diff * y_diff;
                    if (dist > max_dist)
                        max_dist = dist;

                    float my_depth = depth.at<float>(check_j, check_i);
                    if (my_depth > 0.05)
                    {
                        pixel_depth.push_back(my_depth);
                        pixel_norms.push_back(norm_map[pixel_index * 3]);
                        pixel_norms.push_back(norm_map[pixel_index * 3 + 1]);
                        pixel_norms.push_back(norm_map[pixel_index * 3 + 2]);
                        valid_depth_num += 1;
                        pixel_positions.push_back(space_map[pixel_index * 3]);
                        pixel_positions.push_back(space_map[pixel_index * 3 + 1]);
                        pixel_positions.push_back(space_map[pixel_index * 3 + 2]);
                    }
                }
            }
        }
        if (valid_depth_num < 16)
            continue;
        float mean_depth = superpixel_seeds[seed_i].mean_depth;
        float norm_x, norm_y, norm_z, norm_b;
        norm_x = norm_y = norm_z = norm_b = 0.0;
        float inlier_num = 0;
        for (int p_i = 0; p_i < pixel_depth.size(); p_i++)
        {
            float residual = mean_depth - pixel_depth[p_i];
            if (residual < HUBER_RANGE && residual > -HUBER_RANGE)
            {
                norm_x += pixel_norms[p_i * 3];
                norm_y += pixel_norms[p_i * 3 + 1];
                norm_z += pixel_norms[p_i * 3 + 2];
                inlier_num += 1;
                // for test huber norm
                pixel_inlier_positions.push_back(pixel_positions[p_i * 3]);
                pixel_inlier_positions.push_back(pixel_positions[p_i * 3 + 1]);
                pixel_inlier_positions.push_back(pixel_positions[p_i * 3 + 2]);
            }
        }
        if (inlier_num / pixel_depth.size() < 0.8)
            continue;
        float norm_length = std::sqrt(norm_x * norm_x + norm_y * norm_y + norm_z * norm_z);
        if (norm_length != norm_length)
        {
            std::cout << norm_x << " " << norm_y << " " << norm_z << " " << norm_length << std::endl;
        }
        norm_x = norm_x / norm_length;
        norm_y = norm_y / norm_length;
        norm_z = norm_z / norm_length;
        {
            // robust norm
            float gn_nx = norm_x;
            float gn_ny = norm_y;
            float gn_nz = norm_z;
            float gn_nb = norm_b;
            get_huber_norm(gn_nx, gn_ny, gn_nz, gn_nb, pixel_inlier_positions);
            norm_x = gn_nx;
            norm_y = gn_ny;
            norm_z = gn_nz;
            norm_b = gn_nb;
        }
        double avg_x, avg_y, avg_z;
        back_project(
            superpixel_seeds[seed_i].x, superpixel_seeds[seed_i].y, mean_depth,
            avg_x, avg_y, avg_z);
        {
            // make sure the avg_x, avg_y, and avg_z are one the surfel
            float k = -1 * (avg_x * norm_x + avg_y * norm_y + avg_z * norm_z) - norm_b;
            avg_x += k * norm_x;
            avg_y += k * norm_y;
            avg_z += k * norm_z;
            mean_depth = avg_z;
        }
        float view_cos = -1.0 * (norm_x * avg_x + norm_y * avg_y + norm_z * avg_z) / std::sqrt(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z);
        if (view_cos < 0)
        {
            view_cos *= -1.0;
            norm_x *= -1.0;
            norm_y *= -1.0;
            norm_z *= -1.0;
        }
        superpixel_seeds[seed_i].norm_x = norm_x;
        superpixel_seeds[seed_i].norm_y = norm_y;
        superpixel_seeds[seed_i].norm_z = norm_z;
        superpixel_seeds[seed_i].posi_x = avg_x;
        superpixel_seeds[seed_i].posi_y = avg_y;
        superpixel_seeds[seed_i].posi_z = avg_z;
        superpixel_seeds[seed_i].mean_depth = mean_depth;
        superpixel_seeds[seed_i].view_cos = view_cos;
        superpixel_seeds[seed_i].size = std::sqrt(max_dist);
    }
}

// void FusionFunctions::calculate_sp_depth_norms_kernel(int thread_i, int thread_num)
// {
//     int step = superpixel_seeds.size() / thread_num;
//     int begin_index = step * thread_i;
//     int end_index = begin_index + step;
//     if (thread_i == thread_num - 1)
//         end_index = superpixel_seeds.size();
//     for (int seed_i = begin_index; seed_i < end_index; seed_i++)
//     {
//         int sp_x = seed_i % sp_width;
//         int sp_y = seed_i / sp_width;
//         int check_x_begin = sp_x * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
//         int check_y_begin = sp_y * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
//         std::vector<float> pixel_depth;
//         std::vector<float> pixel_norms;
//         std::vector<float> pixel_positions;
//         float sum_a, sum_b;
//         sum_a = sum_b = 0;
//         float max_dist = 0;
//         for (int check_j = check_y_begin; check_j < (check_y_begin + SP_SIZE * 2); check_j++)
//         {
//             for (int check_i = check_x_begin; check_i < (check_x_begin + SP_SIZE * 2); check_i++)
//             {
//                 int pixel_index = check_j * image_width + check_i;
//                 if (pixel_index < 0 || pixel_index >= superpixel_index.size())
//                     continue;
//                 if (superpixel_index[pixel_index] == seed_i)
//                 {
//                     float x_diff = check_i - superpixel_seeds[seed_i].x;
//                     float y_diff = check_j - superpixel_seeds[seed_i].y;
//                     float dist = x_diff * x_diff + y_diff * y_diff;
//                     if (dist > max_dist)
//                         max_dist = dist;

//                     float my_depth = depth.at<float>(check_j, check_i);
//                     if (my_depth > 0.05)
//                     {
//                         pixel_depth.push_back(my_depth);
//                         pixel_norms.push_back(norm_map[pixel_index * 3]);
//                         pixel_norms.push_back(norm_map[pixel_index * 3 + 1]);
//                         pixel_norms.push_back(norm_map[pixel_index * 3 + 2]);
//                         sum_a += my_depth;
//                         sum_b += 1;
//                         pixel_positions.push_back(space_map[pixel_index * 3]);
//                         pixel_positions.push_back(space_map[pixel_index * 3 + 1]);
//                         pixel_positions.push_back(space_map[pixel_index * 3 + 2]);
//                     }
//                 }
//             }
//         }
//         if (sum_b < 16)
//             continue;
//         float mean_depth = sum_a / sum_b;
//         float norm_x, norm_y, norm_z;
//         norm_x = norm_y = norm_z = 0.0;
//         for (int p_i = 0; p_i < pixel_depth.size(); p_i++)
//         {
//             norm_x += pixel_norms[p_i * 3];
//             norm_y += pixel_norms[p_i * 3 + 1];
//             norm_z += pixel_norms[p_i * 3 + 2];
//         }
//         float norm_length = std::sqrt(norm_x * norm_x + norm_y * norm_y + norm_z * norm_z);
//         if (norm_length != norm_length)
//         {
//             std::cout << norm_x << " " << norm_y << " " << norm_z << " " << norm_length << std::endl;
//         }
//         norm_x = norm_x / norm_length;
//         norm_y = norm_y / norm_length;
//         norm_z = norm_z / norm_length;
//         {
//             // robust norm
//             float gn_nx = norm_x;
//             float gn_ny = norm_y;
//             float gn_nz = norm_z;
//             float gn_nb = 0;
//             get_huber_norm(gn_nx, gn_ny, gn_nz, gn_nb, pixel_positions);
//             norm_x = gn_nx;
//             norm_y = gn_ny;
//             norm_z = gn_nz;
//         }
//         double avg_x, avg_y, avg_z;
//         back_project(
//             superpixel_seeds[seed_i].x, superpixel_seeds[seed_i].y, mean_depth,
//             avg_x, avg_y, avg_z);
//         float view_cos = -1.0 * (norm_x * avg_x + norm_y * avg_y + norm_z * avg_z) / std::sqrt(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z);
//         if (view_cos < 0)
//         {
//             view_cos *= -1.0;
//             norm_x *= -1.0;
//             norm_y *= -1.0;
//             norm_z *= -1.0;
//         }
//         superpixel_seeds[seed_i].norm_x = norm_x;
//         superpixel_seeds[seed_i].norm_y = norm_y;
//         superpixel_seeds[seed_i].norm_z = norm_z;
//         superpixel_seeds[seed_i].posi_x = avg_x;
//         superpixel_seeds[seed_i].posi_y = avg_y;
//         superpixel_seeds[seed_i].posi_z = avg_z;
//         superpixel_seeds[seed_i].mean_depth = mean_depth;
//         superpixel_seeds[seed_i].view_cos = view_cos;
//         superpixel_seeds[seed_i].size = std::sqrt(max_dist);
//     }
// }

// void FusionFunctions::calculate_sp_depth_norms_kernel(int thread_i, int thread_num)
// {
//     int step = superpixel_seeds.size() / thread_num;
//     int begin_index = step * thread_i;
//     int end_index = begin_index + step;
//     if (thread_i == thread_num - 1)
//         end_index = superpixel_seeds.size();
//     for (int seed_i = begin_index; seed_i < end_index; seed_i++)
//     {
//         int sp_x = seed_i % sp_width;
//         int sp_y = seed_i / sp_width;
//         int check_x_begin = sp_x * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
//         int check_y_begin = sp_y * SP_SIZE + SP_SIZE / 2 - SP_SIZE;
//         std::vector<float> pixel_depth;
//         std::vector<int> pixel_indexs;
//         float sum_a, sum_b;
//         sum_a = sum_b = 0;
//         float max_dist = 0;
//         for (int check_j = check_y_begin; check_j < (check_y_begin + SP_SIZE * 2); check_j++)
//         {
//             for (int check_i = check_x_begin; check_i < (check_x_begin + SP_SIZE * 2); check_i++)
//             {
//                 int pixel_index = check_j * image_width + check_i;
//                 if (pixel_index < 0 || pixel_index >= superpixel_index.size())
//                     continue;
//                 if (superpixel_index[pixel_index] == seed_i)
//                 {
//                     float x_diff = check_i - superpixel_seeds[seed_i].x;
//                     float y_diff = check_j - superpixel_seeds[seed_i].y;
//                     float dist = x_diff * x_diff + y_diff * y_diff;
//                     if (dist > max_dist)
//                         max_dist = dist;

//                     float my_depth = depth.at<float>(check_j, check_i);
//                     if (my_depth > 0.05)
//                     {
//                         pixel_depth.push_back(my_depth);
//                         pixel_indexs.push_back(pixel_index);
//                         sum_a += my_depth;
//                         sum_b += 1;
//                     }
//                 }
//             }
//         }
//         if (sum_b < 32)
//             continue;
//         float mean_depth = sum_a / sum_b;
//         // std::cout << "initial depth " << mean_depth << std::endl;
//         for (int newton_i = 0; newton_i < 5; newton_i++)
//         {
//             sum_a = sum_b = 0;
//             for (int p_i = 0; p_i < pixel_depth.size(); p_i++)
//             {
//                 float residual = mean_depth - pixel_depth[p_i];
//                 if (residual < HUBER_RANGE && residual > -HUBER_RANGE)
//                 {
//                     sum_a += 2 * residual;
//                     sum_b += 2;
//                 }
//                 else
//                 {
//                     sum_a += residual > 0 ? HUBER_RANGE : -1 * HUBER_RANGE;
//                     // sum_b += 0;
//                 }
//             }
//             mean_depth = mean_depth - sum_a / (sum_b + 10.0);
//             // std::cout << "iter " << newton_i << " a " << sum_a << " b " << sum_b << " depth " << mean_depth << std::endl;
//         }
//         float sum_x, sum_y, sum_z;
//         sum_x = sum_y = sum_z = 0.0;
//         std::vector<float> pixel_positions;
//         float inlier_num = 0;
//         for (int p_i = 0; p_i < pixel_depth.size(); p_i++)
//         {
//             float residual = mean_depth - pixel_depth[p_i];
//             if (residual < HUBER_RANGE && residual > -HUBER_RANGE)
//             {
//                 int inlier_index = pixel_indexs[p_i];
//                 sum_x += space_map[inlier_index*3];
//                 sum_y += space_map[inlier_index*3 + 1];
//                 sum_z += space_map[inlier_index*3 + 2];
//                 inlier_num += 1;
//                 pixel_positions.push_back(space_map[inlier_index * 3]);
//                 pixel_positions.push_back(space_map[inlier_index * 3 + 1]);
//                 pixel_positions.push_back(space_map[inlier_index * 3 + 2]);
//             }
//         }
//         if (inlier_num / pixel_depth.size() < 0.9)
//             continue;
//         sum_x /= inlier_num;
//         sum_y /= inlier_num;
//         sum_z /= inlier_num;

//         Eigen::Matrix3d covariance_m = Eigen::Matrix3d::Zero();
//         for (int i = 0; i < inlier_num; i++)
//         {
//             double dx = pixel_positions[i * 3] - sum_x;
//             double dy = pixel_positions[i * 3 + 1] - sum_y;
//             double dz = pixel_positions[i * 3 + 2] - sum_z;
//             covariance_m(0, 0) += dx * dx;
//             covariance_m(0, 1) += dx * dy;
//             covariance_m(0, 2) += dx * dz;
//             covariance_m(1, 0) += dy * dx;
//             covariance_m(1, 1) += dy * dy;
//             covariance_m(1, 2) += dy * dz;
//             covariance_m(2, 0) += dz * dx;
//             covariance_m(2, 1) += dz * dy;
//             covariance_m(2, 2) += dz * dz;
//         }
//         Eigen::EigenSolver<Eigen::Matrix3d> cov_eigen(covariance_m);
//         float min_eigen_value = 1e9;
//         int min_eigen_index;
//         float norm_x, norm_y, norm_z;
//         for(int i = 0; i < 3; i++)
//         {
//             if(cov_eigen.eigenvalues().real()(i) < min_eigen_value)
//             {
//                 min_eigen_value = cov_eigen.eigenvalues().real()(i);
//                 min_eigen_index = i;
//             }
//         }
//         norm_x = cov_eigen.eigenvectors().real()(0, min_eigen_index);
//         norm_y = cov_eigen.eigenvectors().real()(1, min_eigen_index);
//         norm_z = cov_eigen.eigenvectors().real()(2, min_eigen_index);

//         double avg_x, avg_y, avg_z;
//         // back_project(
//         //     superpixel_seeds[seed_i].x, superpixel_seeds[seed_i].y, mean_depth,
//         //     avg_x, avg_y, avg_z);
//         avg_x = sum_x;
//         avg_y = sum_y;
//         avg_z = sum_z;
//         float view_cos = -1.0 * (norm_x * avg_x + norm_y * avg_y + norm_z * avg_z) / std::sqrt(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z);
//         if (view_cos < 0)
//         {
//             view_cos *= -1.0;
//             norm_x *= -1.0;
//             norm_y *= -1.0;
//             norm_z *= -1.0;
//         }
//         superpixel_seeds[seed_i].norm_x = norm_x;
//         superpixel_seeds[seed_i].norm_y = norm_y;
//         superpixel_seeds[seed_i].norm_z = norm_z;
//         superpixel_seeds[seed_i].posi_x = avg_x;
//         superpixel_seeds[seed_i].posi_y = avg_y;
//         superpixel_seeds[seed_i].posi_z = avg_z;
//         superpixel_seeds[seed_i].mean_depth = mean_depth;
//         superpixel_seeds[seed_i].view_cos = view_cos;
//         superpixel_seeds[seed_i].size = std::sqrt(max_dist);
//     }
// }

void FusionFunctions::calculate_norms()
{
    std::vector<std::thread> thread_pool;
    for (int i = 0; i < THREAD_NUM; i++)
    {
        std::thread this_thread(&FusionFunctions::calculate_spaces_kernel, this, i, THREAD_NUM);
        thread_pool.push_back(std::move(this_thread));
    }
    for (int i = 0; i < thread_pool.size(); i++)
        if (thread_pool[i].joinable())
            thread_pool[i].join();
    thread_pool.clear();

    for (int i = 0; i < THREAD_NUM; i++)
    {
        std::thread this_thread(&FusionFunctions::calculate_pixels_norms_kernel, this, i, THREAD_NUM);
        thread_pool.push_back(std::move(this_thread));
    }
    for (int i = 0; i < thread_pool.size(); i++)
        if (thread_pool[i].joinable())
            thread_pool[i].join();
    thread_pool.clear();

    // for (int i = 0; i < THREAD_NUM; i++)
    // {
    //     std::thread this_thread(&FusionFunctions::calculate_sp_norms_kernel, this, i, THREAD_NUM);
    //     thread_pool.push_back(std::move(this_thread));
    // }
    // for (int i = 0; i < thread_pool.size(); i++)
    //     if (thread_pool[i].joinable())
    //         thread_pool[i].join();
    // thread_pool.clear();

    for (int i = 0; i < THREAD_NUM; i++)
    {
        std::thread this_thread(&FusionFunctions::calculate_sp_depth_norms_kernel, this, i, THREAD_NUM);
        thread_pool.push_back(std::move(this_thread));
    }
    for (int i = 0; i < thread_pool.size(); i++)
        if (thread_pool[i].joinable())
            thread_pool[i].join();
    thread_pool.clear();
}

void FusionFunctions::generate_super_pixels()
{
    // std::fill(superpixel_seeds.begin(), superpixel_seeds.end(), 0);
    memset(superpixel_seeds.data(), 0, superpixel_seeds.size() * sizeof(Superpixel_seed));
    std::fill(superpixel_index.begin(), superpixel_index.end(), 0);
    std::fill(norm_map.begin(), norm_map.end(), 0);
    initialize_seeds();

    for (int it_i = 0; it_i < ITERATION_NUM; it_i++)
    {
        update_pixels();
        update_seeds();
    }
    calculate_norms();
    debug_show();
}

void FusionFunctions::debug_show()
{
    cv::Mat result = cv::Mat(image_height, image_width, CV_8UC3);
    for (int j = 0; j < image_height; j++)
        for (int i = 0; i < image_width; i++)
        {
            // pixel sp index
            int sp_index = superpixel_index[j * image_width + i];
            cv::Vec3b this_norm;
            this_norm[0] = fabs(superpixel_seeds[sp_index].norm_x) * 255;
            this_norm[1] = fabs(superpixel_seeds[sp_index].norm_y) * 255;
            this_norm[2] = fabs(superpixel_seeds[sp_index].norm_z) * 255;
            result.at<cv::Vec3b>(j, i) = this_norm;
        }
    for (int i = 0; i < superpixel_index.size(); i++)
    {
        int p_x = i % image_width;
        int p_y = i / image_width;
        int my_index = superpixel_index[i];
        if (p_x + 1 < image_width && superpixel_index[i + 1] != my_index)
            result.at<cv::Vec3b>(p_y, p_x) = cv::Vec3b(0, 0, 0);
        if (p_y + 1 < image_height && superpixel_index[i + image_width] != my_index)
            result.at<cv::Vec3b>(p_y, p_x) = cv::Vec3b(0, 0, 0);
    }
    cv::imshow("superpixel norm", result);
    // cv::imwrite("/home/wang/average_norm.png", result);
    // cv::imwrite("/home/wang/huber_norm.png", result);
    // cv::imshow("input image", image);
    cv::waitKey(5);
}
