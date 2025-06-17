/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file CollisionPrevention.cpp
 * CollisionPrevention controller.
 *
 */

#include "CollisionPrevention.hpp"

#include <px4_platform_common/events.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_vect.h>
// #include <vector>
// #include <algorithm>

using namespace matrix;
using namespace time_literals;

namespace
{
// lc add
// v2 三维合并 18*6=108
static constexpr int INTERNAL_MAP_INCRE_DEG_HOR = 20;
static constexpr int INTERNAL_MAP_INCRE_DEG_VER = 30;
static constexpr int INTERNAL_MAP_USED_BINS_HOR = 360 / INTERNAL_MAP_INCRE_DEG_HOR;
static constexpr int INTERNAL_MAP_USED_BINS_VER = 180 / INTERNAL_MAP_INCRE_DEG_VER;

static float wrap(float f, float min, float max)
{
    if (f < min)
    {
        return max + (f - min);
    }

    if (f > max)
    {
        return min + (f - max);
    }

    return f;
}

static float wrap_360(float f) { return wrap(f, 0.f, 360.f); }

static int wrap_bin(int i)
{
    i = i % INTERNAL_MAP_USED_BINS_HOR;

    while (i < 0)
    {
        i += INTERNAL_MAP_USED_BINS_HOR;
    }

    return i;
}

}  // namespace

CollisionPrevention::CollisionPrevention(ModuleParams *parent) : ModuleParams(parent)
{
    static_assert(INTERNAL_MAP_INCRE_DEG_HOR >= 5,
                  "INTERNAL_MAP_INCRE_DEG_HOR needs to be at least 5");
    static_assert(360 % INTERNAL_MAP_INCRE_DEG_HOR == 0,
                  "INTERNAL_MAP_INCRE_DEG_HOR should divide 360 evenly");
    static_assert(INTERNAL_MAP_INCRE_DEG_VER >= 5,
                  "INTERNAL_MAP_INCRE_DEG_VER needs to be at least 5");
    static_assert(360 % INTERNAL_MAP_INCRE_DEG_VER == 0,
                  "INTERNAL_MAP_INCRE_DEG_VER should divide 360 evenly");


    // initialize internal obstacle map
    _obstacle_map_body_frame.timestamp = getTime();
    _obstacle_map_body_frame.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
    _obstacle_map_body_frame.increment = INTERNAL_MAP_INCRE_DEG_HOR;
    _obstacle_map_body_frame.min_distance = UINT16_MAX;
    _obstacle_map_body_frame.max_distance = 0;
    _obstacle_map_body_frame.angle_offset = 0.f;
    uint32_t internal_bins =
        sizeof(_obstacle_map_body_frame.distances) / sizeof(_obstacle_map_body_frame.distances[0]);
    uint64_t current_time = getTime();

    for (uint32_t i = 0; i < internal_bins; i++)
    {
        _data_timestamps[i] = current_time;
        _data_maxranges[i] = 0;
        _data_fov[i] = 0;
        _obstacle_map_body_frame.distances[i] = UINT16_MAX;
    }
}

hrt_abstime CollisionPrevention::getTime() { return hrt_absolute_time(); }

hrt_abstime CollisionPrevention::getElapsedTime(const hrt_abstime *ptr)
{
    return hrt_absolute_time() - *ptr;
}

bool CollisionPrevention::is_active()
{
    bool activated = _param_cp_dist.get() > 0;

    if (activated && !_was_active)
    {
        _time_activated = getTime();
    }

    _was_active = activated;
    return activated;
}

void CollisionPrevention::_addObstacleSensorData(const obstacle_distance_s &obstacle,
                                                 const matrix::Quatf &vehicle_attitude)
{
    int msg_index = 0;
    float vehicle_orientation_deg = math::degrees(Eulerf(vehicle_attitude).psi());
    float increment_factor = 1.f / obstacle.increment;

    if (obstacle.frame == obstacle.MAV_FRAME_GLOBAL ||
        obstacle.frame == obstacle.MAV_FRAME_LOCAL_NED)
    {
        // Obstacle message arrives in local_origin frame (north aligned)
        // corresponding data index (convert to world frame and shift by msg offset)
        for (int e = 0; e < INTERNAL_MAP_USED_BINS_VER; e++){
            for (int z = 0; z < INTERNAL_MAP_USED_BINS_HOR; z++)
            {
                float bin_angle_deg = (float)z * INTERNAL_MAP_INCRE_DEG_HOR + _obstacle_map_body_frame.angle_offset;
                msg_index = floor(wrap_360(vehicle_orientation_deg + bin_angle_deg - obstacle.angle_offset) * increment_factor);
                // lc add 确保数组不越界
                //msg_index %= INTERNAL_MAP_USED_BINS_HOR;

                msg_index += e * INTERNAL_MAP_USED_BINS_HOR;

                // add all data points inside to FOV
                if (obstacle.distances[msg_index] != UINT16_MAX)
                {
                    if (_enterData(e * INTERNAL_MAP_USED_BINS_HOR + z, obstacle.max_distance * 0.01f,
                                obstacle.distances[msg_index] * 0.01f))
                    {
                        _obstacle_map_body_frame.distances[e * INTERNAL_MAP_USED_BINS_HOR + z] = obstacle.distances[msg_index];
                        _data_timestamps[e * INTERNAL_MAP_USED_BINS_HOR + z] = _obstacle_map_body_frame.timestamp;
                        _data_maxranges[e * INTERNAL_MAP_USED_BINS_HOR + z] = obstacle.max_distance;
                        _data_fov[e * INTERNAL_MAP_USED_BINS_HOR + z] = 1;
                    }
                }
            }
        }

        // lc add
        // // todo transform to body frame
        // // 前6位为下方数据 后6位为上方数据
        // for (int i = INTERNAL_MAP_USED_BINS; i < INTERNAL_MAP_USED_BINS + INTERNAL_MAP_UPDOWN_BLOCK; i++)
        // {
        //     _obstacle_map_body_frame.distances[i] = obstacle.distances[i];
        // }
    }
    else if (obstacle.frame == obstacle.MAV_FRAME_BODY_FRD)
    {
        // Obstacle message arrives in body frame (front aligned)
        // corresponding data index (shift by msg offset)
        for (int e = 0; e < INTERNAL_MAP_USED_BINS_VER; e++)
        {
            for (int z = 0; z < INTERNAL_MAP_USED_BINS_HOR; z++)
            {
                float bin_angle_deg = (float)z * INTERNAL_MAP_INCRE_DEG_HOR + _obstacle_map_body_frame.angle_offset;
                msg_index = floor(wrap_360(bin_angle_deg - obstacle.angle_offset) * increment_factor);
                msg_index += e * INTERNAL_MAP_USED_BINS_HOR;

                // add all data points inside to FOV
                if (obstacle.distances[msg_index] != UINT16_MAX)
                {
                    if (_enterData(e * INTERNAL_MAP_USED_BINS_HOR + z, obstacle.max_distance * 0.01f,
                                obstacle.distances[msg_index] * 0.01f))
                    {
                        _obstacle_map_body_frame.distances[e * INTERNAL_MAP_USED_BINS_HOR + z] = obstacle.distances[msg_index];
                        _data_timestamps[e * INTERNAL_MAP_USED_BINS_HOR + z] = _obstacle_map_body_frame.timestamp;
                        _data_maxranges[e * INTERNAL_MAP_USED_BINS_HOR + z] = obstacle.max_distance;
                        _data_fov[e * INTERNAL_MAP_USED_BINS_HOR + z] = 1;
                    }
                }
            }
        }
    }

    else
    {
        mavlink_log_critical(&_mavlink_log_pub,
                             "Obstacle message received in unsupported frame %i\t", obstacle.frame);
        events::send<uint8_t>(events::ID("col_prev_unsup_frame"), events::Log::Error,
                              "Obstacle message received in unsupported frame {1}", obstacle.frame);
    }
}

bool CollisionPrevention::_enterData(int map_index, float sensor_range, float sensor_reading)
{
    // use data from this sensor if:
    // 1. this sensor data is in range, the bin contains already valid data and this data is coming
    // from the same or less range sensor
    // 2. this sensor data is in range, and the last reading was out of range
    // 3. this sensor data is out of range, the last reading was as well and this is the sensor with
    // longest range
    // 4. this sensor data is out of range, the last reading was valid and coming from the same
    // sensor

    uint16_t sensor_range_cm = static_cast<uint16_t>(100.0f * sensor_range + 0.5f);  // convert to
                                                                                     // cm

    if (sensor_reading < sensor_range)
    {
        if ((_obstacle_map_body_frame.distances[map_index] < _data_maxranges[map_index] &&
             sensor_range_cm <= _data_maxranges[map_index]) ||
            _obstacle_map_body_frame.distances[map_index] >= _data_maxranges[map_index])
        {
            return true;
        }
    }
    else
    {
        if ((_obstacle_map_body_frame.distances[map_index] >= _data_maxranges[map_index] &&
             sensor_range_cm >= _data_maxranges[map_index]) ||
            (_obstacle_map_body_frame.distances[map_index] < _data_maxranges[map_index] &&
             sensor_range_cm == _data_maxranges[map_index]))
        {
            return true;
        }
    }

    return false;
}

void CollisionPrevention::_updateObstacleMap()
{
    _sub_vehicle_attitude.update();

    // add distance sensor data
    for (auto &dist_sens_sub : _distance_sensor_subs)
    {
        distance_sensor_s distance_sensor;

        if (dist_sens_sub.update(&distance_sensor))
        {
            // consider only instances with valid data and orientations useful for collision
            // prevention
            if ((getElapsedTime(&distance_sensor.timestamp) < RANGE_STREAM_TIMEOUT_US) &&
                (distance_sensor.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING) &&
                (distance_sensor.orientation != distance_sensor_s::ROTATION_UPWARD_FACING))
            {
                // update message description
                _obstacle_map_body_frame.timestamp =
                    math::max(_obstacle_map_body_frame.timestamp, distance_sensor.timestamp);
                _obstacle_map_body_frame.max_distance =
                    math::max(_obstacle_map_body_frame.max_distance,
                              (uint16_t)(distance_sensor.max_distance * 100.0f));
                _obstacle_map_body_frame.min_distance =
                    math::min(_obstacle_map_body_frame.min_distance,
                              (uint16_t)(distance_sensor.min_distance * 100.0f));

                _addDistanceSensorData(distance_sensor, Quatf(_sub_vehicle_attitude.get().q));
            }
        }
    }

    // add obstacle distance data
    if (_sub_obstacle_distance.update())
    {
        const obstacle_distance_s &obstacle_distance = _sub_obstacle_distance.get();

        // Update map with obstacle data if the data is not stale
        if (getElapsedTime(&obstacle_distance.timestamp) < RANGE_STREAM_TIMEOUT_US &&
            obstacle_distance.increment > 0.f)
        {
            // update message description
            _obstacle_map_body_frame.timestamp =
                math::max(_obstacle_map_body_frame.timestamp, obstacle_distance.timestamp);
            _obstacle_map_body_frame.max_distance =
                math::max(_obstacle_map_body_frame.max_distance, obstacle_distance.max_distance);
            _obstacle_map_body_frame.min_distance =
                math::min(_obstacle_map_body_frame.min_distance, obstacle_distance.min_distance);
            _addObstacleSensorData(obstacle_distance, Quatf(_sub_vehicle_attitude.get().q));
        }
    }
    // publish fused obtacle distance message with data from offboard obstacle_distance and distance
    // sensor
    _obstacle_distance_pub.publish(_obstacle_map_body_frame);
}

void CollisionPrevention::_addDistanceSensorData(distance_sensor_s &distance_sensor,
                                                 const matrix::Quatf &vehicle_attitude)
{
    // clamp at maximum sensor range
    float distance_reading =
        math::min(distance_sensor.current_distance, distance_sensor.max_distance);

    // discard values below min range
    if ((distance_reading > distance_sensor.min_distance))
    {
        float sensor_yaw_body_rad =
            _sensorOrientationToYawOffset(distance_sensor, _obstacle_map_body_frame.angle_offset);
        float sensor_yaw_body_deg = math::degrees(wrap_2pi(sensor_yaw_body_rad));

        // calculate the field of view boundary bin indices
        int lower_bound =
            (int)floor((sensor_yaw_body_deg - math::degrees(distance_sensor.h_fov / 2.0f)) /
                       INTERNAL_MAP_INCRE_DEG_HOR);
        int upper_bound =
            (int)floor((sensor_yaw_body_deg + math::degrees(distance_sensor.h_fov / 2.0f)) /
                       INTERNAL_MAP_INCRE_DEG_HOR);

        // floor values above zero, ceil values below zero
        if (lower_bound < 0)
        {
            lower_bound++;
        }

        if (upper_bound < 0)
        {
            upper_bound++;
        }

        // rotate vehicle attitude into the sensor body frame
        matrix::Quatf attitude_sensor_frame = vehicle_attitude;
        attitude_sensor_frame.rotate(Vector3f(0.f, 0.f, sensor_yaw_body_rad));
        float sensor_dist_scale = cosf(Eulerf(attitude_sensor_frame).theta());

        if (distance_reading < distance_sensor.max_distance)
        {
            distance_reading = distance_reading * sensor_dist_scale;
        }

        uint16_t sensor_range =
            static_cast<uint16_t>(100.0f * distance_sensor.max_distance + 0.5f);  // convert to cm

        for (int bin = lower_bound; bin <= upper_bound; ++bin)
        {
            int wrapped_bin = wrap_bin(bin);

            if (_enterData(wrapped_bin, distance_sensor.max_distance, distance_reading))
            {
                _obstacle_map_body_frame.distances[wrapped_bin] =
                    static_cast<uint16_t>(100.0f * distance_reading + 0.5f);
                _data_timestamps[wrapped_bin] = _obstacle_map_body_frame.timestamp;
                _data_maxranges[wrapped_bin] = sensor_range;
                _data_fov[wrapped_bin] = 1;
            }
        }
    }
}

void CollisionPrevention::_adaptSetpointDirection(Vector2f &setpoint_dir, int &setpoint_index,
                                                  float vehicle_yaw_angle_rad)
{
    const float col_prev_d = _param_cp_dist.get();
    const int guidance_bins = floor(_param_cp_guide_ang.get() / INTERNAL_MAP_INCRE_DEG_HOR);
    const int sp_index_original = setpoint_index;
    float best_cost = 9999.f;
    int new_sp_index = setpoint_index;

    for (int i = sp_index_original - guidance_bins; i <= sp_index_original + guidance_bins; i++)
    {
        // apply moving average filter to the distance array to be able to center in larger gaps
        const int filter_size = 1;
        float mean_dist = 0;

        for (int j = i - filter_size; j <= i + filter_size; j++)
        {
            int bin = wrap_bin(j);

            if (_obstacle_map_body_frame.distances[bin] == UINT16_MAX)
            {
                mean_dist += col_prev_d * 100.f;
            }
            else
            {
                mean_dist += _obstacle_map_body_frame.distances[bin];
            }
        }

        const int bin = wrap_bin(i);
        mean_dist = mean_dist / (2.f * filter_size + 1.f);
        const float deviation_cost = col_prev_d * 50.f * abs(i - sp_index_original);
        const float bin_cost = deviation_cost - mean_dist - _obstacle_map_body_frame.distances[bin];

        if (bin_cost < best_cost && _obstacle_map_body_frame.distances[bin] != UINT16_MAX)
        {
            best_cost = bin_cost;
            new_sp_index = bin;
        }
    }

    // only change setpoint direction if it was moved to a different bin
    if (new_sp_index != setpoint_index)
    {
        float angle = math::radians((float)new_sp_index * INTERNAL_MAP_INCRE_DEG_HOR +
                                    _obstacle_map_body_frame.angle_offset);
        angle = wrap_2pi(vehicle_yaw_angle_rad + angle);
        setpoint_dir = {cosf(angle), sinf(angle)};
        setpoint_index = new_sp_index;
    }
}

float CollisionPrevention::_sensorOrientationToYawOffset(const distance_sensor_s &distance_sensor,
                                                         float angle_offset) const
{
    float offset = angle_offset > 0.0f ? math::radians(angle_offset) : 0.0f;

    switch (distance_sensor.orientation)
    {
        case distance_sensor_s::ROTATION_YAW_0:
            offset = 0.0f;
            break;

        case distance_sensor_s::ROTATION_YAW_45:
            offset = M_PI_F / 4.0f;
            break;

        case distance_sensor_s::ROTATION_YAW_90:
            offset = M_PI_F / 2.0f;
            break;

        case distance_sensor_s::ROTATION_YAW_135:
            offset = 3.0f * M_PI_F / 4.0f;
            break;

        case distance_sensor_s::ROTATION_YAW_180:
            offset = M_PI_F;
            break;

        case distance_sensor_s::ROTATION_YAW_225:
            offset = -3.0f * M_PI_F / 4.0f;
            break;

        case distance_sensor_s::ROTATION_YAW_270:
            offset = -M_PI_F / 2.0f;
            break;

        case distance_sensor_s::ROTATION_YAW_315:
            offset = -M_PI_F / 4.0f;
            break;

        case distance_sensor_s::ROTATION_CUSTOM:
            offset = matrix::Eulerf(matrix::Quatf(distance_sensor.q)).psi();
            break;
    }

    return offset;
}

// lc add
void CollisionPrevention::_ConstrainSetpoint_ZDown(float &setpointz, float stick)
{
    /* 参数说明:
     * stick > 0 向下运动 | stick < 0 向上运动 | 距离单位: 厘米/---
     * STOP_GAP_PHASE1: 第一阶段警戒距离 //m
     * STOP_GAP_PHASE2: 第二阶段危险距离 //m
     * DECEL_EXPONENT:  指数衰减系数(值越大减速越剧烈)
     */

    const float STOP_GAP_PHASE1 = _param_cp_down_gate1.get();
    const float STOP_GAP_PHASE2 = _param_cp_down_gate2.get();
    const float DECEL_EXPONENT = _param_cp_down_decay.get();  // 指数衰减强度系数
    static constexpr int CLIP_MAX_PHASE1 = 100;               // 第一阶段需持续推动次数
    static constexpr int CLIP_MAX_PHASE2 = 150;               // 第二阶段需持续推动次数

    // 状态变量
    static int phase1_counter = 0;  // 第一阶段操作计数器
    static int phase2_counter = 0;  // 第二阶段操作计数器

    // 初始化最小距离为最大可测距离
    float min_dist = _obstacle_map_body_frame.max_distance * 0.01f;
    const hrt_abstime current_time = getTime();

    // 公共处理逻辑
    auto handlePhase = [&](int &counter, int counter_max, float stop_gap)
    {
        if (stick > 0.0f)
        {  // 仅处理向下运动
            if (counter <= counter_max)
            {
                // 安全模式：强制停止并累积操作计数
                setpointz = 0.0f;
                // 摇杆强度>50%时累积，否则重置
                counter += (stick > 0.5f) ? 1 : -counter;
            }
            else
            {
                // 解除限制后：根据距离指数衰减速度
                // setpointz = stick * expf(-DECEL_EXPONENT * math::constrain((stop_gap - min_dist)
                // / stop_gap, 0.0f, 1.0f));
                float distance_ratio = (stop_gap - min_dist) / stop_gap;  // 危险程度[0,1]
                distance_ratio = math::constrain(distance_ratio, 0.0f, 1.0f);
                float speed_decay = expf(-DECEL_EXPONENT * distance_ratio);  // 指数衰减
                setpointz = stick * speed_decay;                             // 应用衰减后的速度
            }
        }
        else
        {  // 摇杆释放时重置状态
            setpointz = stick;
            counter = 0;
        }
    };

    // 更新障碍物地图数据
    _updateObstacleMap();

    // 检查传感器数据有效性(300ms超时)
    if ((current_time - _obstacle_map_body_frame.timestamp) < RANGE_STREAM_TIMEOUT_US)
    {
        // 遍历指定区域寻找最小障碍物距离

        //E方向分隔数为偶数
        int e = 0;
        for (int z = 0; z < INTERNAL_MAP_USED_BINS_HOR; ++z)
        {
            if (_obstacle_map_body_frame.distances[e*INTERNAL_MAP_INCRE_DEG_HOR+z] * 0.01f < min_dist)
                min_dist = _obstacle_map_body_frame.distances[e*INTERNAL_MAP_INCRE_DEG_HOR+z] * 0.01f;
        }

        // 分级安全处理
        if (min_dist < STOP_GAP_PHASE2)
        {  // 进入危险距离
            handlePhase(phase2_counter, CLIP_MAX_PHASE2, STOP_GAP_PHASE2);
            phase1_counter = 0;  // 重置第一阶段计数器
        }
        else if (min_dist < STOP_GAP_PHASE1)
        {  // 进入警戒距离
            handlePhase(phase1_counter, CLIP_MAX_PHASE1, STOP_GAP_PHASE1);
            phase2_counter = 0;  // 重置第二阶段计数器
        }
        else
        {  // 安全距离内
            setpointz = stick;
            phase1_counter = phase2_counter = 0;
        }
    }
    else
    {  // 传感器数据超时处理
        setpointz = 0.0f;
        phase1_counter = phase2_counter = 0;
    }

    // 调试数据发布
    // struct debug_key_value_s dbg{};
    // strncpy(dbg.key, "min_dist", sizeof(dbg.key));
    // dbg.value = min_dist;
    // dbg.timestamp = current_time;
    // orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);
    // orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
}

void CollisionPrevention::_ConstrainSetpoint_ZUp(float &setpointz, float stick)
{
    const float slow_gap = _param_cp_up_gate1.get();
    const float stop_gap = _param_cp_up_gate2.get();
    const float DECEL_EXPONENT = _param_cp_up_decay.get();  // 指数衰减强度系数
    const hrt_abstime current_time = getTime();
    // 初始化最小距离为最大可测距离
    float min_dist = _obstacle_map_body_frame.max_distance * 0.01f;

    // 更新障碍物地图数据
    _updateObstacleMap();

    // 检查传感器数据有效性(300ms超时)
    if ((current_time - _obstacle_map_body_frame.timestamp) < RANGE_STREAM_TIMEOUT_US)
    {
        if (stick < 0.0f)
        {
            // 遍历指定区域寻找最小障碍物距离
            int e = INTERNAL_MAP_USED_BINS_VER - 1;
            for (int z = 0; z < INTERNAL_MAP_USED_BINS_HOR; ++z)
            {
                if (_obstacle_map_body_frame.distances[e*INTERNAL_MAP_USED_BINS_HOR+z] * 0.01f < min_dist)
                    min_dist = _obstacle_map_body_frame.distances[e*INTERNAL_MAP_USED_BINS_HOR+z] * 0.01f;
            }

            if (min_dist < stop_gap)
                setpointz = 0.0f;

            else if (min_dist < slow_gap)
            {
                // 计算减速系数：越接近停止距离，衰减越强
                const float decel_range = slow_gap - stop_gap;
                const float distance_ratio = (min_dist - stop_gap) / decel_range;

                // 使用三次曲线实现平滑过渡（可替换为其他缓动函数）
                const float eased_ratio =
                    distance_ratio * distance_ratio * (3.0f - 2.0f * distance_ratio);
                const float speed_decay = expf(-DECEL_EXPONENT * (1.0f - eased_ratio));

                setpointz = stick * speed_decay;
            }
            else
                return;
        }

        else
            return;
    }

    else
    {
        setpointz = 0.0f;
    }
}

// xy平面刹停逻辑
void CollisionPrevention::_ConstrainSetpoint_XY(Vector2f &setpoint, const Vector2f &curr_pos, const Vector2f &curr_vel)
{
    // 更新obstacle_distance
    _updateObstacleMap();

    // read parameters
    // 最小安全距离
    const float col_prev_d = _param_cp_dist.get();
    // 距离延时时间 附加
    const float col_prev_dly = _param_cp_delay.get();
    // 无数据时是否允许运动
    //const bool move_no_data = _param_cp_go_nodata.get();
    // 运动学参数获取
    const float xy_p = _param_mpc_xy_p.get();
    const float max_jerk = _param_mpc_jerk_max.get();
    const float max_accel = _param_mpc_acc_hor.get();
    const matrix::Quatf attitude = Quatf(_sub_vehicle_attitude.get().q);
    const float vehicle_yaw_angle_rad = Eulerf(attitude).psi();

    // 设定点模长
    const float setpoint_length = setpoint.norm();

    const hrt_abstime constrain_time = getTime();


    // 数据有效 进入避障逻辑
    if ((constrain_time - _obstacle_map_body_frame.timestamp) > RANGE_STREAM_TIMEOUT_US)
    {
        setpoint = Vector2f(0.0f, 0.0f);
        // 发布进入悬停模式
        _publishVehicleCmdDoLoiter();
        return;
    }

    // 用户未打杆
    if (setpoint_length <= 0.001f)
        return;


    // lc add 检测范围正负18度
    const float rad_threshold = math::radians(20.0f);
    // xy平面 setpoint_dir设定值归一化 归一化为单位向量
    Vector2f setpoint_dir = setpoint / setpoint_length;
    // 最大速度值 设置为模长
    float vel_max = setpoint_length;
    // 最小安全距离
    const float min_dist_to_keep =
        math::max(_obstacle_map_body_frame.min_distance / 100.0f, col_prev_d);

    // lc add 世界系设定弧度
    const float sp_rad_local = wrap_2pi((atan2f(setpoint_dir(1), setpoint_dir(0))));

    // limit speed for safe flight
    // 遍历每一个扇区
    for (int e = INTERNAL_MAP_USED_BINS_VER / 2 - 1; e <= INTERNAL_MAP_USED_BINS_VER / 2; e++)
    {
        for (int z = 0; z < INTERNAL_MAP_USED_BINS_HOR; z++)
        {
            int index = e * INTERNAL_MAP_USED_BINS_HOR + z;

            // delete stale values
            // 保留计算数据时间
            const hrt_abstime data_age = constrain_time - _data_timestamps[index];

            // 超时则将所有数据置换为无效
            if (data_age > RANGE_STREAM_TIMEOUT_US)
            {
                _obstacle_map_body_frame.distances[index] = UINT16_MAX;
            }

            if (_obstacle_map_body_frame.distances[index] < _obstacle_map_body_frame.min_distance || _obstacle_map_body_frame.distances[index] == UINT16_MAX)
                continue;

            // 转换单位为米 m
            const float distance = _obstacle_map_body_frame.distances[index] * 0.01f;   // convert to meters
            const float max_range = _data_maxranges[index] * 0.01f;  // convert to meters

            // 当前循环中的扇区的对应弧度
            float rad = math::radians((float)z * INTERNAL_MAP_INCRE_DEG_HOR +
                                        _obstacle_map_body_frame.angle_offset);

            // convert from body to local frame in the range [0, 2*pi]
            // 机体系下弧度 + 当前yaw角 转换为世界系下弧度
            rad = wrap_2pi(vehicle_yaw_angle_rad + rad);

            // get direction of current bin
            // 当前扇区值角度转换为xy方向向量
            // 转换为世界系 统一后方便与setpoint_dir比较
            const Vector2f bin_direction = {cosf(rad), sinf(rad)};

            // 不在检测范围内 直接跳过循环
            float rad_diff = wrap_pi(rad - sp_rad_local);
            if (fabsf(rad_diff) > rad_threshold)
            {
                continue;
            }

            // 在范围内时进行避障逻辑

            // calculate max allowed velocity with a P-controller (same gain as in the
            // position controller)
            // 当前速度在该方向的投影
            const float curr_vel_parallel = math::max(0.f, curr_vel.dot(bin_direction));
            // 延迟导致的位移
            float delay_distance = curr_vel_parallel * col_prev_dly;

            if (distance < max_range)
            {
                // 数据老化补偿
                delay_distance += curr_vel_parallel * (data_age * 1e-6f);
            }

            // 安全距离为识别到的障碍距离-最小安全距离-延迟位移
            // 若stop_distance为0 则输出vel_max_posctrl直接为0
            const float stop_distance = math::max(0.f, distance - min_dist_to_keep - delay_distance);
            // 位置控制器速度限制；距离越近 速度被限制得越小
            const float vel_max_posctrl = xy_p * stop_distance;

            // 调用函数光滑化目标设定值  运动学平滑限制
            const float vel_max_smooth = math::trajectory::computeMaxSpeedFromDistance(
                max_jerk, max_accel, stop_distance, 0.f);
            // 投影值（projection）越大（方向越一致）d
            const float projection = bin_direction.dot(setpoint_dir);
            float vel_max_bin = vel_max;

            if (projection > 0.01f)
            {
                // 综合限制  projection越大方向越一致，即分母越大，vel_max_bin
                // 越小，速度限制越严格
                // 结合位置控制器（响应快）和运动学模型（平滑性）生成速度限制
                vel_max_bin = math::min(vel_max_posctrl, vel_max_smooth) / projection;
            }

            // constrain the velocity
            if (vel_max_bin >= 0)
            {
                vel_max = math::min(vel_max, vel_max_bin);
            }

        }
    }

    // 调整后最终值为单位方向向量*速度
    // 该函数主要通过遍历并计算邻近扇区（点乘>0）的障碍物信息来调整速度值
    //  调整方向主要在_adaptSetpointDirection 函数中
    setpoint = setpoint_dir * vel_max;
}

void CollisionPrevention::modifySetpoint(Vector2f &original_setpoint, const float max_speed,
                                         const Vector2f &curr_pos, const Vector2f &curr_vel,
                                         float &setpointz)
{
    //_updateObstacleMap();
    const bool cp_mode = _param_cp_mode.get();
    // calculate movement constraints based on range data
    Vector2f new_setpoint = original_setpoint;

    if (!cp_mode)
    {
        _ConstrainSetpoint_XY(new_setpoint, curr_pos, curr_vel);
        float sp_zd = setpointz;
        _ConstrainSetpoint_ZDown(setpointz, sp_zd);
        float sp_zu = setpointz;
        _ConstrainSetpoint_ZUp(setpointz, sp_zu);
    }

    else
    {
        applyAvoidance(new_setpoint, setpointz);
    }

    // warn user if collision prevention starts to interfere
    bool currently_interfering = (new_setpoint(0) < original_setpoint(0) - 0.05f * max_speed ||
                                  new_setpoint(0) > original_setpoint(0) + 0.05f * max_speed ||
                                  new_setpoint(1) < original_setpoint(1) - 0.05f * max_speed ||
                                  new_setpoint(1) > original_setpoint(1) + 0.05f * max_speed);

    _interfering = currently_interfering;

    // publish constraints
    collision_constraints_s constraints{};
    constraints.timestamp = getTime();
    original_setpoint.copyTo(constraints.original_setpoint);
    new_setpoint.copyTo(constraints.adapted_setpoint);
    _constraints_pub.publish(constraints);

    original_setpoint = new_setpoint;
}

void CollisionPrevention::_publishVehicleCmdDoLoiter()
{
    vehicle_command_s command{};
    command.timestamp = getTime();
    command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
    command.param1 = (float)1;  // base mode
    command.param3 = (float)0;  // sub mode
    command.target_system = 1;
    command.target_component = 1;
    command.source_system = 1;
    command.source_component = 1;
    command.confirmation = false;
    command.from_external = false;
    command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
    command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;

    // publish the vehicle command
    _vehicle_command_pub.publish(command);
}

void CollisionPrevention::applyAvoidance(Vector2f &setpoint_xy, float &setpointz)
{
    // const bool ooa_mode = _param_cp_mode.get();
    const float DECEL_DIS = _param_cp_decel_dis.get();
    const float BP_DIS = _param_cp_bypass_dis.get();
    const float PRESET_XY_VEL_MAX = _param_mpc_xy_vmax.get();
    const float PRESET_Z_VEL_MAX = std::fmax(_param_mpc_zup_vmax.get(), _param_mpc_zdn_vmax.get());
    const float VEL_MAX = _param_cp_vel_max.get();

    //速度限幅映射
    if(PRESET_XY_VEL_MAX > VEL_MAX)
    {
        scaled_setpoint(0) = setpoint_xy(0) * (VEL_MAX / PRESET_XY_VEL_MAX);
        scaled_setpoint(1) = setpoint_xy(1) * (VEL_MAX / PRESET_XY_VEL_MAX);
    }
    else
    {
        scaled_setpoint(0) = setpoint_xy(0);
        scaled_setpoint(1) = setpoint_xy(1);
    }

    if(PRESET_Z_VEL_MAX > VEL_MAX)
    {
        scaled_setpoint(2) = setpointz * (VEL_MAX / PRESET_Z_VEL_MAX);
    }
    else
    {
        scaled_setpoint(2) = setpointz;
    }


    //debug used
    //Vector3f scaled_setpoint_dir = scaled_setpoint / scaled_setpoint.norm();

    // 用户未打杆时 不检测
    if (scaled_setpoint.norm() < 0.1f)
        return;

    // 更新障碍物距离数据
    _updateObstacleMap();

    // 获取目标方向最小距离
    float tardir_min_dist = _get_TargetDir_MinDist(scaled_setpoint);

    // 状态切换逻辑
    switch (bp_state_)
    {
        // 纯摇杆映射
        case MANUAL:
            // 障碍物距离未触发绕行 到达减速阈值 切换到减速状态
            if (tardir_min_dist < DECEL_DIS && tardir_min_dist >= BP_DIS)
            {
                bp_state_ = SLOWING_DOWN;
            }
            // 可能是移动yaw 障碍物距离到达绕行阈值 切换到绕行状态
            else if (tardir_min_dist < BP_DIS)
            {
                bp_state_ = AVOIDING;
            }
            break;

        case SLOWING_DOWN:
            // 判读为安全 切换回摇杆映射模式
            if (tardir_min_dist >= DECEL_DIS)
            {
                bp_state_ = MANUAL;
            }
            // 减速后达到绕行阈值后切换到绕行状态
            else if (tardir_min_dist < BP_DIS)
            {
                bp_state_ = AVOIDING;
            }
            break;

        case AVOIDING:
            // 绕行完成 切换到恢复状态
            if (tardir_min_dist >= BP_DIS)
            {
                bp_state_ = RECOVERING;
                _recovery_start = hrt_absolute_time();
            }
            break;

        case RECOVERING:
            // 恢复完成 切换回摇杆映射模式
            if (tardir_min_dist >= DECEL_DIS)
            {
                if (hrt_elapsed_time(&_recovery_start) > RECOVERY_TIME * 1e6)
                {
                    bp_state_ = MANUAL;
                }
            }
            // 若突然又识别到障碍物 且距离范围在减速过程中 切换到减速状态
            else if (tardir_min_dist < DECEL_DIS && tardir_min_dist > BP_DIS)
            {
                bp_state_ = SLOWING_DOWN;
            }
            // 若突然又识别到障碍物 且距离低于绕行阈值 切换到绕行状态
            else if (tardir_min_dist <= BP_DIS)
            {
                bp_state_ = AVOIDING;
            }
            break;
    }

    // 输出控制逻辑
    switch (bp_state_)
    {
        case MANUAL:
            // 原样输出
            break;

        case SLOWING_DOWN:
            // 减速平滑输出
            scaled_setpoint = calculateSlowdown(scaled_setpoint, tardir_min_dist);
            break;

        case AVOIDING:
            // 绕行输出 保存最后一次绕行命令 以备恢复使用
            scaled_setpoint = _calculateAvoidanceCommand(scaled_setpoint);
            _last_avoidance_cmd = scaled_setpoint;
            break;

        case RECOVERING:
            // 恢复输出 混合绕行命令和原命令 以平滑过渡
            float t = hrt_elapsed_time(&_recovery_start) / (RECOVERY_TIME * 1e6);
            t = math::constrain(t, 0.0f, 1.0f);
            scaled_setpoint = _blendCommands(scaled_setpoint, _last_avoidance_cmd, 1.0f - t);
            break;
    }

    setpoint_xy(0) = scaled_setpoint(0);
    setpoint_xy(1) = scaled_setpoint(1);
    setpointz = scaled_setpoint(2);

    // // 调试使用
    // static orb_advert_t dbg_vect_pub = nullptr;
    // struct debug_vect_s dbg_vect{};
    // strncpy(dbg_vect.name, "bp_state_", sizeof(dbg_vect.name));
    // dbg_vect.timestamp = hrt_absolute_time();
    // dbg_vect.x = tardir_min_dist;
    // dbg_vect.y = setpoint_xy(0);
    // dbg_vect.z = setpoint_xy(1);

    // if (dbg_vect_pub == nullptr)
    // {
    //     dbg_vect_pub = orb_advertise(ORB_ID(debug_vect), &dbg_vect);
    // }
    // else
    // {
    //     orb_publish(ORB_ID(debug_vect), dbg_vect_pub, &dbg_vect);
    // }

}

Vector3f CollisionPrevention::_blendCommands(const Vector3f &user_cmd, const Vector3f &avoid_cmd,
                                             float ratio)
{
    // 使用三次贝塞尔曲线平滑过渡
    float t = ratio * ratio * (3.0f - 2.0f * ratio);
    return user_cmd * (1.0f - t) + avoid_cmd * t;
}

Vector3f CollisionPrevention::calculateSlowdown(const Vector3f &original, float min_dist) const
{
    const float MIN_RATIO = 0.25f;
    const float DECEL_DIS = _param_cp_decel_dis.get();
    const float BP_DIS = _param_cp_bypass_dis.get();
    float ratio = math::constrain((min_dist - BP_DIS) / (DECEL_DIS - BP_DIS), MIN_RATIO, 1.0f);
    return original * ratio;
}

float CollisionPrevention::_get_TargetDir_MinDist(const Vector3f &setpoint)
{
    const matrix::Quatf attitude = Quatf(_sub_vehicle_attitude.get().q);
    const float vehicle_yaw_angle_rad = Eulerf(attitude).psi();
    //setpoint为世界系下设定速度
    Vector3f setpoint_dir = setpoint / setpoint.norm();
    // 角度限制：30度内的锥形区域
    const float COS_ANGLE_THRESHOLD = cosf(math::radians(30.0f));  // ≈ 0.866
    float min_dist = INFINITY;

    //遍历所有网格
    for(int e = 0; e < INTERNAL_MAP_USED_BINS_VER; e++)
    {
        for (int z = 0; z < INTERNAL_MAP_USED_BINS_HOR; z++)
        {
            float raw_dist = _obstacle_map_body_frame.distances[e*INTERNAL_MAP_USED_BINS_HOR + z];
            if (raw_dist <= _obstacle_map_body_frame.min_distance || raw_dist >= UINT16_MAX)
                continue;

            float angle_e = -(((float)e + 0.5f) * INTERNAL_MAP_INCRE_DEG_VER - 90.0f);
            float rad_e = math::radians(angle_e + _obstacle_map_body_frame.angle_offset);
            float rad_z = math::radians(((float)z + 0.5f) * INTERNAL_MAP_INCRE_DEG_HOR + _obstacle_map_body_frame.angle_offset);
            //_obstacle_map_body_frame中数据为机体系 需要转到世界系下进行比较
            rad_z = wrap_2pi(vehicle_yaw_angle_rad + rad_z);

            //遍历的网格对应的方向向量
            Vector3f ray_dir = {
                cosf(rad_e) * cosf(rad_z),
                cosf(rad_e) * sinf(rad_z),
                sinf(rad_e)
            };

            // 只保留前向 ±30° 的锥形区域
            float dot_prod = ray_dir.dot(setpoint_dir);
            //越小夹角越大
            if (dot_prod < COS_ANGLE_THRESHOLD)
                continue;

            float dist_m = raw_dist * 0.01f;
            if (dist_m < min_dist)
            {
                min_dist = dist_m;
            }
        }
    }

    return min_dist;
}

Vector4f CollisionPrevention::_get_Safe_Dir(const Vector3f &setpoint)
{
    const float CP_DIS = _param_cp_bypass_dis.get();
    //方向权重
    //const float ALIGNMENT_GAIN = _param_cp_align_gain.get();
    //距离权重
    const float DISTANCE_GAIN = _param_cp_dis_gain.get();
    // 计算距离评分时考虑临近扇区 避免绕行时过于贴近障碍物
    const int NEIGHBOR_BINS = _param_cp_nei_bins.get();
    const matrix::Quatf attitude = Quatf(_sub_vehicle_attitude.get().q);
    const float vehicle_yaw_angle_rad = Eulerf(attitude).psi();
    //setpoint为世界系下设定速度
    Vector3f setpoint_dir = setpoint / setpoint.norm();
    // 角度限制：180度内的锥形区域
    const float COS_ANGLE_THRESHOLD = cosf(math::radians(180.0f));
    float min_dist = INFINITY;
    float max_safety = -INFINITY;
    Vector3f best_dir = setpoint.normalized();  // 默认方向为 setpoint
    const float SAFE_DISTANCE = _obstacle_map_body_frame.max_distance * 0.01f;
    int best_e = 0;
    int best_z = 0;

    //遍历所有网格
    for(int e = 0; e < INTERNAL_MAP_USED_BINS_VER; e++)
    {
        for (int z = 0; z < INTERNAL_MAP_USED_BINS_HOR; z++)
        {
            float raw_dist = _obstacle_map_body_frame.distances[e*INTERNAL_MAP_USED_BINS_HOR + z];
            if (raw_dist <= _obstacle_map_body_frame.min_distance || raw_dist >= UINT16_MAX)
                continue;

            //e范围0-180 但是此处z轴方向向下为正 朝下时应为1
            float angle_e = -(((float)e + 0.5f) * INTERNAL_MAP_INCRE_DEG_VER - 90.0f);
            float rad_e = math::radians(angle_e +_obstacle_map_body_frame.angle_offset);
            float rad_z = math::radians(((float)z + 0.5f) * INTERNAL_MAP_INCRE_DEG_HOR + _obstacle_map_body_frame.angle_offset);
            //_obstacle_map_body_frame中数据为机体系 需要转到世界系下进行比较
            rad_z = wrap_2pi(vehicle_yaw_angle_rad + rad_z);

            //遍历的网格对应的方向向量
            Vector3f ray_dir = {
                cosf(rad_e) * cosf(rad_z),
                cosf(rad_e) * sinf(rad_z),
                sinf(rad_e)
            };

            // 只保留前向 ±=180° 的锥形区域
            //dot_prod范围-1~1 越接近1对齐程度越高 0时90度正交
            float dot_prod = ray_dir.dot(setpoint_dir);
            //越小夹角越大
            if (dot_prod < COS_ANGLE_THRESHOLD)
                continue;

            //仅适用于18*6=108的情况 由于摄像头视场角原因 斜向上与斜向下无数据
            float block_min_dist = INFINITY;
            if(e == 0 || e == INTERNAL_MAP_USED_BINS_VER - 1)
            {
                for(int nei_z = -2; nei_z <= 2; nei_z++)
                {
                    int neighbor_z = z + nei_z;
                    neighbor_z %= INTERNAL_MAP_USED_BINS_HOR;
                    if (_obstacle_map_body_frame.distances[e * INTERNAL_MAP_USED_BINS_HOR + neighbor_z] >= UINT16_MAX)
                    {
                        block_min_dist = INFINITY;
                        break;
                    }
                    float nei_min_dist = _obstacle_map_body_frame.distances[e * INTERNAL_MAP_USED_BINS_HOR + neighbor_z] * 0.01f;
                    if(nei_min_dist < CP_DIS)
                    {
                        block_min_dist = INFINITY;
                        break;
                    }
                    if(nei_min_dist < block_min_dist)
                    {
                        block_min_dist = nei_min_dist;
                    }
                }
            }

            if(e == INTERNAL_MAP_USED_BINS_VER / 2 || e == INTERNAL_MAP_USED_BINS_VER / 2 - 1)
            {
                for(int nei_e = INTERNAL_MAP_USED_BINS_VER / 2 - 1; nei_e <= INTERNAL_MAP_USED_BINS_VER / 2; nei_e++)
                {
                    for(int nei_z = -NEIGHBOR_BINS; nei_z <= NEIGHBOR_BINS; nei_z++)
                    {
                        int neighbor_z = z + nei_z;
                        neighbor_z %= INTERNAL_MAP_USED_BINS_HOR;
                        if (_obstacle_map_body_frame.distances[nei_e * INTERNAL_MAP_USED_BINS_HOR + neighbor_z] >= UINT16_MAX)
                        {
                            block_min_dist = INFINITY;
                            break;
                        }
                        float nei_min_dist = _obstacle_map_body_frame.distances[nei_e * INTERNAL_MAP_USED_BINS_HOR + neighbor_z] * 0.01f;
                        if(nei_min_dist < block_min_dist)
                        {
                            block_min_dist = nei_min_dist;
                        }
                    }
                }
            }
            else{
                continue;
            }


            if (block_min_dist < min_dist) {
                min_dist = block_min_dist;
            }

            // 安全评分 = 距离评分 + 方向对齐评分
            float distance_score = block_min_dist / SAFE_DISTANCE;  //[0,1]
            float alignment_score = 0.5f * (dot_prod + 1.0f);  // [0,1]

            float total_score = DISTANCE_GAIN * distance_score + 0 * alignment_score;

            if (total_score > max_safety)
            {
                best_e = e;
                best_z = z;
                max_safety = total_score;
                best_dir = ray_dir.normalized();
            }
        }
    }

    // 调试使用
    static orb_advert_t dbg_vect_pub = nullptr;
    struct debug_vect_s dbg_vect{};
    strncpy(dbg_vect.name, "best_dir", sizeof(dbg_vect.name));
    dbg_vect.timestamp = hrt_absolute_time();
    dbg_vect.x = best_e;
    dbg_vect.y = best_z;
    dbg_vect.z = 1;

    if (dbg_vect_pub == nullptr)
    {
        dbg_vect_pub = orb_advertise(ORB_ID(debug_vect), &dbg_vect);
    }
    else
    {
        orb_publish(ORB_ID(debug_vect), dbg_vect_pub, &dbg_vect);
    }

    return Vector4f(best_dir(0), best_dir(1), best_dir(2), min_dist);
}

Vector3f CollisionPrevention::_calculateAvoidanceCommand(const Vector3f &original)
{
    const float MAX_AVOID_SPEED = _param_cp_bypass_vel.get();
    //const float EMERGENCY_DISTANCE = _param_cp_emergency_dis.get();

    // 计算最佳方向和最小距离
    Vector4f best_direction = _get_Safe_Dir(original);
    //float min_dist = best_direction(3);
    Vector3f best_dir = Vector3f(best_direction(0), best_direction(1), best_direction(2));

    //min_dist 小于紧急值则停止
    // if(min_dist < EMERGENCY_DISTANCE)
    //     return Vector3f{0.f,0.f,0.f};
    // else
    //     return best_dir * MAX_AVOID_SPEED;

    return best_dir * MAX_AVOID_SPEED;
}
