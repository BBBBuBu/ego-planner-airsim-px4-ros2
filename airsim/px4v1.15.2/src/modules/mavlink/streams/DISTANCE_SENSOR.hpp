/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef DISTANCE_SENSOR_HPP
#define DISTANCE_SENSOR_HPP

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/distance_sensor.h>

// hw-modify 由于MAVLINK-SDK没有解析wxyz的四元数，因此先根据四元数得到方向参数ORIENTATION
// 后续要修改 MAVLINK-SDK 让其支持四元数，还要修改AirSIM让它的距离传感器支持ORIENTATION设置
class MavlinkStreamDistanceSensor : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamDistanceSensor(mavlink); }

	static constexpr const char *get_name_static() { return "DISTANCE_SENSOR"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DISTANCE_SENSOR; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _distance_sensor_subs.advertised_count() * (MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	explicit MavlinkStreamDistanceSensor(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};

	// hw-modify 根据四元数计算方向
	uint8_t quaternion_to_orientation(float w, float x, float y, float z)
	{
	
		// 传感器旋转角，决定前方探测四边形的大小。
		// 目前有 90,45,25 三种。因为目前只有45°和90°的宏定义，因此25°也用45°的宏表示，后续要改
		#define DISTANCE_ANGLE	25
		#define FLOAT_EQUE(F1, F2) (std::fabs((double)(F1) - (double)(F2)) < (double)(1e-4f))
		if (FLOAT_EQUE(w,1.0) && FLOAT_EQUE(x,0.0) && FLOAT_EQUE(y,0.0) && FLOAT_EQUE(z,0.0))
		{
			// 指向前方的传感器
			return MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_NONE;
			//ROTATION_PITCH_180_YAW_90;
		}
		#if (DISTANCE_ANGLE==25)
		else if (FLOAT_EQUE(w,0.97630) && FLOAT_EQUE(x,0.0) && FLOAT_EQUE(y,0.0) && FLOAT_EQUE(z,0.21644))
		{
			// 指向右前方的传感器
			return MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_45;
		}
		else if (FLOAT_EQUE(w,0.97630) && FLOAT_EQUE(x,0.0) && FLOAT_EQUE(y,0.0) && FLOAT_EQUE(z,(-0.21644)))
		{
			// 指向左前方的传感器
			return MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_135;
		}
		else if (FLOAT_EQUE(w,0.97630) && FLOAT_EQUE(x,0.0) && FLOAT_EQUE(y,0.21644) && FLOAT_EQUE(z,0.0))
		{
			// 指向前上方的传感器
			return MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_ROLL_90_YAW_135;
		}
		else if (FLOAT_EQUE(w,0.97630) && FLOAT_EQUE(x,0.0) && FLOAT_EQUE(y,(-0.21644)) && FLOAT_EQUE(z,0.0))
		{
			// 指向前下方的传感器
			return MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_ROLL_90_YAW_45;
		}
		
		#elif (DISTANCE_ANGLE==45)
		else if (FLOAT_EQUE(w,0.92388) && FLOAT_EQUE(x,0.0) && FLOAT_EQUE(y,0.0) && FLOAT_EQUE(z,0.38268))
		{
			// 指向右前方的传感器
			return MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_45;
		}
		else if (FLOAT_EQUE(w,0.92388) && FLOAT_EQUE(x,0.0) && FLOAT_EQUE(y,0.0) && FLOAT_EQUE(z,(-0.38268)))
		{
			// 指向左前方的传感器
			return MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_135;
		}
		else if (FLOAT_EQUE(w,0.92388) && FLOAT_EQUE(x,0.0) && FLOAT_EQUE(y,0.38268) && FLOAT_EQUE(z,0.0))
		{
			// 指向前上方的传感器
			return MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_ROLL_90_YAW_135;
		}
		else if (FLOAT_EQUE(w,0.92388) && FLOAT_EQUE(x,0.0) && FLOAT_EQUE(y,(-0.38268)) && FLOAT_EQUE(z,0.0))
		{
			// 指向前下方的传感器
			return MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_ROLL_90_YAW_45;
		}
		#elif (DISTANCE_ANGLE==90)
		else if (FLOAT_EQUE(w,0.70711) && FLOAT_EQUE(x,0.0) && FLOAT_EQUE(y,0.0) && FLOAT_EQUE(z,(-0.70711)))
		{
			// 指向左边的传感器
			return MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_270;
		}
		else if (FLOAT_EQUE(w,0.70711) && FLOAT_EQUE(x,0.0) && FLOAT_EQUE(y,0.0) && FLOAT_EQUE(z,0.70711))
		{
			// 指向右边的传感器
			return MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_90;
		}
		else if (FLOAT_EQUE(w,0.70711) && FLOAT_EQUE(x,0.0) && FLOAT_EQUE(y,0.70711) && FLOAT_EQUE(z,0.0))
		{
			// 指向上方的传感器
			return MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_90;
		}
		else if (FLOAT_EQUE(w,0.70711) && FLOAT_EQUE(x,0.0) && FLOAT_EQUE(y,(-0.70711)) && FLOAT_EQUE(z,0.0))
		{
			// 指向下方的传感器
			return MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_270;
		}
		#endif
		else
		{
			// 不是上述6个方向的传感器
			return MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_CUSTOM;
		}
	}

	bool send() override
	{
		bool updated = false;

		for (int i = 0; i < _distance_sensor_subs.size(); i++) {
			distance_sensor_s dist_sensor;

			if (_distance_sensor_subs[i].update(&dist_sensor)) {
				mavlink_distance_sensor_t msg{};

				msg.time_boot_ms = dist_sensor.timestamp / 1000; /* us to ms */

				switch (dist_sensor.type) {
				case MAV_DISTANCE_SENSOR_ULTRASOUND:
					msg.type = MAV_DISTANCE_SENSOR_ULTRASOUND;
					break;

				case MAV_DISTANCE_SENSOR_LASER:
					msg.type = MAV_DISTANCE_SENSOR_LASER;
					break;

				case MAV_DISTANCE_SENSOR_INFRARED:
					msg.type = MAV_DISTANCE_SENSOR_INFRARED;
					break;

				default:
					msg.type = MAV_DISTANCE_SENSOR_LASER;
					break;
				}

				msg.current_distance = dist_sensor.current_distance * 1e2f; // m to cm
				msg.id               = i;
				msg.max_distance     = dist_sensor.max_distance * 1e2f;     // m to cm
				msg.min_distance     = dist_sensor.min_distance * 1e2f;     // m to cm
				msg.orientation      = dist_sensor.orientation;
				msg.covariance       = dist_sensor.variance * 1e4f;         // m^2 to cm^2

				// hw-modify 补全信息
				msg.orientation = quaternion_to_orientation(dist_sensor.q[0], dist_sensor.q[1], dist_sensor.q[2], dist_sensor.q[3]);
				msg.horizontal_fov = dist_sensor.h_fov;
				msg.vertical_fov = dist_sensor.v_fov;
				msg.quaternion[0] = dist_sensor.q[0];
				msg.quaternion[1] = dist_sensor.q[1];
				msg.quaternion[2] = dist_sensor.q[2];
				msg.quaternion[3] = dist_sensor.q[3];
				msg.signal_quality = dist_sensor.signal_quality;

				mavlink_msg_distance_sensor_send_struct(_mavlink->get_channel(), &msg);

				updated = true;
			}
		}

		return updated;
	}
};

#endif // DISTANCE_SENSOR
