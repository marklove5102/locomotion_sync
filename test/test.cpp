#include "locomotion.h"
#include <iostream>
#include <vector>
#include <array>
#include "point.h"
#include <random>
#include <cassert>
using namespace spiritsaway::locomotion_sync;
using namespace spiritsaway::geometry;

template <typename T, int N>
void sync_two_pos_sync(const vec_sync_data<T, N>& sender, vec_sync_data<T, N>& receiver)
{
	std::array<std::int8_t, 128> buffer;
	auto encode_sz = sender.encode(buffer.data());
	auto remain_sz = receiver.decode(buffer.data(), encode_sz);
	assert(remain_sz == encode_sz);
}
template <typename T>
void test_line(T max_accelerate, T max_speed, std::vector<basic_point<T>> corridor,  const T radius, const T position_scale, const T speed_scale)
{
	vec_sync_data<T, 3> pos_sync_data_sender(position_scale);
	vec_sync_data<T, 3> pos_sync_data_receiver(position_scale);
	vec_sync_data<T, 3> speed_sync_data_sender(speed_scale);
	vec_sync_data<T, 3>	speed_sync_data_receiver(speed_scale);

	std::uint32_t pos_sync_data_sz = 0;
	std::uint32_t sync_raw_sz = 0;
	std::uint32_t speed_sync_data_sz = 0;
	basic_point<T> pos = corridor[0];
	basic_point<T> pre_speed, new_speed;

	basic_point<T> sync_pos;
	basic_point<T> sync_speed;
	
	std::uint32_t dest_idx = 1;
	pos_sync_data_sender.full(pos.data);
	sync_two_pos_sync(pos_sync_data_sender, pos_sync_data_receiver);
	sync_pos = pos_sync_data_receiver.latest_snapshot();

	pos_sync_data_sz += pos_sync_data_sender.diff_buffer_sz();
	speed_sync_data_sender.full(pre_speed.data);
	sync_two_pos_sync(speed_sync_data_sender, speed_sync_data_receiver);
	sync_speed = speed_sync_data_receiver.latest_snapshot();
	speed_sync_data_sz += speed_sync_data_sender.diff_buffer_sz();
	T dt = 0.1f;
	sync_raw_sz = sizeof(T) * 3;
	int i = 0;
	while (true)
	{
		i++;
		auto cur_dest = corridor[dest_idx];
		auto desire_speed = cur_dest - pos;
		desire_speed.normalize();
		desire_speed *= max_speed;
		auto speed_diff = desire_speed - pre_speed;
		if (speed_diff.length() > dt * max_accelerate)
		{
			speed_diff.normalize();
			new_speed += speed_diff * dt * max_accelerate;
		}
		else
		{
			new_speed = desire_speed;
		}
		basic_point<T> new_pos = pos + new_speed * dt;

		speed_sync_data_sender.diff(new_speed.data);
		sync_two_pos_sync(speed_sync_data_sender, speed_sync_data_receiver);
		sync_speed = speed_sync_data_receiver.latest_snapshot();
		speed_sync_data_sz += speed_sync_data_sender.diff_buffer_sz() + 1;
		pos_sync_data_sender.diff(new_pos.data);
		sync_two_pos_sync(pos_sync_data_sender, pos_sync_data_receiver);
		pos_sync_data_sz += pos_sync_data_sender.diff_buffer_sz() + 1;
		sync_pos = pos_sync_data_receiver.latest_snapshot();
		sync_raw_sz += sizeof(T) * 3;
		
		auto speed_dis = sync_speed.distance(new_speed);
		auto pos_dis = sync_pos.distance(new_pos);
		
		if (speed_dis > 2 * speed_scale || pos_dis > 2 * position_scale)
		{
			std::cout << "sync fail" << std::endl;
			exit(1);
		}
		pre_speed = new_speed;
		pos = new_pos;
		if (pos.distance(cur_dest) < radius)
		{
			dest_idx += 1;
			std::cout <<"iter "<< i<< " sync_raw_sz " << sync_raw_sz << " pos_sync_data_sz " << pos_sync_data_sz << " speed sync sz " << speed_sync_data_sz << std::endl;
			if (dest_idx == corridor.size())
			{
				break;
			}
			i = 0;
			sync_raw_sz = 0;
			pos_sync_data_sz = 0;
			speed_sync_data_sz = 0;
		}
	}
	

}
template <typename T>
std::vector<basic_point<T>> create_corridor_for_cube()
{
	std::vector<basic_point<T>> corridor;
	std::array<basic_point<T>, 8> cube;
	cube[0] = basic_point<T>(1, -1, -1);
	cube[1] = basic_point<T>(1, 1, -1);
	cube[2] = basic_point<T>(-1, 1, -1);
	cube[3] = basic_point<T>(-1, -1, -1);
	cube[4] = basic_point<T>(1, -1, 1);
	cube[5] = basic_point<T>(1, 1, 1);
	cube[6] = basic_point<T>(-1, 1, 1);
	cube[7] = basic_point<T>(-1, -1, 1);
	for (int i = 0; i < 8; i++)
	{
		cube[i] *= 100;
	}
	for (int j = 0; j < 13; j++)
	{
		corridor.push_back(cube[(j * 47) % 8]);
	}
	return corridor;
}

template <typename T>
std::vector<basic_point<T>> create_corridor_for_line()
{
	std::vector<basic_point<T>> corridor;
	corridor.push_back(basic_point<T>(-100, -100, -100));
	corridor.push_back(basic_point<T>(100, 100, 100));
	return corridor;
}
template <typename T>
void test(T max_speed, T max_accelerate, T radius)
{
	test_line<T>(max_accelerate, max_speed, create_corridor_for_line<T>(), radius, 0.01, 0.01);
}
int main()
{

	std::cout << "test float" << std::endl;
	test<float>(10, 4, 1);
	test<float>(10, 2, 1);
	test<float>(5, 2, 1);
	std::cout << "test double" << std::endl;
	test<double>(10, 4, 1);
	test<double>(10, 2, 1);
	test<double>(5, 2, 1);
}