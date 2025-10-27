#pragma once
#include <cstdint>
#include <array>

namespace spiritsaway::locomotion_sync
{
	template <typename T, std::uint8_t N>
	class vec_sync_data
	{
		std::uint8_t flags = 0;
		std::array<std::int8_t, N * sizeof(T)> diff_buffer; // 用于存放diff或者full同步的数据
		std::array<T, N> lastest_snapshot; // 用于存放最新的位置信息
	public:
		vec_sync_data()
		{
			static_assert(N <= 4);
		}
		std::uint8_t data_sz()const
		{

			std::uint8_t result = 1;
			std::uint8_t remain_flag = flags;
			while (remain_flag)
			{
				std::uint8_t test_flag = remain_flag % 4;
				remain_flag = remain_flag / 4;
				if (test_flag == 0x3)
				{
					// 全量同步
					result += sizeof(T);
				}
				else
				{
					// 增量同步 0x0为相等不需要同步 0x1代表diff所需一个字节 0x2代表diff所需2个字节
					result += test_flag;
				}
			}
			return result;
		}
		void to_big_endian(std::int8_t* data, std::uint32_t byte_num) const
		{
			(void)data;
			(void)byte_num;
		}
		void from_big_endian(std::int8_t* data, std::uint32_t byte_num) const
		{
			(void)data;
			(void)byte_num;
		}
		const std::array<T, N>& get_lastest_snapshot() const
		{
			return lastest_snapshot;
		}
		template <typename D>
		void encode_to_data(std::uint8_t i, int diff_v, std::uint8_t& data_fill_next, float scale)
		{
			flags |= (sizeof(D) << (i * 2));
			D diff_v_d = D(diff_v);
			std::int8_t* diff_v_d_ptr = reinterpret_cast<std::int8_t*>(&diff_v_d);
			to_big_endian(diff_v_d_ptr, sizeof(D));
			std::copy(diff_v_d_ptr, diff_v_d_ptr + sizeof(D), diff_buffer.begin() + data_fill_next);
			data_fill_next += sizeof(D);
			lastest_snapshot[i] += diff_v * scale;
		}

		template <typename D>
		void decode_from_data(std::uint8_t i, T* dest, T scale, std::uint8_t& next_data_idx) const
		{
			D diff_v;
			std::int8_t* diff_v_ptr = reinterpret_cast<std::int8_t*>(&diff_v);
			std::copy(diff_buffer.data() + next_data_idx, diff_buffer.data() + next_data_idx + sizeof(D), diff_v_ptr);
			from_big_endian(diff_v_ptr, sizeof(D));
			T cur_diff = diff_v * scale;
			dest[i] += cur_diff;
			next_data_idx += sizeof(D);
		}
		void diff(const T* new_data, T scale)
		{
			std::uint8_t data_fill_next = 0;
			flags = 0;
			for (int i = 0; i < N; i++)
			{
				T old_v = lastest_snapshot[i];
				T new_v = new_data[i];
				int diff_v = int((new_v - old_v) / scale);
				if (diff_v > std::numeric_limits<std::int16_t>::max() || diff_v < std::numeric_limits<std::int16_t>::min())
				{
					// 双字节的diff覆盖不住 直接全量同步
					flags |= (0x3 << (i * 2));
					std::int8_t* new_v_ptr = reinterpret_cast<std::int8_t*>(&new_v);
					to_big_endian(new_v_ptr, sizeof(T));
					std::copy(new_v_ptr, new_v_ptr + sizeof(T), diff_buffer.begin() + data_fill_next);
					data_fill_next += sizeof(T);
					lastest_snapshot[i] = new_v;
				}
				else if (diff_v > std::numeric_limits<std::int8_t>::max() || diff_v < std::numeric_limits<std::int8_t>::min())
				{
					// 单字节diff覆盖不住 但是双字节diff能覆盖 则使用双字节同步
					encode_to_data<std::int16_t>(i, diff_v, data_fill_next, scale);
				}
				else if (diff_v != 0)
				{
					// 如果能被单字节覆盖 则使用单字节同步
					encode_to_data<std::int8_t>(i, diff_v, data_fill_next, scale);
				}
				// 如果0 则不需要同步

			}
		}
		void full(const T* new_data)
		{
			flags = 0;
			for (std::uint8_t i = 0; i < N; i++)
			{
				T new_v = new_data[i];

				flags |= (0x3 << (i * 2));
				std::int8_t* new_v_ptr = reinterpret_cast<std::int8_t*>(&new_v);
				to_big_endian(new_v_ptr, sizeof(T));
				std::copy(new_v_ptr, new_v_ptr + sizeof(T), diff_buffer.begin() + i * sizeof(T));
			}
			std::copy(new_data, new_data + N, lastest_snapshot.data());
		}
		void replay(T* dest, T scale) const
		{
			std::uint8_t next_data_idx = 0;
			for (std::uint8_t i = 0; i < N; i++)
			{
				std::uint8_t cur_flag = (flags >> (i * 2)) & 0x3;
				switch (cur_flag)
				{
				case 0x1:
				{
					decode_from_data<std::int8_t>(i, dest, scale, next_data_idx);
					break;
				}
				case 0x2:
				{
					decode_from_data<std::int16_t>(i, dest, scale, next_data_idx);
					break;
				}
				case 0x3:
				{
					std::int8_t* new_v_ptr = reinterpret_cast<std::int8_t*>(dest + i);
					std::copy(diff_buffer.data() + next_data_idx, diff_buffer.data() + next_data_idx + sizeof(T), new_v_ptr);
					from_big_endian(new_v_ptr, sizeof(T));
					next_data_idx += sizeof(T);
					break;
				}
				default:
					break;
				}

			}
		}
		std::uint8_t encode(std::int8_t* buffer) const
		{
			buffer[0] = *reinterpret_cast<const std::int8_t*>(&flags);
			auto cur_data_sz = data_sz();
			std::copy(diff_buffer.begin(), diff_buffer.begin() + data_sz(), buffer + 1);
			return cur_data_sz + 1;
		}
		std::uint8_t decode(const std::int8_t* buffer, std::uint32_t remain_sz)
		{
			if (remain_sz == 0)
			{
				return 0;
			}
			flags = *reinterpret_cast<const std::uint8_t*>(buffer);
			auto cur_data_sz = data_sz();
			if (remain_sz < std::uint32_t(cur_data_sz + 1))
			{
				return 0;
			}
			std::copy(buffer + 1, buffer + 1 + cur_data_sz, diff_buffer.begin());
			return cur_data_sz + 1;
		}
	};

	using vec3_sync_double = vec_sync_data<double, 3>;
	using vec3_sync_float = vec_sync_data<float, 3>;
	using vec4_sync_double = vec_sync_data<double, 4>;
	using vec4_sync_float = vec_sync_data<float, 4>;
}