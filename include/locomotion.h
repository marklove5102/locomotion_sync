#pragma once
#include <cstdint>
#include <array>

namespace spiritsaway::locomotion_sync
{
	template <typename T, std::uint8_t N>
	class vec_sync_data
	{
		std::uint8_t m_flags = 0;
		std::array<std::int8_t, N * sizeof(T)> m_diff_buffer; // 用于存放diff或者full同步的数据
		std::array<T, N> m_latest_snapshot; // 用于存放最新的位置信息 full/diff/replay之后都会更新这个数据
		const T m_scale; // 用于将浮点数缩放为整数进行同步 作为精度控制手段
	public:
		vec_sync_data(T in_scale)
			: m_scale(in_scale)
		{
			static_assert(N <= 4);
		}
		std::uint8_t diff_buffer_sz()const // 获取当前m_diff_buffer中存放的数据大小
		{

			std::uint8_t result = 0;
			std::uint8_t remain_flag = m_flags;
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

		const std::array<T, N>& latest_snapshot() const
		{
			return m_latest_snapshot;
		}
	private:
		template <typename D>
		void encode_to_data(std::uint8_t i, int diff_v, std::uint8_t& data_fill_next)
		{
			m_flags |= (sizeof(D) << (i * 2));
			D diff_v_d = D(diff_v);
			std::int8_t* diff_v_d_ptr = reinterpret_cast<std::int8_t*>(&diff_v_d);
			std::copy(diff_v_d_ptr, diff_v_d_ptr + sizeof(D), m_diff_buffer.begin() + data_fill_next);
			data_fill_next += sizeof(D);
			m_latest_snapshot[i] += diff_v * m_scale;
		}

		template <typename D>
		void decode_from_data(std::uint8_t i,  std::uint8_t& next_data_idx)
		{
			D diff_v;
			std::int8_t* diff_v_ptr = reinterpret_cast<std::int8_t*>(&diff_v);
			std::copy(m_diff_buffer.data() + next_data_idx, m_diff_buffer.data() + next_data_idx + sizeof(D), diff_v_ptr);
			T cur_diff = diff_v * m_scale;
			m_latest_snapshot[i] += cur_diff;
			next_data_idx += sizeof(D);
		}
		const std::array<T, N>& replay()
		{
			std::uint8_t next_data_idx = 0;
			for (std::uint8_t i = 0; i < N; i++)
			{
				std::uint8_t cur_flag = (m_flags >> (i * 2)) & 0x3;
				switch (cur_flag)
				{
				case 0x1:
				{
					decode_from_data<std::int8_t>(i, next_data_idx);
					break;
				}
				case 0x2:
				{
					decode_from_data<std::int16_t>(i, next_data_idx);
					break;
				}
				case 0x3:
				{
					std::int8_t* new_v_ptr = reinterpret_cast<std::int8_t*>(m_latest_snapshot.data() + i);
					std::copy(m_diff_buffer.data() + next_data_idx, m_diff_buffer.data() + next_data_idx + sizeof(T), new_v_ptr);
					next_data_idx += sizeof(T);
					break;
				}
				default:
					break;
				}

			}
			return m_latest_snapshot;
		}
	public:
		void diff(const std::array<T, N>& new_data)
		{
			std::uint8_t data_fill_next = 0;
			m_flags = 0;
			for (int i = 0; i < N; i++)
			{
				T old_v = m_latest_snapshot[i];
				T new_v = new_data[i];
				int diff_v = int((new_v - old_v) / m_scale);
				if (diff_v > std::numeric_limits<std::int16_t>::max() || diff_v < std::numeric_limits<std::int16_t>::min())
				{
					// 双字节的diff覆盖不住 直接全量同步
					m_flags |= (0x3 << (i * 2));
					std::int8_t* new_v_ptr = reinterpret_cast<std::int8_t*>(&new_v);
					std::copy(new_v_ptr, new_v_ptr + sizeof(T), m_diff_buffer.begin() + data_fill_next);
					data_fill_next += sizeof(T);
					m_latest_snapshot[i] = new_v;
				}
				else if (diff_v > std::numeric_limits<std::int8_t>::max() || diff_v < std::numeric_limits<std::int8_t>::min())
				{
					// 单字节diff覆盖不住 但是双字节diff能覆盖 则使用双字节同步
					encode_to_data<std::int16_t>(i, diff_v, data_fill_next);
				}
				else if (diff_v != 0)
				{
					// 如果能被单字节覆盖 则使用单字节同步
					encode_to_data<std::int8_t>(i, diff_v, data_fill_next);
				}
				// 如果0 则不需要同步

			}
		}
		void full(const std::array<T, N>& new_data)
		{
			m_flags = 0;
			for (std::uint8_t i = 0; i < N; i++)
			{
				T new_v = new_data[i];

				m_flags |= (0x3 << (i * 2));
				std::int8_t* new_v_ptr = reinterpret_cast<std::int8_t*>(&new_v);
				std::copy(new_v_ptr, new_v_ptr + sizeof(T), m_diff_buffer.begin() + i * sizeof(T));
			}
			m_latest_snapshot = new_data;
		}


		// 编码到指定的外部buffer 返回实际编码的字节数
		std::uint8_t encode(std::int8_t* buffer) const
		{
			buffer[0] = *reinterpret_cast<const std::int8_t*>(&m_flags);
			auto cur_data_sz = diff_buffer_sz();
			std::copy(m_diff_buffer.begin(), m_diff_buffer.begin() + cur_data_sz, buffer + 1);
			return cur_data_sz + 1;
		}
		// 从指定的外部buffer解码 remain_sz为外部buffer剩余的字节数 返回实际解码的字节数
		std::uint8_t decode(const std::int8_t* buffer, std::uint32_t remain_sz)
		{
			if (remain_sz == 0)
			{
				return 0;
			}
			m_flags = *reinterpret_cast<const std::uint8_t*>(buffer);
			auto cur_data_sz = diff_buffer_sz();
			if (remain_sz < std::uint32_t(cur_data_sz + 1))
			{
				return 0;
			}
			std::copy(buffer + 1, buffer + 1 + cur_data_sz, m_diff_buffer.begin());
			replay();
			return cur_data_sz + 1;
		}
	};

	using vec3_sync_double = vec_sync_data<double, 3>;
	using vec3_sync_float = vec_sync_data<float, 3>;
	using vec4_sync_double = vec_sync_data<double, 4>;
	using vec4_sync_float = vec_sync_data<float, 4>;
}