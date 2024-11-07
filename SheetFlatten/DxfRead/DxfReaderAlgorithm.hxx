#ifndef _DXF_READER_ALGORITHM_4048D6300DC049F284F69DEE6C8A7BCA
#define _DXF_READER_ALGORITHM_4048D6300DC049F284F69DEE6C8A7BCA

#include <cassert>

namespace dxf{
	namespace algorithm {
		typedef struct Point
		{
			double x;
			double y;
		} Point;

		inline double min(double x, double y)
		{
			return x < y ? x : y;
		}

		inline double max(double x, double y)
		{
			return x > y ? x : y;
		}

		inline bool IsRectCross(const Point &p1, const Point &p2, const Point &q1, const Point &q2)
		{
			bool ret = min(p1.x, p2.x) <= max(q1.x, q2.x) &&
				min(q1.x, q2.x) <= max(p1.x, p2.x) &&
				min(p1.y, p2.y) <= max(q1.y, q2.y) &&
				min(q1.y, q2.y) <= max(p1.y, p2.y);
			return ret;
		}


		inline bool IsLineSegmentCross(const Point &P1, const Point &P2, const Point &Q1, const Point &Q2)
		{
			if (
				((Q1.x - P1.x)*(Q1.y - Q2.y) - (Q1.y - P1.y)*(Q1.x - Q2.x)) * ((Q1.x - P2.x)*(Q1.y - Q2.y) - (Q1.y - P2.y)*(Q1.x - Q2.x)) < 0 ||
				((P1.x - Q1.x)*(P1.y - P2.y) - (P1.y - Q1.y)*(P1.x - P2.x)) * ((P1.x - Q2.x)*(P1.y - P2.y) - (P1.y - Q2.y)*(P1.x - P2.x)) < 0
				)
				return true;
			else
				return false;
		}

		inline bool GetCrossPoint(const Point &p1, const Point &p2, const Point &q1, const Point &q2, double &x, double &y)
		{
			if (IsRectCross(p1, p2, q1, q2))
			{
				if (IsLineSegmentCross(p1, p2, q1, q2))
				{
					double tmpLeft, tmpRight;
					tmpLeft = (q2.x - q1.x) * (p1.y - p2.y) - (p2.x - p1.x) * (q1.y - q2.y);
					tmpRight = (p1.y - q1.y) * (p2.x - p1.x) * (q2.x - q1.x) + q1.x * (q2.y - q1.y) * (p2.x - p1.x) - p1.x * (p2.y - p1.y) * (q2.x - q1.x);

					x = ((double)tmpRight / (double)tmpLeft);

					tmpLeft = (p1.x - p2.x) * (q2.y - q1.y) - (p2.y - p1.y) * (q1.x - q2.x);
					tmpRight = p2.y * (p1.x - p2.x) * (q2.y - q1.y) + (q2.x - p2.x) * (q2.y - q1.y) * (p1.y - p2.y) - q2.y * (q1.x - q2.x) * (p2.y - p1.y);
					y = ((double)tmpRight / (double)tmpLeft);
					return true;
				}
			}
			return false;
		}

		//function: fork_product.
		template<typename vector3d_t>
		vector3d_t fork_product(const vector3d_t left, const vector3d_t right)
		{
			vector3d_t ret;

			ret.x = left.y * right.z - left.z * right.y;
			ret.y = left.z * right.x - left.x * right.z;
			ret.z = left.x * right.y - left.y * right.x;

			return ret;
		}

		// function: get_model.
		template<typename vector3d_t>
		double get_model(const vector3d_t t)
		{
			return sqrt(t.x * t.x + t.y * t.y + t.z * t.z);
		}

		
		// function: normalize_vec.
		template<typename vector3d_t>
		vector3d_t normalize_vec(const vector3d_t t)
		{
			vector3d_t ret = t;

			double model = dxf::algorithm::get_model(t);
			assert(model > 1e-10);

			ret.x /= model;
			ret.y /= model;
			ret.z /= model;

			return ret;
		}


	}
}

#endif // !_DXF_READER_ALGORITHM_4048D6300DC049F284F69DEE6C8A7BCA

