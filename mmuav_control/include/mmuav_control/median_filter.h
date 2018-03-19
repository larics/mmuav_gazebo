#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include <stdint.h>

class median_filter{
	private:
		float *measurements_;
		uint32_t size_;
	public:
		median_filter();
		void init(uint32_t size);
		float filter(float data);
		~median_filter();
};

#endif