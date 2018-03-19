#include <mmuav_control/median_filter.h>
#include <algorithm>
#include <iostream>

median_filter::median_filter(void)
{

}

void median_filter::init(uint32_t size)
{
	if (size % 2 != 0)
		size_ = size;
	else
		size_ = 1;

	measurements_ = new float[size_];

	for (int i = 0; i < size_; i++) measurements_[i] = 0;
}

median_filter::~median_filter()
{
	delete[] measurements_;
}

float median_filter::filter(float data)
{
	int i = 0;
	float *sorted_measurements_;
	float filtered_data = 0;

	sorted_measurements_ = new float[size_];

	for (i = size_-1; i > 0; i--)
	{
		measurements_[i] = measurements_[i-1];
	}
	measurements_[0] = data;

	std::copy(measurements_, measurements_+size_, sorted_measurements_);

	std::sort(sorted_measurements_, sorted_measurements_ + size_);

	filtered_data = sorted_measurements_[int(size_/2)];

	delete[] sorted_measurements_;

	return filtered_data;
}