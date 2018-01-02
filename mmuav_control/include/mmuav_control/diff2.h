#ifndef DIFF2_H
#define DIFF2_H

class diff2{
	private:
		enum DiffType { DOUBLE_REAL, SINGLE_REAL, COMPLEX_CONJUGATE};
		
		float a1_, a0_, r1_, r2_;
		float alpha_, beta_;

		bool diff2_init_;
		diff2::DiffType diff2_type_;


	public:
		diff2(void);
		float dsolve(float x, float *cond);
		void init(float a0, float a1);

};

#endif