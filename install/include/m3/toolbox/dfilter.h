/* 
M3 -- Meka Robotics Robot Components
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.


*/
#ifndef M3_DFILTER_H
#define M3_DFILTER_H

#include "m3rt/base/toolbox.h"
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/QR>
#include "inttypes.h"

namespace m3
{

	using namespace std;
	using namespace m3rt;
	using namespace Eigen;
	
	class M3DiffAperiodic
	{
	  public:
	    M3DiffAperiodic() : previous_qraw(0.0), previous_timestamp(0),tmp_cnt(0),previous_vel(0) {}
	    mReal Step(mReal qraw, uint64_t timestamp);
	  private:
	    mReal previous_qraw, previous_vel;
	    uint64_t previous_timestamp;
	    int tmp_cnt;
	    
	};
	
	//Does a polynomial least squares fit to the last num_data_pts
	//Then takes velocity/acceleration calculation of poly
	class M3PolyLeastSquares
	{
		public:
		// weighting: how much emphasis to place on the newer readings versus old.
		//		Best between 1.0-2.0.  Low values are more convervative.
		// num_data_pts:  between 100-400.  More is better, but increases computational time.
		// n(poly_order):  Default is 3.
		// period:  time period between readings.
			M3PolyLeastSquares(){}
			void Init(int poly_order, int num_data_pts, mReal period, mReal weighting);
			void Step(mReal new_qy);
			mReal GetVal(){return y_smooth;}
			mReal GetValDot(){return ydot_smooth;} 	
			mReal GetValDotDot(){return ydotdot_smooth;}
		private:
			int m;
			int n;
			mReal T;
			mReal weight;
			VectorXf qy;
			VectorXf wy;
			mReal y_smooth;
			mReal ydot_smooth;
			mReal ydotdot_smooth;
			MatrixXf w;		
			MatrixXf M,MM;
			// Eigen3 port
			HouseholderQR<MatrixXf> qr;
			//MatrixXf Q;
			//MatrixXf R;
			VectorXf x;
	};
	
	
	
	
/* 
	Implements a IIR digital filter calculation.
  
	y[T] = b[n-1]*x[T]+b[n-2]*x[T-1]+...+b[0]*x[T-n]
	- (a[n-2]*y[T-1]+a[n-3]*y[T-2]+...+a[0]*y[T-n])
  
	Where T is the present time interval; n is the number of terms; x & y are sampled at descrete points
	in time with a constant period. Note that b[0],a[0] are coefficients for the 
	oldest sample, which may be counter-intuitive to book algorithms.
  
	Coefficients b[0..n] and a[1..n] must be predivided by a[0] if it does not equal 1.0
	The algorithm does no use a[0] for calculation efficiency. The value stored at a[0] 
	is not affected by the algorithm.

  Usage: 
	For the butterworth filters, you will need to choose a cutoff frequency. The
	lower the cutoff_freq, the smoother the estimated velocity, at the cost of delay.
    
	For a starting value, select the cutoff frequency to be equal to the lowest 
	response frequency you need. Ex: 1kHz sample freq & 30Hz cutoff Freq for
	a haptic device. (Humans can affect input up to about 8Hz).
  
*/

#define MAXFILTERTERMS 128
	class M3DFilter
	{
		public:
			M3DFilter():_a(MAXFILTERTERMS,0.0),_b(MAXFILTERTERMS,0.0),_x(MAXFILTERTERMS,0.0),_y(MAXFILTERTERMS,0.0),Nterms(MAXFILTERTERMS),order(2),buffer_idx(0),T(1.0/RT_TASK_FREQUENCY)
			{
			  
			}
			void Dump(); //Dump the contents of the class to cout
			void Clear(); //Clear the history of the filter
			int Coefficients(int N,std::vector<mReal> A,std::vector<mReal> B); //Set the coefficients of the filter.
			mReal Step(mReal x_0); //evaluate the filter
			mReal Step(mReal x_0,int N); //evaluate the filter
			void UpdateFilter();
			bool ReadConfig(const YAML::Node & doc);
			int GetN()const{return Nterms;}
			int GetOrder()const{return order;}
			//int GetType()const{return type;}
			mReal GetCutOffFreq(){return cutoff_freq;}
			void SetN(int N){
			  if(N>MAXFILTERTERMS){
			    N = MAXFILTERTERMS;
			    M3_INFO("N set to max (%d)",MAXFILTERTERMS);
			  }
			  Nterms = N;
			  UpdateFilter();}
			void SetCutoff_freq(mReal cutoff_freq){this->cutoff_freq = cutoff_freq;UpdateFilter();} 
			void SetOrder(int order){this->order = order;UpdateFilter();}
			void SetType(string t);
                       const char * GetType();
			
		protected:
			enum FILTER_TYPE{NONE, BUTTERWORTH, DIFF_BUTTERWORTH, LEAST_SQUARES_ESTIMATE, IDENTITY, AVERAGE}; // is inside actuator.pb.h
			int Nterms; //Number of terms in the calculation
			int buffer_idx; //Present start of the x & y history circular buffers
			void      Butterworth_Filter();
			void Diff_Butterworth_Filter();
			void Least_Squares_Estimate();
			void Identity_Filter();
			void Average_Filter();
			int type;
			int order;
			mReal cutoff_freq;
			mReal T; //Period
		private:
			std::vector<mReal> _a; //y filter coefficients in reverse order.
			std::vector<mReal> _b; //x filter coefficients in reverse order. 
			std::vector<mReal> _x; //independent value history (circular buffer)
			std::vector<mReal> _y; //dependent value history (circular buffer)
			
			
	};
	
	
	class M3SensorFilter
	{
		public:
			M3SensorFilter():x(0),xdot(0),xdotdot(0),type(NONE){}
			~M3SensorFilter(){}
			bool ReadConfig(const YAML::Node & doc);
			void Step(mReal qraw, mReal qdotraw);
			void Reset();
			mReal GetX(){return x;}
			mReal GetXDot(){return xdot;}
			mReal GetXDotDot(){return xdotdot;}
			// Setters for params
			M3DFilter* GetXdf(){return &x_df;}
			M3DFilter* GetXdotdf(){return &xdot_df;}
			M3DFilter* GetXdotdotdf(){return &xdotdot_df;}
			/*void SetXdf_N(int N){x_df.SetN(N);}
			void SetXdotdf_N(int N){xdot_df.SetN(N);}
			void SetXdotdotdf_N(int N){xdotdot_df.SetN(N);}
			
			void SetXdf_CutoffFreq(mReal cutoff_freq){x_df.SetCutoff_freq(cutoff_freq);}
			void SetXdotdf_CutoffFreq(mReal cutoff_freq){xdot_df.SetCutoff_freq(cutoff_freq);}
			void SetXdotdotdf_CutoffFreq(mReal cutoff_freq){xdotdot_df.SetCutoff_freq(cutoff_freq);}
			
			void SetXdf_Order(int order){x_df.SetOrder(order);}
			void SetXdotdf_Order(int order){xdot_df.SetOrder(order);}
			void SetXdotdotdf_Order(int order){xdotdot_df.SetOrder(order);}*/
		protected: 
			enum {NONE, DF_CHAIN, POLY_LEAST_SQUARES, DF_POLY_LEAST_SQUARES, DF_APERIODIC};
			M3DiffAperiodic xdot_aperiodic;
			M3PolyLeastSquares pls;
			M3DFilter x_df;
			M3DFilter xdot_df;
			M3DFilter xdotdot_df;
			int type;
			mReal x,xdot,xdotdot;
	};
	
	//This wrapper just supports the older config file naming. Use M3SensorFilter for new designs.
	class M3JointFilter : public M3SensorFilter
	{
	  public:
	  mReal GetTheta(){return GetX();}
	  mReal GetThetaDot(){return GetXDot();}
	  mReal GetThetaDotDot(){return GetXDotDot();}
	  bool ReadConfig(const YAML::Node & doc);
	};
	
}//namespace

#endif