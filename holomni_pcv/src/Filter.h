/* =======================================================================
   (c) 2006, Robert Holmberg

   PROPRIETARY and CONFIDENTIAL

   This file contains source code that constitutes proprietary and
   confidential information owned by Robert Holmberg (RAH) that is
   considered a trade secret of RAH.

   RAH retains the title, ownership and intellectual property rights
   in and to the Software and all subsequent copies regardless of the
   form or media.  Copying or distributing any portion of this file
   without the written permission of RAH is prohibited.

   Use of this code is governed by the license agreement,
   confidentiality agreement, and/or other agreement under which it
   was distributed. When conflicts or ambiguities exist between this
   header and the written agreement, the agreement supersedes this header.
   ========================================================================*/

#ifndef _Filter_h_
#define _Filter_h_

#include "PrGlobalDefn.h"
#include <string>

class PrVector;

class Filter
{
public:
  Filter();
  Filter(const Filter &);
  Filter &operator=(const Filter &);
 ~Filter();

  // Use this after a filter has been initialized to filter the data
  void Filt(const PrVector &in, PrVector &out);


  // General digital filter: z_num/zden
  void Z_NumDen(PrVector const &r0,
                PrVector const &z_num, PrVector const &z_den,
                PrVector const &f0 );

  // Filters below are digital equivalents using Tustin's method
  void LowPass   // w/(s+w)
       (PrVector const &r0, 
        Float const sampFreq, Float cutOffFreq);
  void D_LowPass  // (s*w)/(s+w)
       (PrVector const &r0, Float sampFreq, Float cutOffFreq);
  void D         //  s
       (PrVector const &r0, Float sampFreq);
  void LeadLag   // (s+w1)/[(s+w2)(s+w3)]
       (PrVector const &r0, Float sampFreq,
        Float f1, Float f2, Float f3 );
  void Unity();  //  1
  void D_LowPass_Reset(const PrVector &r0);
  void LowPass_Reset(const PrVector &r0);
  void D_LowPass3_Reset(const PrVector &r0);
  void D_LowPass3(const PrVector &r0, Float sampFreq, Float cutOffFreq);
  void Normalize();
private:
  int size_;
  int order_;
  
  PrVector *a_;
  PrVector *b_;

  PrVector **f_;
  PrVector **r_;

  void Allocate( int size, int order );
  void Purge();
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

typedef double mReal;
using namespace std;

#define MAXFILTERTERMS 128
	class M3DFilter2
	{
		public:
			M3DFilter2();
			
			void Clear(); //Clear the history of the filter
			int Coefficients(int N,mReal *A,mReal *B); //Set the coefficients of the filter.
			mReal Step(mReal x_0); //evaluate the filter			
			void Diff_Butterworth_Filter(int order, mReal cutoff_freq, mReal sample_period);
		protected:
			enum {NONE, BUTTERWORTH, DIFF_BUTTERWORTH, LEAST_SQUARES_ESTIMATE, IDENTITY, AVERAGE};
			int Nterms; //Number of terms in the calculation
			int buffer_idx; //Present start of the x & y history circular buffers
			mReal a[MAXFILTERTERMS]; //y filter coefficients in reverse order.
			mReal b[MAXFILTERTERMS]; //x filter coefficients in reverse order. 
			mReal x[MAXFILTERTERMS]; //independent value history (circular buffer)
			mReal y[MAXFILTERTERMS]; //dependent value history (circular buffer)
			
			
			
			string type;
			int order;
			mReal cutoff_freq;
	};
	

#endif // _Filter_h_
