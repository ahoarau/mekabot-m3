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

#include "Filter.h"
#include "matrix/PrVector.h"

#ifndef M_PI
#define TWOPI (2.0*3.14159265358979323846264338327950288419716939937510582)
#else
#define TWOPI (2*M_PI)
#endif //M_PI


Filter::Filter() 
  : size_(0),order_(0),a_(NULL),b_(NULL),f_(NULL),r_(NULL)
{ }

Filter::Filter(const Filter &src)
{
  Allocate(src.size_,src.order_);
  operator=(src);
}


Filter &
Filter::operator=(const Filter &src)
{
  (*a_) = *src.a_;
  (*b_) = *src.b_;
  for(int i=0; i<order_+1 ; i++ )
  { *f_[i] = *src.f_[i];
    *r_[i] = *src.r_[i];
  }
  return (*this);
}


void
Filter::Allocate( int size,  int order)
// Allocates memory for filter constants and aging the data
{
  if( size_>0 ) // PURGE DATA, IF CURRENTLY HAS DATA
    Purge();

  size_ = size;
  order_ = order;

  a_ = new PrVector(order_+1);
  b_ = new PrVector(order_+1);

  f_ = new PrVector * [order_+1];
  r_ = new PrVector * [order_+1];
  for(int i=0; i<order_+1 ; i++ )
  { f_[i] = new PrVector(size_);
    r_[i] = new PrVector(size_);
  }
}

void
Filter::Purge()
{
  delete a_;
  delete b_;
  for(int i=0; i<order_+1 ; i++ )
  { delete f_[i];
    delete r_[i];
  }
  delete [] f_;
  delete [] r_; 

  size_ = 0;
  order_ = 0;
}


Filter::~Filter()
// Destructor
{
  Purge();
}


void
Filter::Filt(const PrVector &in, PrVector &out)
// Use this after assigning a filter type to filter the data
{
  int i;

  if( order_ == 0 )  // COPY AND BAIL OUT
  { out = in;
    return;
  }

  *r_[0] = in;  // NEED THIS ASSIGNMENT FOR AGEING TOO
  *f_[0] = *r_[0] * (*b_)[0];

  for( i=1; i<order_+1 ; i++) // NOTE: a_[0] NOT USED. (NORMALIZED)
  { 
    *f_[0] +=   *r_[i] * (*b_)[i] - *f_[i] * (*a_)[i];
  }
  // AGE THE VALUES
  for( i=0; i<order_ ; i++)
  { *f_[i+1] = *f_[i];
    *r_[i+1] = *r_[i];
  }
  out = *f_[0];
}

void
Filter::Z_NumDen(PrVector const &r0, 
                 PrVector const &z_num, PrVector const &z_den,
                 PrVector const &f0 )
// general digital transfer function: z_num/z_den
{
  Allocate( r0.size(), z_den.size()-1 );

  // COPY COEFF VECTORS
  for(int i=0; i<z_num.size(); i++)
    (*b_)[i] = z_num[i];  // FOR order(num) <= order(den)
  *a_ = z_den; // COPY DENOMINATOR

  for(int i=1; i<order_+1; i++)
  { *r_[i] = r0; //Store initial value
    *f_[i] = f0; //Store initial value
  }
}


void
Filter::LowPass(const PrVector &r0, Float sampFreq, Float cutOffFreq)
// w/(s+w)
{
  Allocate( r0.size(), 1 );
  Float T = 1.0/sampFreq;

  Float w = TWOPI*cutOffFreq;
  w = (2/T)*tan(w*T/2);  // WARPING TO MATCH CUTOFF FREQ

  Float p = (T*w/2)+1;
  Float m = (T*w/2)-1;

  (*b_)[0] =  (T*w/2)/p;
  (*b_)[1] =  (*b_)[0];
  (*a_)[0] =  1;
  (*a_)[1] =  m/p;

  LowPass_Reset(r0);  
}

void Filter::LowPass_Reset(const PrVector &r0)
{
  *r_[1] = r0; //Store initial value
  *f_[1] = r0; //Store initial value
}



void Filter::D_LowPass_Reset(const PrVector &r0)
{
  *r_[1] = r0;   //Store initial value
   f_[1]->zero(); //Store initial value
}


void Filter::D_LowPass3_Reset(const PrVector &r0)
{
  *r_[1] = r0;   //Store initial value
  *r_[2] = r0;   //Store initial value
  *r_[3] = r0;   //Store initial value
   f_[1]->zero(); //Store initial value
   f_[2]->zero(); //Store initial value
   f_[3]->zero(); //Store initial value
}


void
Filter::D_LowPass(const PrVector &r0, Float sampFreq, Float cutOffFreq)
// (s*w)/(s+w)
{
  Allocate( r0.size(), 1 );
  Float T = 1.0/sampFreq;

  Float w = TWOPI*cutOffFreq;
  w = (2/T)*tan(w*T/2);  // WARPING TO MATCH CUTOFF FREQ

  (*b_)[0] =  2*w/(w*T+2);
  (*b_)[1] = -(*b_)[0];
  (*a_)[0] =  1;
  (*a_)[1] =  (w*T-2)/(w*T+2);

  D_LowPass_Reset(r0);
}

void
Filter::D_LowPass3(const PrVector &r0, Float sampFreq, Float cutOffFreq)
// (s*w)/(s+w)
{
  Allocate( r0.size(), 3 );
  Float T = 1.0/sampFreq;

  Float w = TWOPI*cutOffFreq;
  w = (2/T)*tan(w*T/2);  // WARPING TO MATCH CUTOFF FREQ
  
  Float st = 2/w;
  Float nd = st*st/T;
  Float scale;
  
  scale = 1/(T+st+nd);

  (*a_)[3] = 0.0;
  (*a_)[2] = (st-nd)*scale;
  (*a_)[1] = (3*T-st-nd) * scale;
  (*a_)[0] = (nd-st) * scale;

  (*b_)[3] = scale;
  (*b_)[2] = scale;
  (*b_)[1] = -scale;
  (*b_)[0] = -scale;

/*
  (*b_)[0] =  2*w/(w*T+2);
  (*b_)[1] = -(*b_)[0];
  (*a_)[0] =  1;
  (*a_)[1] =  (w*T-2)/(w*T+2);*/

  // MUST NORMALIZE a[0]=1 To use Filt() method properly
  Normalize();

  D_LowPass3_Reset(r0);
}


void
Filter::Normalize()
{
	// Normalize the filter coefficients so that a[0]=1.
	// The Filt() function assumes the coeffs are normalized.
	Float ai;
	ai = 1./(*a_)[0];
	for (int i = 0; i < b_->size(); i++){
		(*b_)[i] *= ai;
		(*a_)[i] *= ai;
	}
}


void
Filter::D(const PrVector &r0, Float sampFreq)
//  s
{
  Allocate( r0.size(), 1 );
  Float T = 1.0/sampFreq;

  (*b_)[0] =  1.0/T;
  (*b_)[1] = -(*b_)[0];
  (*a_)[0] =  1;
  (*a_)[1] =  0;

  *r_[1] = r0;   //Store initial value
   f_[1]->zero(); //Store initial value
}


void
Filter::LeadLag(const PrVector &r0, Float sampFreq, 
                     Float f1, Float f2, Float f3 )
// [w2*w3/w1](s+w1)/[(s+w2)(s+w3)]
{
  Allocate( r0.size(), 2 );
  Float T = 1.0/sampFreq;

  Float w1 = TWOPI * f1;
  w1 = (2/T)*tan(w1*T/2);  // WARPING TO MATCH FREQ
  Float w2 = TWOPI * f2;
  w2 = (2/T)*tan(w2*T/2);  // WARPING TO MATCH FREQ
  Float w3 = TWOPI * f3;
  w3 = (2/T)*tan(w3*T/2);  // WARPING TO MATCH FREQ

  Float g0 = (w2*w3/w1); // UNITY DC GAIN
  Float p2 = T*w2/2 + 1;
  Float m2 = T*w2/2 - 1;
  Float p3 = T*w3/2 + 1;
  Float m3 = T*w3/2 - 1;
  Float pp = p2*p3;
  Float  c = g0*T/2/pp;

  (*b_)[0] =  c*(w1+1);
  (*b_)[1] =  c*w1;
  (*b_)[2] = -c;
  (*a_)[0] =  1;
  (*a_)[1] =  (m2*p3+p2*m3)/pp;
  (*a_)[2] =  m2*m3/pp;

  *r_[1] = r0; //Store initial value
  *r_[2] = r0; //Store initial value
  *f_[1] = r0; //Store initial value
  *f_[2] = r0; //Store initial value
}


void
Filter::Unity()   // COPIES INPUT TO OUTPUT
{
  Allocate( 1, 0 );
  (*b_)[0] = 1;
  (*a_)[0] = 1;
}

/*
	Creates a digitized butterworth filter plus differentiator using the
	bilinear transform. Oppenheim & Schafer pp450 (2nd ed)
  
	order: the order of the filter. 1, 2 or 3.
	cutoff_freq: cutoff frequency of the lowpass filter
	sample_period: time between samples.
*/
	void M3DFilter2::Diff_Butterworth_Filter(int order, mReal cutoff_freq, mReal sample_period)
	{
		mReal T = sample_period; //Shorthand for clearer code.
		mReal scale; //Scaling factor (really the a[1] value which everything gets divided by)

		mReal pi = 3.14159625;
  
  
		mReal a[4] = {0.0,0.0,0.0,0.0};
		mReal b[4] = {0.0,0.0,0.0,0.0};
  
		mReal wo = cutoff_freq*2.0*pi; //Omega naught (rad/sec)
		mReal w = 2/T*tan(wo*T/2); //Omega cutoff (frequency warping from analog to digital domain)
  
		if(order == 1) {
			mReal st = 1/w;
			scale = 1/(T/2+st);
    
			a[1] = 0.0;
			a[0] = (T/2-st)*scale;
    
			b[1] = scale;
			b[0] = -scale;
		} 
		else if(order == 2) {
			mReal st = sqrt(2.0)/w;
			scale = 1/(st*st/T + st + T/2.0);
    
			a[2] = 0.0;
			a[1] = (T-2*st*st/T) * scale;
			a[0] = (T/2 - st + st*st/T) * scale;
    
			b[2] = scale;
			b[1] = 0.0;
			b[0] = -scale;
		} 
		else if (order == 3){
			mReal st = 2/w;
			mReal nd = st*st/T;
  
			scale = 1/(T+st+nd);
     
			a[3] = 0.0;
			a[2] = (st-nd)*scale;
			a[1] = (3*T-st-nd) * scale;
			a[0] = (nd-st) * scale;
  
			b[3] = scale;
			b[2] = scale;
			b[1] = -scale;
			b[0] = -scale;
		}
		else 
		{
//			M3_INFO("Incorrect M3DFilter configuration for Diff_Butterworth_Filter\n");
		}
 
		Coefficients(order+1,a,b);
	}


/*
	Evaluates the digital filter, starting with the oldest values in the history
	and working towards the most recent. 

*/
	mReal M3DFilter2::Step(mReal x_0)
	{
		mReal retval;
		int i, start_idx;
  
		x[buffer_idx] = x_0;
		y[buffer_idx] = 0.0;
  
		start_idx = buffer_idx - Nterms + MAXFILTERTERMS + 1; //Precalc the index where the history data starts in the buffer.
  
		for (int n = 0; n < Nterms; n++){
			i = (start_idx + n) % (MAXFILTERTERMS);
    
			y[buffer_idx] += b[n]*x[i] - a[n]*y[i];
    
		}
  
		retval = y[buffer_idx];
  
		buffer_idx = (buffer_idx+1) % (MAXFILTERTERMS);
  
		return retval;
	}
	
		
	M3DFilter2::M3DFilter2()
	{
		int cnt;
		Nterms=0;
		buffer_idx=0;
		for (cnt = 0;cnt < MAXFILTERTERMS;cnt++){
			a[cnt]=0;
			b[cnt]=0;
			x[cnt]=0;
			y[cnt]=0;
		}
	}
	
		
/*
	Zero's out the history of the filter.
*/
	void M3DFilter2::Clear()
	{
		int cnt;
  
		buffer_idx=0;
		for (cnt = 0;cnt < MAXFILTERTERMS;cnt++){
			x[cnt]=0;
			y[cnt]=0;
		}
	}

/*
	Set the coefficients of the digital filter.
  
	for N coefficients associated with samples from time T to time T-N+1 the calc
  will be (a,b coefficiencs in C array notation):
  
	y(T) = x(T)*b[N-1]+x(T-1)*b[N-2]+..+x(T-N)*b[0] - 
	(y(T-1)*a[N-2]+y(T-2)*a[N-3]+...+y(T-N)*a[0])
 */
	int M3DFilter2::Coefficients(int N,mReal *A,mReal *B)
	{
		int cnt;
  
		if (N > MAXFILTERTERMS)
			return -1; //Filter is too small to take this many coefficients
  
		Nterms=N;
		for (cnt = 0;cnt < N;cnt++){
			a[cnt]=A[cnt];
			b[cnt]=B[cnt];
		}
		return 0; //success
	}
