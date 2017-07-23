/* Author: Traveler Hauptman Sep '09
   (c)2009 Meka Robotics
*/

#include "m3/toolbox/dfilter.h"
#include "m3rt/base/m3rt_def.h"
#include <cmath>
#include <iostream>


namespace m3 {
using namespace std;
using namespace m3rt;
using namespace Eigen;

void M3SensorFilter::Step ( mReal qraw, mReal qdotraw ) {
    x=0;
    xdot=0;
    xdotdot=0;
    if ( type==NONE ) {
        x=qraw;
        xdot=0;
        xdotdot=0;
        }
    if ( type==DF_CHAIN ) {
        x=x_df.Step ( qraw );
        xdot=xdot_df.Step ( qraw );
        xdotdot=xdotdot_df.Step ( xdot );
        }
    if ( type==DF_APERIODIC ) {
        x=x_df.Step ( qraw );
        xdot=xdot_df.Step ( qdotraw );
        xdotdot=xdotdot_df.Step ( qdotraw );
        }
    if ( type==POLY_LEAST_SQUARES ) {
        pls.Step ( qraw );
        x=pls.GetVal();
        xdot=pls.GetValDot();
        xdotdot=pls.GetValDotDot();
        }
    if ( type==DF_POLY_LEAST_SQUARES ) {
        pls.Step ( x_df.Step ( qraw ) );
        x=pls.GetVal();
        xdot=pls.GetValDot();
        xdotdot=pls.GetValDotDot();
        }

    }

void M3SensorFilter::Reset() {
    x_df.Clear();
    xdot_df.Clear();
    xdotdot_df.Clear();
    }
    
bool M3SensorFilter::ReadConfig ( const YAML::Node & doc ) {
    string t;
    doc["type"] >> t;
    if ( t.compare ( "none" ) ==0 ) type=NONE;
    if ( t.compare ( "df_chain" ) ==0 ) type=DF_CHAIN;
    if ( t.compare ( "poly_least_squares" ) ==0 ) type=POLY_LEAST_SQUARES;
    if ( t.compare ( "df_poly_least_squares" ) ==0 ) type=DF_POLY_LEAST_SQUARES;
    if ( t.compare ( "df_aperiodic" ) ==0 ) type=DF_APERIODIC;

    if ( type==POLY_LEAST_SQUARES||type==DF_POLY_LEAST_SQUARES ) {
        int num_data_pts, order;
        mReal weighting;
        doc["pls"]["order"] >> order;
        doc["pls"]["num_data_pts"] >> num_data_pts;
        doc["pls"]["weighting"] >> weighting;
        pls.Init ( order,num_data_pts,1.0/RT_TASK_FREQUENCY,weighting );
        }
    if ( type==DF_POLY_LEAST_SQUARES ) {
        try {
            x_df.ReadConfig ( doc["x_df"] );
            }
        catch ( ... ) {
		M3_ERR("x_df not found in doc\n");
		return false;
	}
        }
    if ( type==DF_CHAIN || type==DF_APERIODIC ) {
        try {
            x_df.ReadConfig ( doc["x_df"] );
            }
        catch ( ... ) {
		M3_ERR("x_df not found in doc\n");
		return false;
	}
        try {
            xdot_df.ReadConfig ( doc["xdot_df"] );
            }
        catch ( ... ) {
		M3_ERR("xdot_df not found in doc\n");
		return false;
	}
        try {
            xdotdot_df.ReadConfig ( doc["xdotdot_df"] );
            }
        catch ( ... ) {
		M3_ERR("xdotdot_df not found in doc\n");
		return false;
	}
        }
    return true;
    }

bool M3JointFilter::ReadConfig ( const YAML::Node & doc ) {
    string t;
	try {
    doc["type"] >> t;
    if ( t.compare ( "none" ) ==0 ) type=NONE;
    if ( t.compare ( "df_chain" ) ==0 ) type=DF_CHAIN;
    if ( t.compare ( "poly_least_squares" ) ==0 ) type=POLY_LEAST_SQUARES;
    if ( t.compare ( "df_poly_least_squares" ) ==0 ) type=DF_POLY_LEAST_SQUARES;
    if ( t.compare ( "df_aperiodic" ) ==0 ) type=DF_APERIODIC;

    if ( type==POLY_LEAST_SQUARES||type==DF_POLY_LEAST_SQUARES ) {
        int num_data_pts, order;
        mReal weighting;
        doc["pls"]["order"] >> order;
        doc["pls"]["num_data_pts"] >> num_data_pts;
        doc["pls"]["weighting"] >> weighting;
        pls.Init ( order,num_data_pts,1.0/RT_TASK_FREQUENCY,weighting );
        }
    if ( type==DF_POLY_LEAST_SQUARES ) {
        try {
            x_df.ReadConfig ( doc["theta_df"] );
            }
        catch ( ... ) {
		M3_ERR("theta_df not found in doc\n");
		return false;
	}
        }
    if ( type==DF_CHAIN || type==DF_APERIODIC ) {
        try {
            x_df.ReadConfig ( doc["theta_df"] );
            }
        catch ( ... ) {
		M3_ERR("theta_df not found in doc\n");
		return false;
	}
        try {
            xdot_df.ReadConfig ( doc["thetadot_df"] );
            }
        catch ( ... ) {
		M3_ERR("thetadot_df not found in doc\n");
		return false;
	}
        try {
            xdotdot_df.ReadConfig ( doc["thetadotdot_df"] );
            }
        catch ( ... ) {
		M3_ERR("thetadotdot_df not found in doc\n");
		return false;
	}
        }
	}catch(...){
		return false;
	}
	return true;
}

void M3PolyLeastSquares::Init ( int poly_order, int num_data_pts, mReal period, mReal weighting ) {
    n=poly_order;
    T=period;
    weight=weighting;
    m=num_data_pts;
    qy = VectorXf::Zero ( m );
    wy = VectorXf::Zero ( m );
    w = MatrixXf::Zero ( m,m );
    M = MatrixXf::Zero ( m,n+1 );
    MM = MatrixXf::Zero ( m,n+1 );
    for ( int i=0; i<m; i++ ) {
        for ( int j=0; j< ( n+1 ); j++ ) {
            M ( i,j ) = pow ( T* ( double ) i, ( double ) j );
            }
        w ( i,i ) = pow ( ( double ) ( m-i ),weight ) /pow ( ( double ) m,weight );
        }
    /*cout<<M;
    //M3_INFO("M %d %d\n",M.rows(),M.cols());
    cout<<w;
    //M3_INFO("w %d %d\n",M.rows(),w.cols());
    //M = w*M;
    for (int i=0; i<m; i++)
    {
    for(int j=0; j<(n+1); j++)
    {
    for (int k=0; k<m; k++)
    {
    MM(i,j) += w(i,k)*M(k,j);
    }
    }
    }
    M=MM;
    cout<<M;*/
    M = w*M;
    qr.compute ( M );
    //std::cout<<"M:"<<M<<" qr:"<<qr.matrixQR()<<std::cout;
    //Q = qr.matrixQ();
    //R = qr.matrixR();
    x = VectorXf::Zero ( n+1 );
    }

void M3PolyLeastSquares::Step ( mReal new_qy ) {

    for ( int i=0; i< ( m-1 ); i++ ) {
        qy[m-1-i] = qy[m-i-2];
        }

    qy[0] = new_qy;
    wy = w*qy;
    // Eigen3 support
    x = qr.solve ( wy );
    //std::cout<<"wy:"<<wy<<" x:"<<x<<std::endl;
    //x = R.marked<Eigen::UpperTriangular>().solveTriangular((Q.transpose()*wy));

    y_smooth = x[0];
    ydot_smooth = -x[1];
    ydotdot_smooth =  2.*x[2];

    return;
    }



bool M3DFilter::ReadConfig ( const YAML::Node & doc ) {
	string t;
    try {
        doc["type"] >> t;
        if ( t.compare ( "butterworth" ) ==0 ) {
            type = BUTTERWORTH;
            doc["order"] >> order;
            doc["cutoff_freq"] >> cutoff_freq;
            Butterworth_Filter();
            }
        if ( t.compare ( "diff_butterworth" ) ==0 ) {
            type = DIFF_BUTTERWORTH;
            doc["order"] >> order;
            doc["cutoff_freq"] >> cutoff_freq;
            Diff_Butterworth_Filter();
            }
        if ( t.compare ( "least_squares_estimate" ) ==0 ) {
            type = LEAST_SQUARES_ESTIMATE;
            doc["N"] >> Nterms;
            doc["cutoff_freq"] >> cutoff_freq;
            Least_Squares_Estimate();
            }
        if ( t.compare ( "identity" ) ==0 || t.compare ( "none" ) ==0 ) {
            type = IDENTITY;
	    Nterms = 1;
            Identity_Filter();
            }
        if ( t.compare ( "average" ) ==0 ) {
            type = AVERAGE;
            doc["N"] >> Nterms;
            Average_Filter();
            }
        }
    catch ( YAML::Exception e ) {
        //M3_ERR ( "DFilter error while reading config (type:%s) : %s\n",t.c_str(),e.what() );
        return false;
        }
    return true;
    }
void M3DFilter::SetType ( string t ) {
    if ( t.compare ( "butterworth" ) ==0 ) {
        type = BUTTERWORTH;
        UpdateFilter();
        return;
        }
    if ( t.compare ( "diff_butterworth" ) ==0 ) {
        type = DIFF_BUTTERWORTH;
        UpdateFilter();
        return;
        }
    if ( t.compare ( "least_squares_estimate" ) ==0 ) {
        type = LEAST_SQUARES_ESTIMATE;
        UpdateFilter();
        return;
        }
    if ( t.compare ( "identity" ) ==0 || t.compare ( "none" ) ==0 ) {
        type = IDENTITY;
        UpdateFilter();
        return;
        }
    if ( t.compare ( "average" ) ==0 ) {
        type = AVERAGE;
        UpdateFilter();
        return;
        }
    }
const char * M3DFilter::GetType () {

    switch ( type ) {
        case BUTTERWORTH:
            return "butterworth";
        case DIFF_BUTTERWORTH:
            return "diff_butterworth";
        case AVERAGE:
            return "average";
        case LEAST_SQUARES_ESTIMATE:
            return "least_squares_estimate";
        case IDENTITY:
            return "identity";
	case NONE:
            return "none";
        default:
            M3_ERR ( "M3DFilter Get error: Not a valid type.\n" );
            return "none";
        }
    }
/*Dump filter info to stdout. */
void M3DFilter::Dump() {
    std::cout << "terms:" << Nterms << "\n";
    std::cout << "Index:" << buffer_idx << "\n";
    for ( int cnt = 0; cnt < Nterms; cnt++ ) {
        std::cout << "a: " << _a[cnt] << " b: " << _b[cnt] << " x: " << _x[cnt] << " y: " << _y[cnt] << "\n" ;
        }
    }

/*
	Zero's out the history of the filter.
*/
void M3DFilter::Clear() {
    buffer_idx=0;
    for ( int cnt = 0; cnt < MAXFILTERTERMS; cnt++ ) {
        _x[cnt]=0.0;
        _y[cnt]=0.0;
        _a[cnt]=0.0;
        _b[cnt]=0.0;
        }
    }

/*
	Set the coefficients of the digital filter.

	for N coefficients associated with samples from time T to time T-N+1 the calc
  will be (a,b coefficiencs in C array notation):

	y(T) = x(T)*b[N-1]+x(T-1)*b[N-2]+..+x(T-N)*b[0] -
	(y(T-1)*a[N-2]+y(T-2)*a[N-3]+...+y(T-N)*a[0])
 */
/*int M3DFilter::Coefficients(int N,std::vector<mReal>& A, std::vector<mReal>& B)
{
	if (N > MAXFILTERTERMS)
		return -1; //Filter is too small to take this many coefficients
	Nterms=N;
	for (int cnt = 0;cnt < N;cnt++){
		_a[cnt]=A[cnt];
		_b[cnt]=B[cnt];
	}

	return 0; //success
}*/

void M3DFilter::Average_Filter() {
    //std::vector<mReal> a(MAXFILTERTERMS,0.0);
    //std::vector<mReal> b(MAXFILTERTERMS,0.0);

    for ( int i=0; i<Nterms; i++ ) {
        _b[i]=1.0/ ( mReal ) Nterms;
        _a[i]=0.0;
        }
    //Coefficients(N,a,b);
    }
void M3DFilter::Identity_Filter() {
    //std::vector<mReal> a(1,0.0);
    //std::vector<mReal> b(1,1.0);
    _b[0] = 1.0;
    _a[0] = 0.0;
    //Coefficients(1,a,b);
    }

mReal M3DFilter::Step ( mReal x_0 ) {
	mReal yt=0.0;
    switch ( type ) {
        case BUTTERWORTH:
            return Step ( x_0,order+1 );
        case DIFF_BUTTERWORTH:
            return Step ( x_0,order+1 );
        case AVERAGE:
            return Step ( x_0,Nterms );
        case LEAST_SQUARES_ESTIMATE:
            return Step ( x_0,Nterms );
        case IDENTITY:
	    return x_0;
        default:
            M3_ERR ( "M3DFilter Step error: Not a valid type.\n" );
            break;
        }
    return 0.0;
    }
void M3DFilter::UpdateFilter ( ) {
    switch ( type ) {
        case NONE:
            Identity_Filter();
            break;
        case BUTTERWORTH:
            Butterworth_Filter();
            break;
        case DIFF_BUTTERWORTH:
            Diff_Butterworth_Filter();
            break;
        case AVERAGE:
            Average_Filter();
            break;
        case LEAST_SQUARES_ESTIMATE:
            Least_Squares_Estimate();
            break;
        case IDENTITY:
            Identity_Filter();
            break;
        default:
            M3_ERR ( "M3DFilter UpdateParams error: Not a valid type.\n" );
            break;
        }
    return;
    }

/*
	Evaluates the digital filter, starting with the oldest values in the history
	and working towards the most recent.
*/
mReal M3DFilter::Step ( mReal x_0,int N ) {

    mReal retval=0.0;
    int i=0, start_idx=0;

    _x[buffer_idx] = x_0;
    _y[buffer_idx] = 0.0;

    start_idx = buffer_idx - N + MAXFILTERTERMS + 1; //Precalc the index where the history data starts in the buffer.

    for ( int n = 0; n < N; n++ ) {
        i = ( start_idx + n ) % ( MAXFILTERTERMS );
        _y[buffer_idx] +=  _b[n]*_x[i] - _a[n]*_y[i];
        //}

        }

    retval = _y[buffer_idx];

    buffer_idx = ( buffer_idx+1 ) % ( MAXFILTERTERMS );

    return retval;
    }



/*
	Filter coefficients from "Velocity estimation using quantized measurements"
	Stephen M Phillips & Michael S Branicky

	N=2 is the same as a euler backward difference.
*/
void M3DFilter::Least_Squares_Estimate() {
    //std::vector<mReal> a(8,0.0);
    //std::vector<mReal> b(8,0.0);
    mReal scale = 1/T;
    if ( Nterms==2 ) {
        _b[1] = scale;
        _b[0] = -scale;
        }
    else if ( Nterms==4 ) {
        _b[3] = 0.3*scale;
        _b[2] = 0.1*scale;
        _b[1] = -0.1*scale;
        _b[0] = -0.3*scale;
        }
    else if ( Nterms==8 ) {
        _b[7] = 0.0833*scale;
        _b[6] = 0.0595*scale;
        _b[5] = 0.0357*scale;
        _b[4] = 0.0119*scale;
        _b[3] = -0.0119*scale;
        _b[2] = -0.357*scale;
        _b[1] = -0.0595*scale;
        _b[0] = -0.0833*scale;
        }
    else {
        M3_INFO ( "Incorrect M3DFilter configuration for Least_Squares_Estimate\n" );
        }
    //Coefficients(N,a,b);
    }


/*
	Creates a digitized butterworth filter using the
	bilinear transform. Oppenheim & Schafer pp450 (2nd ed)

	order: the order of the filter. 1, 2 or 3.
	cutoff_freq: cutoff frequency of the lowpass filter
	sample_period: time between samples.
*/
void M3DFilter::Butterworth_Filter() {
    //mReal T = sample_period; //Shorthand for clearer code.
    mReal scale=1.0; //Scaling factor (really the a[1] value which everything gets divided by)

    mReal pi = 3.14159625;

    //std::vector<mReal> a(4,0.0);
    //std::vector<mReal> b(4,0.0);

    mReal wo = cutoff_freq*2.0*pi; //Omega naught (rad/sec)
    mReal w = ( 2./T ) *atan2 ( wo*T,2. ); //Omega cutoff (frequency warping from analog to digital domain)
    if ( order== 1 ) {
        mReal st = 2/T/w;
        scale = 1./ ( 1.+st );

        _a[1] = 0.0;
        _a[0] = ( 1.-st ) *scale;

        _b[1] = scale;
        _b[0] = scale;
        }
    else if ( order == 2 ) {
        mReal st = 2/T/w;
        scale = 1/ ( 1+sqrt ( 2.0 ) *st+st );

        _a[2] = 0.0;
        _a[1] = ( 2.-2.*st ) * scale;
        _a[0] = ( 1.-static_cast<mReal> ( sqrt ( 2.0 ) ) *st+st ) * scale;

        _b[2] = scale;
        _b[1] = 2.*scale;
        _b[0] = scale;
        }
    else if ( order == 3 ) {
        mReal A = 2./ ( w*T );
        mReal p2 = 2.*A;
        mReal p3 = 2.*A*A;
        mReal p4 = A*A*A;

        scale = 1./ ( 1.+p2+p3+p4 );

        _a[3] = 0.;
        _a[2] = ( 3.+p2-p3-3.*p4 ) *scale;
        _a[1] = ( 3.-p2-p3+3.*p4 ) * scale;
        _a[0] = ( 1.-p2+p3-p4 ) * scale;

        _b[3] = scale;
        _b[2] = 3.*scale;
        _b[1] = 3.*scale;
        _b[0] = scale;

        }
    else {
        M3_INFO ( "Invalid order %d for Butterworth filter\n",order );
        return ; //
        }
    //Coefficients(order+1,a,b);
    }



/*
	Creates a digitized butterworth filter plus differentiator using the
	bilinear transform. Oppenheim & Schafer pp450 (2nd ed)

	order: the order of the filter. 1, 2 or 3.
	cutoff_freq: cutoff frequency of the lowpass filter
	sample_period: time between samples.
*/
void M3DFilter::Diff_Butterworth_Filter() {
    mReal scale=1.0; //Scaling factor (really the a[1] value which everything gets divided by)

    mReal pi = 3.14159625;

    mReal wo = cutoff_freq*2.0*pi; //Omega naught (rad/sec)
    mReal w = 2./T*atan2 ( wo*T,2. ); //Omega cutoff (frequency warping from analog to digital domain)

    if ( order == 1 ) {
        mReal st = 1/w;
        scale = 1/ ( T/2+st );

        _a[1] = 0.0;
        _a[0] = ( T/2-st ) *scale;

        _b[1] = scale;
        _b[0] = -scale;
        }
    else if ( order == 2 ) {
        mReal st = sqrt ( 2.0 ) /w;
        scale = 1/ ( st*st/T + st + T/2.0 );

        _a[2] = 0.0;
        _a[1] = ( T-2*st*st/T ) * scale;
        _a[0] = ( T/2 - st + st*st/T ) * scale;

        _b[2] = scale;
        _b[1] = 0.0;
        _b[0] = -scale;
        }
    else if ( order == 3 ) {
        mReal st = 2/w;
        mReal nd = st*st/T;

        scale = 1/ ( T+st+nd );

        _a[3] = 0.0;
        _a[2] = ( st-nd ) *scale;
        _a[1] = ( 3*T-st-nd ) * scale;
        _a[0] = ( nd-st ) * scale;

        _b[3] = scale;
        _b[2] = scale;
        _b[1] = -scale;
        _b[0] = -scale;
        }
    else {
        M3_INFO ( "Incorrect M3DFilter configuration for Diff_Butterworth_Filter\n" );
        }
    //Coefficients(order+1,a,b);
    }

mReal M3DiffAperiodic::Step ( mReal qraw, uint64_t timestamp ) {
    mReal vel ;
    if ( previous_timestamp != 0  && timestamp != 0 && previous_timestamp != timestamp )
        vel = ( qraw - previous_qraw ) / ( 1e-9*mReal ( ( timestamp - previous_timestamp ) ) );
    else
        vel = previous_vel;

    previous_qraw = qraw;
    previous_timestamp = timestamp;
    previous_vel = vel;

    return vel;
    }



} //Namespace
