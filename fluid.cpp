#include <highgui.h>
#include <stdio.h>
#include <cv.h>


#define IX(i,j) ((i)+(NX+2)*(j))
#define NX 512
#define NY 128
#define N 500
#define size (NX+2)*(NY+2)
#define SWAP(x0,x) {float *tmp=x0;x0=x;x=tmp;}
#define FOR_EACH_CELL for ( i=1 ; i<=NX ; i++ ) { for ( j=1 ; j<=NY ; j++ ) {
#define END_FOR }}




static float u[size], v[size], u_prev[size], v_prev[size];
static float dens[size], dens_prev[size]; 
static char objects[size];

IplImage* dens_image = cvCreateImage( cvSize(NX,NY), 8, 3);
IplImage* draw_image = cvCreateImage( cvSize(1024,256), 8, 3);
//IplImage* draw_image_col = cvCreateImage( cvSize(1024,256), 8, 3);


void set_bnd ( int b, float * x )
{
	int i;

	for ( i=1 ; i<=NY ; i++ ) {
		x[IX(0  ,i)] = b==1 ? -x[IX(1,i)] : x[IX(1,i)];
		//x[IX(NX+1,i)] = b==1 ? -x[IX(NX,i)] : x[IX(NX,i)];
	}

	for ( i=1 ; i<=NX ; i++ ) {
		x[IX(i,0  )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];x[IX(i,NY+1)] = b==2 ? -x[IX(i,NY)] : x[IX(i,NY)];
	}


	//x[IX(0  ,0  )] = 0.5*(x[IX(1,0  )]+x[IX(0  ,1)]);
	//x[IX(0  ,NY+1)] = 0.5*(x[IX(1,NY+1)]+x[IX(0  ,NY )]);
	//x[IX(NX+1,0  )] = 0.5*(x[IX(NX,0  )]+x[IX(NX+1,1)]);
	//x[IX(NX+1,NY+1)] = 0.5*(x[IX(NX,NY+1)]+x[IX(NX+1,NY )]);



    for (int i=1 ; i <= NX ; i++ ) {
      for (int j=1 ; j <= NY ; j++ ) {
	if ( objects[IX(i, j)] ) {
	  if ( b == 1 ) {
	    // inverse horizontal velocity at vertical object border
	    if (objects[IX(i-1, j)]==0) x[IX(i, j)] = -x[IX(i-1, j)];
	    if (objects[IX(i+1, j)]==0) x[IX(i, j)] = -x[IX(i+1, j)];
	  } 
	  else if ( b == 2 ) {
	    // inverse vertical velocity at horizontal object border
	    if (objects[IX(i, j-1)]==0) x[IX(i, j)] = -x[IX(i, j-1)];
	    if (objects[IX(i, j+1)]==0) x[IX(i, j)] = -x[IX(i, j+1)];
	  } 
	  else if (b == 0 ) {
	    // same density as active neighbour for egde-border, 
	    // average of two active neighbours for corner-border
	    int count = 0; 
	    float tmp = 0.0f;
	    x[IX(i, j)] = 0;
	    if (objects[IX(i-1, j)]==0) { tmp += x[IX(i-1, j)]; count++; }
	    if (objects[IX(i+1, j)]==0) { tmp += x[IX(i+1, j)]; count++; }
	    if (objects[IX(i, j-1)]==0) { tmp += x[IX(i, j-1)]; count++; }
	    if (objects[IX(i, j+1)]==0) { tmp += x[IX(i, j+1)]; count++; }
	    if( count == 0){
	      x[IX(i, j)] = 0; 
	    } else {
	      x[IX(i, j)] = tmp/count; 
	    }
	  }
	}
      }
    }



	//if(b==1){
	//for ( unsigned int y = NY/3; y < NY*2/3; y++) {
		//u_prev[IX(130,y)] = -0.01;
		//if( u_prev[IX(130,y)] > 0 )
			//u_prev[IX(130,y)] = -50*u_prev[IX(130,y)];
		//printf("%f\n", u_prev[IX(130,y)]);
	//}
	//}
	

}




void lin_solve ( int b, float * x, float * x0, float a, float c )
{
	int i, j, k;

	for ( k=0 ; k<20 ; k++ ) {
		FOR_EACH_CELL
			x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)]+x[IX(i+1,j)]+x[IX(i,j-1)]+x[IX(i,j+1)]))/c;
		END_FOR
		set_bnd ( b, x );
	}
}

void project (  float * u, float * v, float * p, float * div )
{
	int i, j;

	FOR_EACH_CELL
		//if(objects[IX(i,j)]) continue;
		div[IX(i,j)] = -0.5f*(u[IX(i+1,j)]-u[IX(i-1,j)]+v[IX(i,j+1)]-v[IX(i,j-1)])/N;
		p[IX(i,j)] = 0;
	END_FOR	
	set_bnd (  0, div ); set_bnd (  0, p );

	lin_solve (  0, p, div, 1, 4 );

	FOR_EACH_CELL
		//if(objects[IX(i,j)]) continue;
		u[IX(i,j)] -= 0.5f*N*(p[IX(i+1,j)]-p[IX(i-1,j)]);
		v[IX(i,j)] -= 0.5f*N*(p[IX(i,j+1)]-p[IX(i,j-1)]);
	END_FOR
	set_bnd (  1, u ); set_bnd (  2, v );// aqui
}







void add_source( float* x, float* s, float dt )
{
	int i;
	for ( i = 0 ; i<size ; i++ ) 
		x[i] += dt*s[i];
}



void diffuse ( int b, float * x, float * x0, float diff, float dt )
{
	float a=dt*diff*NX*NY;
	lin_solve (  b, x, x0, a, 1+4*a );
}




void advect ( int b, float * d, float * d0, float * u, float * v, float dt )
{
	int i, j, i0, j0, i1, j1;
	float x, y, s0, t0, s1, t1, dt0;

	dt0 = dt*(NX+NY)*0.5;
	FOR_EACH_CELL
		//if(objects[IX(i,j)]) continue;
		x = i-dt0*u[IX(i,j)]; y = j-dt0*v[IX(i,j)];
		if (x<0.5f) x=0.5f; if (x>NX+0.5f) x=NX+0.5f; i0=(int)x; i1=i0+1;
		if (y<0.5f) y=0.5f; if (y>NY+0.5f) y=NY+0.5f; j0=(int)y; j1=j0+1;
		s1 = x-i0; s0 = 1-s1; t1 = y-j0; t0 = 1-t1;
		d[IX(i,j)] = s0*(t0*d0[IX(i0,j0)]+t1*d0[IX(i0,j1)])+
					 s1*(t0*d0[IX(i1,j0)]+t1*d0[IX(i1,j1)]);
	END_FOR
	set_bnd (  b, d );
}








void dens_step ( float * x, float * x0,  float * u, float * v, float diff, float dt )
{

	add_source ( x, x0, dt );
	SWAP ( x0, x ); diffuse (  0, x, x0, diff, dt );
	SWAP ( x0, x ); advect (  0, x, x0, u, v, dt );

	//for ( unsigned int i = 0; i < size; i++) {
		//x0[i] = 0;
	//}

}

void vel_step ( float * u, float * v, float * u0, float * v0, float visc, float dt )
{
	add_source (  u, u0, dt ); add_source (  v, v0, dt );
	SWAP ( u0, u ); diffuse (  1, u, u0, visc, dt );// aqui
	SWAP ( v0, v ); diffuse (  2, v, v0, visc, dt );// aqui
	project (  u, v, u0, v0 );
	SWAP ( u0, u ); SWAP ( v0, v );
	advect (  1, u, u0, u0, v0, dt ); advect (   2, v, v0, u0, v0, dt );// aqui
	project (  u, v, u0, v0 );

	for ( unsigned int i = 0; i < size; i++) {
		u0[i] = 0;
		v0[i] = 0;
	}



}

void draw_dens( float* dens ){

	for ( unsigned int x = 0; x < NX; x++) {
		for ( unsigned int y = 0; y < NY; y++) {
			//uchar* temp_ptr = &((uchar*)(dens_image->imageData +dens_image->widthStep*y))[x];
			uchar* temp_ptr = &((uchar*)(dens_image->imageData + dens_image->widthStep*y))[x*3];
			
			int scale = 150;
			if( scale * dens[IX(x,y)] < 255 ){
				temp_ptr[0] = scale*dens[IX(x,y)];
				temp_ptr[1] = 0.7*scale*dens[IX(x,y)];
				temp_ptr[2] = 0.7*scale*dens[IX(x,y)];
			} else {
				temp_ptr[0] = 255;
				temp_ptr[1] = 0.7*255;
				temp_ptr[2] = 0.7*255;
			}

			//*temp_ptr = 0;

			//if( objects[IX(x,y)] ) *temp_ptr = 255;



		}
	}


	cvResize(dens_image, draw_image);
	//cvCvtColor( draw_image,draw_image_col,CV_GRAY2BGR);


	float posx = 0.15;
	cvCircle( draw_image, cvPoint(posx*NX*2,NY), 19, CV_RGB(0,255,0),-1,CV_AA );
	


	//for ( unsigned int x = 0; x < NX; x+=10) {
		//for ( unsigned int y = 0; y < NY; y+=10) {
			//int scale = 100;
			//cvLine( draw_image, cvPoint(2*x, 2*y), cvPoint(2*x+scale*u[IX(x,y)],2*y-scale*v[IX(x,y)]), CV_RGB(255,255,255) );
		//}
	//}



	cvNamedWindow("density", 1);
	cvShowImage("density", draw_image);
	if( cvWaitKey(10) != -1 ) exit(0);

}


void get_from_UI( float* dens_prev, float* u_prev, float* v_prev ){

	for ( unsigned int i = 0; i < size; i++) {
		dens_prev[i] = 0;
		u_prev[i] = 0;
		v_prev[i] = 0;
	}

	//for ( unsigned int i = 0; i < NY; i+=15) {
		//dens_prev[IX(5,i)] = 150;
	//}
	
	for ( unsigned int i = NY/2-40; i <= NY/2+40; i+=10) {
		dens_prev[IX(5,i)] = 150;
	}

	for ( unsigned int y = 0; y < NY; y++) {
	for ( unsigned int x = 10; x <= 11; x++) {
		u_prev[IX(x,y)] = 10;
	}
	}


	//for ( unsigned int x = 0; x < NX; x++) {
		//for ( unsigned int y = 0; y < NY; y++) {
			//if( (x-40)*(x-40)+(y-NY/2)*(y-NY/2) < 300 ){
				//u_prev[IX(x,y)] = -50; 
			//}
		//}
	//}

	//for ( unsigned int y = 0; y < NY; y++) {
		//v_prev[IX(50,y)] = -5;
	//}

}



int main(int argc, const char *argv[])
{

	float visc = 0;
	float dt = 0.05;
	float diff = 0;

	float posx = 0.15;

	for ( unsigned int x = 0; x < NX; x++) {
		for ( unsigned int y = 0; y < NY; y++) {
			if( (x-NX*posx)*(x-NX*posx)+(y-NY/2)*(y-NY/2) < 100 ){
				objects[IX(x,y)] = 1;
			} else {
				objects[IX(x,y)] = 0;
			}
		}
	}
	
	//for ( unsigned int x = NX*posx; x < NX*posx+10; x++) {
		//for ( unsigned int y = NY/3; y < NY*2/3; y++) {
			//objects[IX(x,y)] = 1;
		//}
	//}


	while ( true )
	{
		get_from_UI ( dens_prev, u_prev, v_prev );
		vel_step (  u, v, u_prev, v_prev, visc, dt );
		dens_step (  dens, dens_prev, u, v, diff, dt );
		draw_dens (  dens_prev );
	}
	return 0;
}




