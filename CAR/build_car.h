#include <stdio.h>
#include <stdlib.h>
#include <stream.h>
#include <complex.h>
#include <time.h>
// Things that are used from the LEDA library
#include <LEDA/graph.h>
#include <LEDA/list.h>
#include <LEDA/point.h>
#include <LEDA/polygon.h>
#include <LEDA/ps_file.h>
#include <LEDA/random_source.h>
#include <LEDA/window.h>

#define PI 3.1415926
#define carL 10
#define MAXfi PI/4

window W2(625,625);
color col_white(255,255,255), col_gb(0,127,127), col_black(0,0,0), col_grey(127,127,127), col_red(255,0,0), col_green(0,127,0);

//------------------ car_move classes -----------------------
class car_u{
public:
	int v;
	double fi;
public:	
	friend ostream& operator<<(ostream& os, car_u u);
	friend istream& operator>>(istream& is, car_u &u);
	};

ostream& operator<<(ostream& os, car_u u)
{
	os << "Car Speed = " << u.v << ", Turnning Angle = " << u.fi;
	return os;
}

istream& operator>>(istream& is, car_u &u)
{
	is >> u.v >> u.fi;
	return is;
}

class car_q{
public:
	double x;
	double y;
	double cita;
public:	
	friend ostream& operator<<(ostream& os, car_q q);
	friend istream& operator>>(istream& is, car_q &q);
	};

ostream& operator<<(ostream& os, car_q q)
{
	os << "Car Position: x = " << q.x << ", y = " << q.y << ", cita = " << q.cita;
	return os;
}

istream& operator>>(istream& is, car_q &q)
{
	is >> q.x >> q.y >> q.cita;
	return is;
}

class car_move{
public:
	car_q q;
	car_u u;
public:	
	friend ostream& operator<<(ostream& os, car_move m);
	friend istream& operator>>(istream& is, car_move &m);
	};

ostream& operator<<(ostream& os, car_move m)
{
	os << "Car Position: x = " << (m.q).x << ", y = " << (m.q).y << ", cita = " << (m.q).cita << endl << "Car Speed = " << (m.u).v << ", Turnning Angle = " << (m.u).fi;
	return os;
}

istream& operator>>(istream& is, car_move &m)
{
	is >> (m.q).x >> (m.q).y >> (m.q).cita >> (m.u).v >> (m.u).fi;
	return is;
}
//=========================================================================

//---------------------------- draw Car ---------------------------------
polygon drawWheel(vector w, double a, vector T, double cita)
{
list<point> car_wheel;
polygon carWheel;

car_wheel.clear();

car_wheel.push(point(-1.5,-0.8));
car_wheel.push(point(-1.5,0.8));
car_wheel.push(point(1.5,0.8));
car_wheel.push(point(1.5,-0.8));

carWheel= polygon (car_wheel);
carWheel=carWheel.rotate(a);
carWheel=carWheel + w;
carWheel=carWheel + T;
carWheel=carWheel.rotate(point(T.xcoord(),T.ycoord()), cita);

W2.draw_filled_polygon(carWheel);

return(carWheel);
}

polygon drawCarBody(vector T, double cita)
{
list<point> car_body;
polygon carBody;

car_body.clear();

car_body.push(point(-2,-4));
car_body.push(point(-2,4));
car_body.push(point(12,4));
car_body.push(point(12,-4));

carBody= polygon (car_body);
carBody=carBody.rotate(cita);
carBody=carBody + T;

return(carBody);

}

polygon drawCar(car_move carM0, car_move carM1, color col_x)
{
int i;
polygon carBody;
array<segment> H(4), Hj(4);
array<vector> w(4);
double a[4]={0,0,0,0};
vector T, T0, T1;
double cita, cita0, cita1;
double fi0;

T0 = vector((carM0.q).x,(carM0.q).y);
cita0 = (carM0.q).cita;
T1 = vector((carM1.q).x,(carM1.q).y);
cita1 = (carM1.q).cita;

fi0 = (carM1.u).fi;
a[0] = fi0;
a[1] = fi0;

H[0]=segment(0,-2.5,0,2.5);
H[1]=segment(10,-2.5,10,2.5);
H[2]=segment(0,-0.8,10,-0.4);
H[3]=segment(0,0.8,10,0.4);

w[0]=vector(10,2.5);
w[1]=vector(10,-2.5);;
w[2]=vector(0,-2.5);
w[3]=vector(0,2.5);

double dcita;

dcita=cita1-cita0;
if (dcita>PI)
	dcita-=2*PI;
else if (dcita<-PI)
	dcita+=2*PI;
for (int j=0;j<2;j++){
   
   if (j>=1){
	W2.set_color(col_white);	
	W2.draw_polygon(carBody);
	for (i=0;i<4;i++)
		{
		W2.draw_segment(Hj[i]);
		drawWheel(w[i],a[i],T,cita);
		}

        }

	W2.set_color(col_x);	
	T=T0+j*(T1-T0)/1;
	cita=cita0+j*(dcita)/1;
	
	carBody=drawCarBody(T,cita);
	W2.draw_polygon(carBody);

	for (i=0;i<4;i++)
		{
		Hj[i]=H[i]+T;
		Hj[i]=Hj[i].rotate(point(T.xcoord(),T.ycoord()), cita);
		W2.draw_segment(Hj[i]);
		W2.set_color(col_black);	
		drawWheel(w[i],a[i],T,cita);
		W2.set_color(col_x);	
		}

	for (i=0;i<10000;i++)W2.set_color(col_black);
	if(T0==T1&&cita0==cita1)break;

	}

return(carBody);
}

int outBox(polygon P)
{
point v;

forall_vertices(v,P){
	if (v.xcoord()<0 || v.xcoord()>100 || v.ycoord()<0 || v.ycoord()>100){
		return(1);
		}
	}

return(0);
}	



