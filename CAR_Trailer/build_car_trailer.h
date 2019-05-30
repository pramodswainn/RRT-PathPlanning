#include <stdio.h>
#include <stdlib.h>
#include <stream.h>
#include <complex.h>
#include <math.h>
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
#define carL1 10
#define carL2 10
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
	double cita1;
	double cita2;
public:	
	friend ostream& operator<<(ostream& os, car_q q);
	friend istream& operator>>(istream& is, car_q &q);
	};

ostream& operator<<(ostream& os, car_q q)
{
	os << "Car Position: x = " << q.x << ", y = " << q.y << ", cita1 = " << q.cita1 << ", cita2 = " << q.cita2;
	return os;
}

istream& operator>>(istream& is, car_q &q)
{
	is >> q.x >> q.y >> q.cita1 >> q.cita2;
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
	os << m.q << endl << m.u;
	return os;
}

istream& operator>>(istream& is, car_move &m)
{
	is >> m.q >> m.u;
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

polygon drawTrailer(vector T, double cita)
{
list<point> trailer;
polygon carTr;

trailer.clear();

trailer.push(point(-14,-4));
trailer.push(point(-14,4));
trailer.push(point(-7,4));
trailer.push(point(-7,-4));

carTr= polygon (trailer);
carTr=carTr.rotate(cita);
carTr=carTr + T;

return(carTr);

}


polygon drawCar(car_move carM, list<polygon> obsts, color col_x)
{
int i;
polygon carBody, carTr;
array<segment> H(4), Hj(4), H2(4), Hj2(4);
array<vector> w(4), w2(2);
double a[4]={0,0,0,0}, a2[2]={0,0};
vector T;
double cita1, cita2;
double fi;
list_item it;
polygon obs;

T = vector((carM.q).x,(carM.q).y);
cita1 = (carM.q).cita1;
cita2 = (carM.q).cita2;

fi = (carM.u).fi;
a[0] = fi;
a[1] = fi;

H[0]=segment(0,-2.5,0,2.5);
H[1]=segment(10,-2.5,10,2.5);
H[2]=segment(0,-0.8,10,-0.4);
H[3]=segment(0,0.8,10,0.4);

w[0]=vector(10,2.5);
w[1]=vector(10,-2.5);;
w[2]=vector(0,-2.5);
w[3]=vector(0,2.5);

H2[0]=segment(0,0,-12,0);
H2[1]=segment(-12,2.5,-12,-2.5);
w2[0]=vector(-12,2.5);
w2[1]=vector(-12,-2.5);;


it=obsts.first();
while(it!=nil){
	obs=obsts.inf(it);
	W2.draw_filled_polygon(obs, col_grey);
	it=obsts.succ(it);
	}

W2.set_color(col_x);	

carBody=drawCarBody(T,cita1);
carTr=drawTrailer(T,cita1+cita2);
W2.draw_polygon(carBody);
W2.draw_polygon(carTr);

for (i=0;i<4;i++)
	{
	Hj[i]=H[i]+T;
	Hj[i]=Hj[i].rotate(point(T.xcoord(),T.ycoord()), cita1);
	W2.draw_segment(Hj[i]);
	W2.set_color(col_black);	
	drawWheel(w[i],a[i],T,cita1);
	W2.set_color(col_x);	
	}

for (i=0;i<2;i++){
	Hj2[i]=H2[i]+T;
	Hj2[i]=Hj2[i].rotate(point(T.xcoord(),T.ycoord()), cita1+cita2);
	W2.draw_segment(Hj2[i]);
	W2.set_color(col_black);	
	drawWheel(w2[i],a2[i],T,cita1+cita2);
	W2.set_color(col_x);	
	}


for (i=0;i<10000;i++)W2.set_color(col_black);


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



