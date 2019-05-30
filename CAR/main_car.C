#include "obst.h"
#include "RRT_car.h"

extern window W2;
extern window W;

main()
{
GRAPH<car_q, car_u> RRT;
int k, k2;
list<car_move> carPath;
car_move carM0, carM1;
car_q qInit, qGoal;
car_u uInit;
list<polygon> obsts;

qInit.x=50;
qInit.y=50;
qInit.cita=0;
uInit.v=0;
uInit.fi=0;
carM0.q=qInit;
carM0.u=uInit;
carM1=carM0;

qGoal.x=50;
qGoal.y=40;
qGoal.cita=0;

W.display();
W2.display(-100,0);

W.draw_point(point(qInit.x, qInit.y));
W.draw_point(point(qGoal.x, qGoal.y));
drawCar(carM0, carM1, col_gb);
carM0.q=qGoal;
carM1=carM0;
drawCar(carM0, carM1, col_red);

obsts = obst();

RRT = Build_RRT(qInit, qGoal, obsts);
carPath = Car_Path(qInit, qGoal, RRT, obsts);

carM0 = carPath.pop();	 
while(!carPath.empty()){	
	carM1 = carPath.pop();	 
	drawCar(carM0, carM1, col_gb);
	carM0=carM1;
//	W2.read_mouse();
	}


k=W.get_mouse();
k2=W2.get_mouse();
while (k!=MOUSE_BUTTON(3)&&k2!=MOUSE_BUTTON(3)){
  k=W.get_mouse();
  k2=W2.get_mouse();
  } 

W2.close(); 
W.close(); 
}
